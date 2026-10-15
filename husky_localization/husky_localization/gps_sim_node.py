#!/usr/bin/env python3
"""
Simulated GPS for Isaac Sim.

Takes a ground-truth pose (nav_msgs/Odometry from IsaacComputeOdometry),
converts local ENU coordinates to WGS84 and publishes a noisy NavSatFix.

The noise model has two components, because real GPS has both:
  - white noise   (instantaneous imprecision, changes every sample)
  - slow random walk (ionospheric / multipath drift, over minutes)

Without the random walk the EKF gets an unrealistically good GPS and the
whole localisation setup looks better than it will ever be in the field.

Usage:
    ros2 run <your_package> gps_sim_node --ros-args -p use_sim_time:=true
or directly:
    python3 gps_sim_node.py --ros-args -p use_sim_time:=true
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

# WGS84
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


class GpsSim(Node):

    def __init__(self):
        super().__init__("gps_sim")

        # ---------------- parameters ----------------
        self.declare_parameter("ground_truth_topic", "/odom")
        self.declare_parameter("fix_topic", "/gps/fix")
        self.declare_parameter("frame_id", "gps_link")

        # Datum: the point on Earth corresponding to the odom frame origin.
        # SET THIS TO YOUR OWN LOCATION.
        self.declare_parameter("datum_latitude", 45.8150)
        self.declare_parameter("datum_longitude", 15.9819)
        self.declare_parameter("datum_altitude", 120.0)

        self.declare_parameter("publish_rate", 5.0)      # Hz, typical for GPS

        # noise
        self.declare_parameter("noise_stddev_xy", 0.8)   # m, white noise
        self.declare_parameter("noise_stddev_z", 1.6)    # m, altitude is always worse
        self.declare_parameter("drift_stddev", 0.05)     # m per sample, random walk
        self.declare_parameter("drift_limit", 2.5)       # m, drift bound

        # antenna offset relative to base_link (in the base_link frame)
        self.declare_parameter("antenna_offset_x", 0.0)
        self.declare_parameter("antenna_offset_y", 0.0)
        self.declare_parameter("antenna_offset_z", 0.3)

        self.declare_parameter("seed", 0)

        p = self.get_parameter
        self.gt_topic = p("ground_truth_topic").value
        self.fix_topic = p("fix_topic").value
        self.frame_id = p("frame_id").value

        self.lat0 = float(p("datum_latitude").value)
        self.lon0 = float(p("datum_longitude").value)
        self.alt0 = float(p("datum_altitude").value)

        self.rate = float(p("publish_rate").value)
        self.n_xy = float(p("noise_stddev_xy").value)
        self.n_z = float(p("noise_stddev_z").value)
        self.drift_sd = float(p("drift_stddev").value)
        self.drift_lim = float(p("drift_limit").value)

        self.off = (
            float(p("antenna_offset_x").value),
            float(p("antenna_offset_y").value),
            float(p("antenna_offset_z").value),
        )

        seed = int(p("seed").value)
        self.rng = random.Random(seed if seed else None)

        # ---------------- earth radii at the datum ----------------
        lat0_rad = math.radians(self.lat0)
        s = math.sin(lat0_rad)
        denom = 1.0 - WGS84_E2 * s * s
        # meridian and normal radius -- more accurate than a flat approximation
        self.R_meridian = WGS84_A * (1.0 - WGS84_E2) / (denom ** 1.5)
        self.R_normal = WGS84_A / math.sqrt(denom)
        self.cos_lat0 = math.cos(lat0_rad)

        # ---------------- state ----------------
        self.drift = [0.0, 0.0, 0.0]
        self.last_gt = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(Odometry, self.gt_topic, self.on_gt, qos)
        self.pub = self.create_publisher(NavSatFix, self.fix_topic, qos)
        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)

        self.get_logger().info(
            f"GPS sim: {self.gt_topic} -> {self.fix_topic} @ {self.rate} Hz | "
            f"datum {self.lat0:.6f}, {self.lon0:.6f}"
        )

    # ------------------------------------------------------------------
    def on_gt(self, msg: Odometry):
        self.last_gt = msg

    # ------------------------------------------------------------------
    def on_timer(self):
        if self.last_gt is None:
            self.get_logger().warn(
                f"Waiting for data on {self.gt_topic}", throttle_duration_sec=5.0
            )
            return

        pos = self.last_gt.pose.pose.position
        q = self.last_gt.pose.pose.orientation

        # yaw from the quaternion, used to rotate the antenna offset
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        ox, oy, oz = self.off
        ax = pos.x + ox * math.cos(yaw) - oy * math.sin(yaw)
        ay = pos.y + ox * math.sin(yaw) + oy * math.cos(yaw)
        az = pos.z + oz

        # random walk with a soft bound -- unbounded drift runs away to infinity
        for i in range(3):
            self.drift[i] += self.rng.gauss(0.0, self.drift_sd)
            self.drift[i] = max(-self.drift_lim, min(self.drift_lim, self.drift[i]))

        east = ax + self.drift[0] + self.rng.gauss(0.0, self.n_xy)
        north = ay + self.drift[1] + self.rng.gauss(0.0, self.n_xy)
        up = az + self.drift[2] + self.rng.gauss(0.0, self.n_z)

        lat = self.lat0 + math.degrees(north / self.R_meridian)
        lon = self.lon0 + math.degrees(east / (self.R_normal * self.cos_lat0))
        alt = self.alt0 + up

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id

        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt

        # The covariance must reflect the TOTAL uncertainty (noise + drift),
        # otherwise the EKF trusts the GPS too much and the pose jumps around.
        var_xy = self.n_xy ** 2 + (self.drift_lim / 2.0) ** 2
        var_z = self.n_z ** 2 + (self.drift_lim) ** 2
        fix.position_covariance = [
            var_xy, 0.0, 0.0,
            0.0, var_xy, 0.0,
            0.0, 0.0, var_z,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(fix)


def main():
    rclpy.init()
    node = GpsSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
