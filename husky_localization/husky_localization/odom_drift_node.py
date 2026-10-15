#!/usr/bin/env python3
"""
Odometry drift simulator.

Takes ground-truth odometry from Isaac Sim and produces a realistically
degraded copy, so that a dual-EKF setup has something to actually correct.

Why this exists:
    IsaacComputeOdometry gives a perfect pose. Feeding that into an EKF makes
    the filter look flawless while teaching you nothing -- the GPS correction
    never does any work. Real wheel odometry drifts without bound, and the
    whole point of the global EKF is to bound that drift.

How the drift is built:
    Errors are injected into the INCREMENTAL motion between two consecutive
    ground-truth samples, never into the absolute pose. The corrupted
    increments are then integrated into an independent pose that walks away
    from the truth exactly the way real odometry does.

    Three error sources, all of which real skid-steer platforms have:

      1. Systematic scale error   -- wheel radius and effective track width
                                     are never exactly what the URDF says.
                                     This is the dominant long-term error and
                                     it does NOT average out.
      2. Motion-proportional noise -- encoder quantisation, uneven ground.
      3. Lateral slip              -- skid-steer platforms slide sideways
                                     during turns; a differential-drive
                                     odometry model cannot see this at all.

    Turning is where skid-steer odometry falls apart, so the angular scale
    error and the rotation-induced translation noise are set high on purpose.

Frames:
    Input  : ground truth on the odom frame (from Isaac)
    Output : drifted odometry, same frames, different topic

    publish_tf defaults to False because the local EKF owns odom -> base_link.
    Set it True only if you want to inspect the raw drift in RViz without
    running an EKF.

Usage:
    ros2 run <your_package> odom_drift_node --ros-args -p use_sim_time:=true
or:
    python3 odom_drift_node.py --ros-args -p use_sim_time:=true

Reset the accumulated drift at any time:
    ros2 service call /odom_drift/reset std_srvs/srv/Empty
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw):
    """Returns (x, y, z, w) for a rotation about z."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomDrift(Node):

    def __init__(self):
        super().__init__("odom_drift")

        # ---------------- topics and frames ----------------
        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("output_topic", "/odom/noisy")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)

        # ---------------- systematic errors ----------------
        # These do NOT average out. They are the reason odometry alone can
        # never work over long distances.

        # 1.0 = perfect. 1.02 means the robot thinks it travelled 2% further
        # than it did -- e.g. wheel radius slightly overestimated.
        self.declare_parameter("linear_scale", 1.02)

        # Skid-steer platforms systematically UNDER-report rotation because
        # the effective track width is wider than the geometric one.
        # 0.93 = reports 93% of the true rotation.
        self.declare_parameter("angular_scale", 0.93)

        # ---------------- random errors ----------------
        # Standard deviations, all proportional to the amount of motion.
        # Naming: noise_<what is affected>_<what causes it>

        # translation error caused by translating (per metre)
        self.declare_parameter("noise_trans_trans", 0.02)
        # translation error caused by rotating (per radian)
        self.declare_parameter("noise_trans_rot", 0.05)
        # rotation error caused by rotating (per radian)
        self.declare_parameter("noise_rot_rot", 0.06)
        # rotation error caused by translating (per metre)
        self.declare_parameter("noise_rot_trans", 0.02)

        # Sideways slide during turns (per radian of rotation).
        # This is the skid-steer signature: a differential-drive odometry
        # model has no way to observe it.
        self.declare_parameter("lateral_slip", 0.04)

        self.declare_parameter("seed", 0)

        # ---------------- read them ----------------
        p = self.get_parameter
        self.in_topic = p("input_topic").value
        self.out_topic = p("output_topic").value
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.publish_tf = bool(p("publish_tf").value)

        self.lin_scale = float(p("linear_scale").value)
        self.ang_scale = float(p("angular_scale").value)

        self.n_tt = float(p("noise_trans_trans").value)
        self.n_tr = float(p("noise_trans_rot").value)
        self.n_rr = float(p("noise_rot_rot").value)
        self.n_rt = float(p("noise_rot_trans").value)
        self.slip = float(p("lateral_slip").value)

        seed = int(p("seed").value)
        self.rng = random.Random(seed if seed else None)

        # ---------------- state ----------------
        self.prev_gt = None          # previous ground-truth (x, y, yaw)
        self.x = 0.0                 # drifted pose
        self.y = 0.0
        self.yaw = 0.0
        self.initialised = False

        # Accumulated variance. Odometry uncertainty grows without bound,
        # and the EKF needs to know that or it will over-trust the odometry.
        self.var_x = 0.0
        self.var_y = 0.0
        self.var_yaw = 0.0

        self.total_distance = 0.0

        # ---------------- ROS plumbing ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(Odometry, self.in_topic, self.on_odom, qos)
        self.pub = self.create_publisher(Odometry, self.out_topic, qos)
        self.srv = self.create_service(Empty, "~/reset", self.on_reset)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.report_timer = self.create_timer(10.0, self.report)

        self.get_logger().info(
            f"Odom drift: {self.in_topic} -> {self.out_topic} | "
            f"lin_scale={self.lin_scale} ang_scale={self.ang_scale} | "
            f"publish_tf={self.publish_tf}"
        )

    # ------------------------------------------------------------------
    def on_reset(self, request, response):
        self.prev_gt = None
        self.initialised = False
        self.var_x = self.var_y = self.var_yaw = 0.0
        self.total_distance = 0.0
        self.get_logger().info("Drift reset.")
        return response

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        gt_x = msg.pose.pose.position.x
        gt_y = msg.pose.pose.position.y
        gt_yaw = yaw_from_quat(msg.pose.pose.orientation)

        # First sample: start the drifted pose exactly at ground truth.
        if not self.initialised:
            self.x, self.y, self.yaw = gt_x, gt_y, gt_yaw
            self.prev_gt = (gt_x, gt_y, gt_yaw)
            self.initialised = True
            return

        px, py, pyaw = self.prev_gt

        # ---- true increment, expressed in the previous body frame ----
        # Working in the body frame avoids the atan2 singularity that the
        # classic rot1/trans/rot2 decomposition has when the robot is
        # standing still or reversing.
        dx_w = gt_x - px
        dy_w = gt_y - py
        c, s = math.cos(pyaw), math.sin(pyaw)
        dx_b = c * dx_w + s * dy_w        # forward
        dy_b = -s * dx_w + c * dy_w       # left
        dyaw = wrap_pi(gt_yaw - pyaw)

        self.prev_gt = (gt_x, gt_y, gt_yaw)

        trans = math.hypot(dx_b, dy_b)
        abs_yaw = abs(dyaw)
        self.total_distance += trans

        # ---- corrupt the increment ----
        sigma_trans = self.n_tt * trans + self.n_tr * abs_yaw
        sigma_yaw = self.n_rr * abs_yaw + self.n_rt * trans
        sigma_lat = self.slip * abs_yaw

        dx_hat = dx_b * self.lin_scale + self.rng.gauss(0.0, sigma_trans)
        dy_hat = dy_b + self.rng.gauss(0.0, sigma_lat)
        dyaw_hat = dyaw * self.ang_scale + self.rng.gauss(0.0, sigma_yaw)

        # ---- integrate into the drifted pose ----
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        self.x += dx_hat * cy - dy_hat * sy
        self.y += dx_hat * sy + dy_hat * cy
        self.yaw = wrap_pi(self.yaw + dyaw_hat)

        # ---- accumulate uncertainty ----
        # Rotational error couples into position over distance travelled,
        # which is why heading error dominates long-run odometry drift.
        self.var_yaw += sigma_yaw ** 2
        pos_var_step = sigma_trans ** 2 + sigma_lat ** 2 + self.var_yaw * (trans ** 2)
        self.var_x += pos_var_step
        self.var_y += pos_var_step

        # ---- build the output message ----
        out = Odometry()
        out.header.stamp = msg.header.stamp      # keep the source timestamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame

        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(self.yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        # Velocities are measured instantaneously, so they do not accumulate
        # error the way pose does. We apply the same systematic scale plus a
        # little noise, and leave it at that.
        vx = msg.twist.twist.linear.x * self.lin_scale
        vyaw = msg.twist.twist.angular.z * self.ang_scale
        out.twist.twist.linear.x = vx + self.rng.gauss(0.0, 0.01)
        out.twist.twist.linear.y = 0.0
        out.twist.twist.angular.z = vyaw + self.rng.gauss(0.0, 0.02)

        # Pose covariance grows without bound; twist covariance stays fixed.
        # A floor is applied so the EKF never sees a zero variance.
        big = 1e6
        out.pose.covariance = [
            max(self.var_x, 1e-4), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, max(self.var_y, 1e-4), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, big, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, big, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, big, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, max(self.var_yaw, 1e-5),
        ]
        out.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, big, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, big, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, big, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.02,
        ]

        self.pub.publish(out)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    def report(self):
        """Periodic report of how far the drifted pose has wandered."""
        if not self.initialised or self.prev_gt is None:
            return
        gx, gy, gyaw = self.prev_gt
        err_pos = math.hypot(self.x - gx, self.y - gy)
        err_yaw = math.degrees(abs(wrap_pi(self.yaw - gyaw)))
        pct = (100.0 * err_pos / self.total_distance) if self.total_distance > 0.1 else 0.0
        self.get_logger().info(
            f"travelled {self.total_distance:6.1f} m | "
            f"position error {err_pos:5.2f} m ({pct:4.1f}%) | "
            f"heading error {err_yaw:5.1f} deg"
        )


def main():
    rclpy.init()
    node = OdomDrift()
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
