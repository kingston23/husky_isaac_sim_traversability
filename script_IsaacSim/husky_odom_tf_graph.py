"""
Builds an ActionGraph for Husky odometry and TF in Isaac Sim.

Publishes:
    /odom       nav_msgs/Odometry
    /tf         odom -> base_link              (from ComputeOdometry)
    /tf         base_link -> wheels, lidar, imu  (from actual USD transforms)
    /clock      rosgraph_msgs/Clock

Usage: Window -> Script Editor -> paste -> Ctrl+Enter -> PLAY

IMPORTANT: if you also run diff_drive_controller with enable_odom_tf: true,
or an EKF that publishes odom -> base_link, set PUBLISH_ODOM_TF = False.
Two publishers of the same transform will corrupt the TF tree.
"""

import omni.graph.core as og
import usdrt.Sdf

# ============================ CONFIGURE ============================

ROBOT_PRIM = "/World/husky"
CHASSIS_PRIM = "/World/husky/base_link"     # prim with ArticulationRootAPI

GRAPH_PATH = "/World/HuskyOdomGraph"

ODOM_TOPIC = "/odom"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

# False if odom -> base_link is published by diff_drive_controller or an EKF
PUBLISH_ODOM_TF = True

# Links whose pose is published relative to base_link.
# base_footprint is NOT here because the importer merged it into base_link.
CHILD_LINKS = [
    "/World/husky/front_left_wheel",
    "/World/husky/front_right_wheel",
    "/World/husky/rear_left_wheel",
    "/World/husky/rear_right_wheel",
    "/World/husky/base_laser",
    "/World/husky/imu_link",
]

PUBLISH_CLOCK = True    # False if /clock is already published by the cmd_vel graph

# ===================================================================

keys = og.Controller.Keys

nodes = [
    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
    ("Context",        "isaacsim.ros2.bridge.ROS2Context"),
    ("ReadSimTime",    "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("ComputeOdom",    "isaacsim.core.nodes.IsaacComputeOdometry"),
    ("PublishOdom",    "isaacsim.ros2.bridge.ROS2PublishOdometry"),
    ("PublishTF",      "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
]
if PUBLISH_ODOM_TF:
    nodes.append(("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
if PUBLISH_CLOCK:
    nodes.append(("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"))

values = [
    ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(CHASSIS_PRIM)]),

    ("PublishOdom.inputs:topicName",      ODOM_TOPIC),
    ("PublishOdom.inputs:odomFrameId",    ODOM_FRAME),
    ("PublishOdom.inputs:chassisFrameId", BASE_FRAME),

    ("PublishTF.inputs:topicName",  "/tf"),
    ("PublishTF.inputs:parentPrim", [usdrt.Sdf.Path(CHASSIS_PRIM)]),
    ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(p) for p in CHILD_LINKS]),
]
if PUBLISH_ODOM_TF:
    values += [
        ("PublishOdomTF.inputs:topicName",     "/tf"),
        ("PublishOdomTF.inputs:parentFrameId", ODOM_FRAME),
        ("PublishOdomTF.inputs:childFrameId",  BASE_FRAME),
    ]
if PUBLISH_CLOCK:
    values.append(("PublishClock.inputs:topicName", "/clock"))

connections = [
    # tick
    ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
    ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),

    # ROS 2 context
    ("Context.outputs:context", "PublishOdom.inputs:context"),
    ("Context.outputs:context", "PublishTF.inputs:context"),

    # time
    ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),

    # odometry -> message
    ("ComputeOdom.outputs:position",        "PublishOdom.inputs:position"),
    ("ComputeOdom.outputs:orientation",     "PublishOdom.inputs:orientation"),
    ("ComputeOdom.outputs:linearVelocity",  "PublishOdom.inputs:linearVelocity"),
    ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
]

if PUBLISH_ODOM_TF:
    connections += [
        ("OnPlaybackTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
        ("Context.outputs:context",     "PublishOdomTF.inputs:context"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
        # The same pose that goes into /odom also goes into TF,
        # so the two are consistent by construction.
        ("ComputeOdom.outputs:position",    "PublishOdomTF.inputs:translation"),
        ("ComputeOdom.outputs:orientation", "PublishOdomTF.inputs:rotation"),
    ]

if PUBLISH_CLOCK:
    connections += [
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("Context.outputs:context",     "PublishClock.inputs:context"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
    ]

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: nodes,
        keys.SET_VALUES: values,
        keys.CONNECT: connections,
    },
)

print("=" * 60)
print(f"Graph built: {GRAPH_PATH}")
print(f"  /odom        odom_frame={ODOM_FRAME}  base_frame={BASE_FRAME}")
print(f"  odom->base_link TF: {'YES' if PUBLISH_ODOM_TF else 'NO (handled elsewhere)'}")
print(f"  base_link -> {len(CHILD_LINKS)} children")
print("Press PLAY.")
print("=" * 60)
