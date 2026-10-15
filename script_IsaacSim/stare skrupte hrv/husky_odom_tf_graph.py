"""
Gradi ActionGraph za odometriju i TF Huskyja u Isaac Simu.

Objavljuje:
    /odom       nav_msgs/Odometry
    /tf         odom -> base_link        (iz ComputeOdometry)
    /tf         base_link -> kotaci, lidar, imu   (iz stvarnih USD transformacija)
    /clock      rosgraph_msgs/Clock

Pokretanje: Window -> Script Editor -> zalijepi -> Ctrl+Enter -> PLAY

VAZNO: ako paralelno vrtis diff_drive_controller s enable_odom_tf: true,
postavi PUBLISH_ODOM_TF = False. Dva izdavaca iste transformacije rusi TF stablo.
"""

import omni.graph.core as og
import usdrt.Sdf

# ============================ PODESI ============================

ROBOT_PRIM = "/World/husky"
CHASSIS_PRIM = "/World/husky/base_link"     # prim s ArticulationRootAPI

GRAPH_PATH = "/World/HuskyOdomGraph"

ODOM_TOPIC = "/odom"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

# False ako odom->base_link objavljuje diff_drive_controller
PUBLISH_ODOM_TF = True

# Linkovi cija se pozicija objavljuje relativno na base_link.
# base_footprint NIJE ovdje jer ga je importer spojio u base_link.
CHILD_LINKS = [
    "/World/husky/front_left_wheel",
    "/World/husky/front_right_wheel",
    "/World/husky/rear_left_wheel",
    "/World/husky/rear_right_wheel",
    "/World/husky/base_laser",
    "/World/husky/imu_link",
]

PUBLISH_CLOCK = True    # False ako /clock vec objavljuje graf za cmd_vel

# ================================================================

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
    # takt
    ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
    ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),

    # ROS 2 kontekst
    ("Context.outputs:context", "PublishOdom.inputs:context"),
    ("Context.outputs:context", "PublishTF.inputs:context"),

    # vrijeme
    ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),

    # odometrija -> poruka
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
        # ista poza koja ide u /odom ide i u TF -> zajamceno konzistentni
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
print(f"Graf izgraden: {GRAPH_PATH}")
print(f"  /odom        odom_frame={ODOM_FRAME}  base_frame={BASE_FRAME}")
print(f"  odom->base_link TF: {'DA' if PUBLISH_ODOM_TF else 'NE (radi ga ros2_control)'}")
print(f"  base_link -> {len(CHILD_LINKS)} djece")
print("Pritisni PLAY.")
print("=" * 60)
