"""
Creates an IMU sensor on the Husky and builds an ActionGraph publishing it to /imu.

Why OnPlaybackTick may not connect to Read IMU:
    In some versions IsaacReadIMU is a compute node with no inputs:execIn.
    The tick then goes to ROS2PublishImu, which pulls data from ReadIMU.
    This script checks and connects execIn if it exists.

Usage: Window -> Script Editor -> paste -> Ctrl+Enter -> PLAY
"""

import omni.graph.core as og
import omni.kit.commands
import omni.usd
import usdrt.Sdf
from pxr import Gf, UsdPhysics, Usd

# ============================ CONFIGURE ============================

# The parent MUST be a rigid body (base_link), not an empty frame.
# An IMU with no rigid body above it has nothing to measure and outputs zeros.
IMU_PARENT = "/World/husky/base_link"
IMU_NAME = "imu_sensor"

GRAPH_PATH = "/World/HuskyImuGraph"

IMU_TOPIC = "/imu"
IMU_FRAME = "imu_link"

# Sensor offset relative to base_link (from the URDF: CPR_IMU_XYZ)
IMU_TRANSLATION = Gf.Vec3d(0.0, 0.0, 0.0)

# ===================================================================

IMU_PATH = f"{IMU_PARENT}/{IMU_NAME}"
stage = omni.usd.get_context().get_stage()

# ---------- 0. Verify the parent is a rigid body ----------
parent_prim = stage.GetPrimAtPath(IMU_PARENT)
if not parent_prim.IsValid():
    raise RuntimeError(f"Parent does not exist: {IMU_PARENT}")
if not parent_prim.HasAPI(UsdPhysics.RigidBodyAPI):
    print(f"[0] WARNING: {IMU_PARENT} has no RigidBodyAPI.")
    print("    The IMU will output zeros. Pick a rigid body as the parent.")
else:
    print(f"[0] Parent is a rigid body: {IMU_PARENT}")

# ---------- 1. Create the IMU sensor ----------

if stage.GetPrimAtPath(IMU_PATH).IsValid():
    print(f"[1] IMU already exists: {IMU_PATH}")
else:
    ok, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path="/" + IMU_NAME,
        parent=IMU_PARENT,
        sensor_period=-1.0,                  # -1 = every physics step
        translation=IMU_TRANSLATION,
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        linear_acceleration_filter_size=10,
        angular_velocity_filter_size=10,
        orientation_filter_size=10,
    )
    print(f"[1] IMU created: {IMU_PATH}  (ok={ok})")

# ---------- 2. Resolve node type names for this version ----------

def pick_node_type(*candidates):
    """Return the first node type that exists in this Isaac version."""
    registry = og.GraphRegistry()
    for c in candidates:
        try:
            if registry.get_node_type(c) is not None:
                return c
        except Exception:
            pass
    return candidates[0]   # let it fail with a clear name

READ_IMU = pick_node_type(
    "isaacsim.sensors.physics.IsaacReadIMU",
    "omni.isaac.sensor.IsaacReadIMU",
)
PUB_IMU = pick_node_type(
    "isaacsim.ros2.bridge.ROS2PublishImu",
    "omni.isaac.ros2_bridge.ROS2PublishImu",
)
READ_TIME = pick_node_type(
    "isaacsim.core.nodes.IsaacReadSimulationTime",
    "omni.isaac.core_nodes.IsaacReadSimulationTime",
)
CONTEXT = pick_node_type(
    "isaacsim.ros2.bridge.ROS2Context",
    "omni.isaac.ros2_bridge.ROS2Context",
)

print(f"[2] Node types:\n    {READ_IMU}\n    {PUB_IMU}")

# ---------- 3. Build the graph ----------

keys = og.Controller.Keys

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context",        CONTEXT),
            ("ReadSimTime",    READ_TIME),
            ("ReadIMU",        READ_IMU),
            ("PublishIMU",     PUB_IMU),
        ],
        keys.SET_VALUES: [
            ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(IMU_PATH)]),
            ("ReadIMU.inputs:readGravity", True),

            ("PublishIMU.inputs:topicName", IMU_TOPIC),
            ("PublishIMU.inputs:frameId",   IMU_FRAME),
            ("PublishIMU.inputs:publishLinearAcceleration", True),
            ("PublishIMU.inputs:publishAngularVelocity",    True),
            ("PublishIMU.inputs:publishOrientation",        True),
        ],
        keys.CONNECT: [
            # The tick goes to PUBLISH, not to READ
            ("OnPlaybackTick.outputs:tick", "PublishIMU.inputs:execIn"),
            ("Context.outputs:context",     "PublishIMU.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),

            # data
            ("ReadIMU.outputs:linAcc",      "PublishIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:angVel",      "PublishIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
        ],
    },
)

print(f"[3] Graph built: {GRAPH_PATH}")

# ---------- 4. Introspection: what ReadIMU actually exposes ----------

read_node = og.Controller.node(f"{GRAPH_PATH}/ReadIMU")
attrs = [a.get_name() for a in read_node.get_attributes()]
print("\n[4] ReadIMU node attributes in your version:")
for a in sorted(attrs):
    print(f"    {a}")

# If execIn exists, wire it up as well
if "inputs:execIn" in attrs:
    try:
        og.Controller.connect(
            og.Controller.attribute(f"{GRAPH_PATH}/OnPlaybackTick.outputs:tick"),
            og.Controller.attribute(f"{GRAPH_PATH}/ReadIMU.inputs:execIn"),
        )
        print("\n    execIn exists -- connected to OnPlaybackTick.")
    except Exception as e:
        print(f"\n    execIn exists but the connection failed: {e}")
else:
    print("\n    No inputs:execIn -- confirms ReadIMU is a pure compute node.")
    print("    That is normal; it does not need the tick.")

print("\nDONE. Press PLAY, then check:  ros2 topic echo /imu")
print("A stationary robot should report linear_acceleration.z of about 9.81")
