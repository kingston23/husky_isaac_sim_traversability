"""
Builds an ActionGraph for driving the Husky via /cmd_vel in Isaac Sim.

Chain:
    /cmd_vel (Twist) -> BreakVector3 -> DifferentialController
                     -> ConstructArray(4) -> ArticulationController

Usage:
    Isaac Sim -> Window -> Script Editor -> paste -> Ctrl+Enter
    Then press PLAY. The graph does not execute without PLAY.

Prerequisites:
    Robot is in the stage, has ArticulationRootAPI, and the ROS 2 bridge
    extension (isaacsim.ros2.bridge) is enabled.
"""

import omni.graph.core as og
import usdrt.Sdf

# ============================ CONFIGURE ============================

# Path to the prim carrying ArticulationRootAPI. Verify in the Stage panel!
ROBOT_PRIM = "/World/husky"

GRAPH_PATH = "/World/HuskyControlGraph"

CMD_VEL_TOPIC = "/cmd_vel"

# Order matters: LEFT wheels first, then RIGHT.
JOINT_NAMES = [
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
]

WHEEL_RADIUS = 0.1651        # m, from the URDF

# The geometric track is 0.5708, but Husky is skid-steer: the wheels slip
# when turning, so it behaves as if the track were wider.
# Turning too slowly -> decrease. Too fast -> increase.
WHEEL_DISTANCE = 0.5708 * 1.5

MAX_LINEAR_SPEED = 1.0       # m/s
MAX_ANGULAR_SPEED = 2.0      # rad/s

# ===================================================================

keys = og.Controller.Keys

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick",  "omni.graph.action.OnPlaybackTick"),
            ("Context",         "isaacsim.ros2.bridge.ROS2Context"),
            ("ReadSimTime",     "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock",    "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("SubscribeTwist",  "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("BreakLinear",     "omni.graph.nodes.BreakVector3"),
            ("BreakAngular",    "omni.graph.nodes.BreakVector3"),
            ("DiffController",  "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("LeftVel",         "omni.graph.nodes.ArrayIndex"),
            ("RightVel",        "omni.graph.nodes.ArrayIndex"),
            ("MakeArray",       "omni.graph.nodes.ConstructArray"),
            ("ArtController",   "isaacsim.core.nodes.IsaacArticulationController"),
        ],

        # ConstructArray only has input0 by default; add three more.
        keys.CREATE_ATTRIBUTES: [
            ("MakeArray.inputs:input1", "double"),
            ("MakeArray.inputs:input2", "double"),
            ("MakeArray.inputs:input3", "double"),
        ],

        keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "/clock"),

            ("SubscribeTwist.inputs:topicName", CMD_VEL_TOPIC),

            ("DiffController.inputs:wheelRadius",      WHEEL_RADIUS),
            ("DiffController.inputs:wheelDistance",    WHEEL_DISTANCE),
            ("DiffController.inputs:maxLinearSpeed",   MAX_LINEAR_SPEED),
            ("DiffController.inputs:maxAngularSpeed",  MAX_ANGULAR_SPEED),

            # DifferentialController returns [left, right]
            ("LeftVel.inputs:index",  0),
            ("RightVel.inputs:index", 1),

            ("MakeArray.inputs:arraySize", 4),
            ("MakeArray.inputs:arrayType", "double[]"),

            ("ArtController.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT_PRIM)]),
            ("ArtController.inputs:jointNames", JOINT_NAMES),
        ],

        keys.CONNECT: [
            # --- tick ---
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "DiffController.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArtController.inputs:execIn"),

            # --- ROS 2 context and clock ---
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("Context.outputs:context", "SubscribeTwist.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),

            # --- Twist -> scalars ---
            ("SubscribeTwist.outputs:linearVelocity",  "BreakLinear.inputs:tuple"),
            ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
            ("BreakLinear.outputs:x",  "DiffController.inputs:linearVelocity"),
            ("BreakAngular.outputs:z", "DiffController.inputs:angularVelocity"),

            # --- 2 velocities -> 4 wheels ---
            ("DiffController.outputs:velocityCommand", "LeftVel.inputs:array"),
            ("DiffController.outputs:velocityCommand", "RightVel.inputs:array"),

            ("LeftVel.outputs:value",  "MakeArray.inputs:input0"),   # front_left
            ("LeftVel.outputs:value",  "MakeArray.inputs:input1"),   # rear_left
            ("RightVel.outputs:value", "MakeArray.inputs:input2"),   # front_right
            ("RightVel.outputs:value", "MakeArray.inputs:input3"),   # rear_right

            ("MakeArray.outputs:array", "ArtController.inputs:velocityCommand"),
        ],
    },
)

print(f"Graph built: {GRAPH_PATH}")
print(f"Robot: {ROBOT_PRIM}")
print(f"Joints: {JOINT_NAMES}")
print("Press PLAY, then publish to", CMD_VEL_TOPIC, "from a terminal.")
