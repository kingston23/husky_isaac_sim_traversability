# husky_localization

Dual-EKF localisation for a Husky A200 simulated in Isaac Sim.
Build type: `ament_cmake`.

## Layout

```
husky_localization/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── dual_ekf_navsat.yaml          installed to share/<pkg>/config
├── launch/
│   └── dual_ekf_navsat.launch.py     installed to share/<pkg>/launch
├── husky_localization/               importable Python module
│   ├── __init__.py
│   ├── gps_sim_node.py
│   └── odom_drift_node.py
└── scripts/                          thin wrappers -> lib/<pkg>
    ├── gps_sim_node
    └── odom_drift_node
```

The split between `husky_localization/` and `scripts/` is deliberate.
`ament_python_install_package()` handles the module; `install(PROGRAMS ...)`
handles the executables. `ros2 run` only looks in `lib/<pkg>`, so the
wrappers must land there while the real code stays importable.

## Data flow

```
Isaac /odom (ground truth)
   |
   +--> odom_drift_node --> /odom/noisy ------> ekf_odom  (odom -> base_link)
   |                                     \
   |                                      +---> ekf_map   (map  -> odom)
   |                                           ^
   +--> gps_sim_node ----> /gps/fix ----> navsat_transform --> /odometry/gps
```

The GPS branches off the ground truth **independently** of the drift node.
If it did not, the GPS would carry the same error it is supposed to correct.

## TF ownership

Exactly one publisher per transform, or the TF tree breaks:

| Transform | Publisher |
|---|---|
| `map -> odom` | `ekf_filter_node_map` |
| `odom -> base_link` | `ekf_filter_node_odom` |
| `base_link -> sensors` | Isaac OmniGraph |

So in `husky_odom_tf_graph.py` set `PUBLISH_ODOM_TF = False`.

## Build

```bash
sudo apt install ros-jazzy-robot-localization
cd ~/husky_ws_isaacSim
colcon build --symlink-install --packages-select husky_localization
source install/setup.bash
ros2 pkg executables husky_localization    # must list both nodes
```

## Run

Isaac Sim first (PLAY pressed, `/odom` and `/imu` publishing), then:

```bash
ros2 launch husky_localization dual_ekf_navsat.launch.py
```

## Verify

```bash
ros2 topic hz /odom/noisy
ros2 topic echo /gps/fix --once
ros2 topic echo /odometry/gps --once
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

Compare the three position estimates. The filtered one should track ground
truth more closely than the drifted one does:

```bash
ros2 topic echo /odom --field pose.pose.position                     # truth
ros2 topic echo /odom/noisy --field pose.pose.position               # drift
ros2 topic echo /odometry/filtered/global --field pose.pose.position # EKF
```

`odom_drift_node` also logs its own error every 10 seconds.

## Tuning

Drift too small to be interesting? Push `angular_scale` further from 1.0 —
rotation is where skid-steer odometry actually falls apart.

`/odometry/gps` never appears? Almost always the IMU: `navsat_transform_node`
needs an absolute heading. Check that `/imu` has a populated `orientation`
and that `orientation_covariance[0]` is not `-1`.

GPS position consistently rotated by 90 degrees? Set `yaw_offset: 1.5708`.
`navsat_transform_node` assumes ENU, where yaw 0 points East.
