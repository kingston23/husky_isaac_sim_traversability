## Provjera simulacijskih parametara
1. izmjeri stvarni RTF - treba biti što bliže 1
```
import omni.timeline, time, omni.kit.app
tl = omni.timeline.get_timeline_interface()
s0, w0 = tl.get_current_time(), time.time()
def cb(e):
    dw = time.time() - w0
    if dw > 5:
        print("RTF:", (tl.get_current_time() - s0) / dw); sub.unsubscribe()
sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(cb)
```

2. prebaciti Physics Scene da radi s CPU a ne GPU ako se radi s 1 robotom.
U Stage -> /physicsScene -> Property -> PhysicsScene (PhysX dio postavki scene)
prebaciti s Enable GPU Dynamics i Broadphase Type = GPU na isključi Enable GPU Dynamics i Broadphase Type = MBP

3. po potrebi smanjiti broj iteracija za Husky:
32 pozicijske iteracije je maksimum. Za robota na kotačima je 4–8 obično dovoljno, pa ovo spusti na 8

```
import omni.usd
from pxr import PhysxSchema
stage = omni.usd.get_context().get_stage()
art = PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath("/World/husky/base_link"))
art.GetSolverPositionIterationCountAttr().Set(8)
```

Poslije provjeri ponaša li se Husky normalno na stepenicama, odnosno da ne propada i ne trese se. Ako ima problema, probaj 12–16.

4. Fizika piše rezultate u USD umjesto u Fabric
Ako je Fabric isključen, PhysX svaki korak zapisuje poze i brzine svih tijela natrag u USD. To je spor, jednodretveni posao na glavnoj dretvi, a nestaje kad pauziraš. Provjeri:

```
import carb, omni.kit.app
s = carb.settings.get_settings()
for k in ["/physics/updateToUsd", "/physics/updateVelocitiesToUsd"]:
    print(k, s.get(k))
em = omni.kit.app.get_app().get_extension_manager()
print("omni.physx.fabric:", em.is_extension_enabled("omni.physx.fabric"))
```

Ako je omni.physx.fabric False, uključi ga u Window → Extensions (traži “physx fabric”), pa Stop i Play.


## Isaac sim

test linearna - naprijed:
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist   "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

test angularna brzina - rotacija
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist   "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.80}}" -r 10
```

# test husky description package:
1. run simulation in isaac sim - open husky_test.usd
2. in terminal 1 run: ros2 launch husky_description description.launch.py 
3. in terminal 2 run: rviz2 and set Fixed frame as base_link and description topic as robot_description 

# test navigation 
U ROS2 RTX Lidar graph uključi opciju za puni sken Publish Full Scan u LaserScanPublish i PointCloudPublish. Tada node čeka cijelu rotaciju prije objave — dobiješ puni prsten, ali s većom latencijom.

dodan staticki transform za lidar u nav2_test.launch.py:

```
    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "top_plate_front_link", "OS0"],
    )
```

Pokretanje:
ros2 run octomap_server octomap_server_node --ros-args   -p resolution:=0.1   -p frame_id:=map   -p base_frame_id:=base_link   -p sensor_model.max_range:=20.0   -p use_sim_time:=true   -r cloud_in:=/lidar/point_cloud -p use_sim_time:=true -p occupancy_min_z:=0.15

ili ovo:
ros2 run octomap_server octomap_server_node --ros-args \
  -p resolution:=0.15 \
  -p frame_id:=map \
  -p base_frame_id:=base_link \
  -p sensor_model.max_range:=15.0 \
  -p use_sim_time:=true \
  -p filter_ground:=true \
  -p ground_filter.distance:=0.08 \
  -p ground_filter.angle:=0.2 \
  -p ground_filter.plane_distance:=0.10 \
  -p pointcloud_min_z:=-0.3 \
  -p pointcloud_max_z:=3.0 \
  -p occupancy_min_z:=0.15 \
  -p occupancy_max_z:=2.0 \
  -p filter_speckles:=true \
  -p compress_map:=true \
  -r cloud_in:=/lidar/point_cloud
  
ros2 launch husky_navigation nav2_test.launch.py 

ros2 launch husky_viz view_navigation.launch.py 


## Third-party code

| Package | Source | License | Changes |
|---|---|---|---|


All third-party packages retain their original licenses. See the `LICENSE` file in each package directory.

## Third-party code

This repository includes the following third-party packages:

| Package | Source | License | Changes |
|---|---|---|---|
| `ground_segmentation` | [dfki-ric/ground_segmentation](https://github.com/dfki-ric/ground_segmentation), branch `master`, commit `<hash>` | BSD-3-Clause, © DFKI Robotics Innovation Center | Integrated into `husky_traversability` |
| `husky_description` | [husky/husky](https://github.com/husky/husky), branch `<branch>`, commit `<hash>` | BSD-3-Clause, © Clearpath Robotics | Merged `inertial_link` into `base_link` for Isaac Sim compatibility |

Original licenses are kept in:
- `husky_traversability/LICENSE.txt` (DFKI ground segmentation)
- `husky_description/LICENSE` (Clearpath Robotics)

Ground segmentation is based on GSeg3D. If you use this work, please cite:

    @inproceedings{lodhi2025gseg3d,
      author    = {Muhammad Haider Khan Lodhi and Christoph Hertzberg},
      title     = {GSeg3D: A High-Precision Grid-Based Algorithm for Safety-Critical Ground Segmentation in LiDAR Point Clouds},
      booktitle = {2025 7th International Conference on Robotics and Computer Vision (ICRCV)},
      pages     = {119-126},
      year      = {2025},
      doi       = {10.1109/ICRCV67407.2025.11349133}
    }
