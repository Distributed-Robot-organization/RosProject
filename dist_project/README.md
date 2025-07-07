Usare lo spawn_shelfino

``` bash
ros2 launch dist_project spawn_shelfino.launch.py use_sim_time:=True

ros2 launch dist_project start_slam.launch.py use_sim_time:=true

ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

ros2 run nav2_map_server map_saver_cli -f my_map

# Controllo dei transform frame
ros2 run tf2_tools view_frames
```

per controllo con controller:
``` bash
ros2 run teleop_twist_joy teleop_node --ros-args --remap cmd_vel:=shelfino1/cmd_vel

ros2 run joy joy_node
```
