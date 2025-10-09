This is designed to be run as part of the Ros Shelfino project

# Launch simulation
```bash
 ros2 launch dist_project spawn_shelfino.launch.py

 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=shelfino1/cmd_vel

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

ros2 launch dist_project start_slam.launch.py use_sim_time:=true

ros2 run nav2_map_server map_saver_cli -f my_map

# Controllo dei transform frame
ros2 run tf2_tools view_frames

ros2 run rviz2 rviz2 -d /home/computer/src/Distributed_project/dist_project/config/overall_map.rviz
```

per controllo con controller:
``` bash
ros2 run teleop_twist_joy teleop_node --ros-args --remap cmd_vel:=shelfino1/cmd_vel use_sim_time:=true

ros2 run joy joy_node
```

```

# Docker Commands
aliases and bash functions to make the container run as intended

## docker command build
``` bash
docker build . -t dist_docker
```
## Commands to run Docker

``` bash
ros_docker(){
    xhost +local:root
    sudo docker run -it --rm \
      --env="DISPLAY=$DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --volume=".:/home/computer"\
      --network=host \
      --privileged \
      --name ros_docker \
      --workdir=/home/computer \
      dist_docker
}


alias ros_docker_connect="docker exec -it ros_docker bash"
```

Per visualizzare https://github.com/introlab/rtabmap_ros/tree/humble-devel

Using cycloneDDS since the default one has problems in ROS2 humble.
[Followed tutorial](https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/)
## Map
The map is defined at priori using the slamtoolbox to use the nav2 stack.
