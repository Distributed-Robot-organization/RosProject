aliases and bash functions to make the containter run as intended

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
      nav2_tests_turtlebot
}


alias ros_docker_connect="docker exec -it ros_docker bash"
```
