aliases and bash functions to make the container run as intended

docker command build
``` bash
docker build dist_docker .
```

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
