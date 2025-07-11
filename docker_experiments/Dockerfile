# Extend the ROS2 distro
FROM osrf/ros:humble-desktop-full

#RUN useradd -ms /bin/bash ros && echo "ros:ros" | chpasswd && adduser ros sudo
#RUN adduser ros video

# Install the required packages
RUN apt-get update \
    && sudo apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs\
    python3-colcon-common-extensions\
    python3-shapely\
    ros-humble-rmw-cyclonedds-cpp\
    ros-humble-rtabmap-*\
    ros-humble-pcl-*\
    libcpl-dev\
    && rm -rf /var/lib/apt/lists/*

#ros-humble-turtlebot3* \


#RUN  sudo apt-get install ros2-humble-moveit* \
#    ros-humble-ign-ros2-control



# Set the environment variable required to run TurtleBot3
#ENV TURTLEBOT3_MODEL=burger
#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo 'alias refresh="source install/setup.bash"' >> ~/.bashrc
RUN echo 'alias build="colcon build --symlink-install && source install/setup.bash"' >> ~/.bashrc
RUN echo 'clean_build(){\
    rm -rf build install log \
    build\
    }' >> /root/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
