# Course Project: Distributed Robot Perception - AY 2024/2025

# Project Overview:

The project develops a distributed system for 3D recostruction using multiple robots equipped with RGB-D cameras. Each robot captures depth and data from its environment, processes this data locally to create partial 3D maps, and then shares these maps with other robots in the network. the system allows fotr the accurate acquisition of point clouds without using external markers, thanks to object recognition using computer vision techniques and cooperation between robots. The approach combines distributed robotics logic and artificial intelligence to achieve more efficient, scalable, and autonomous digitization, with potential applications in advanced manufacturing, industrial inspection, cultural heritage conservation, and digital twin generation for simulation and optimization.


# Index:
Chapters:
- [Course Project: Distributed Robot Perception - AY 2024/2025](#course-project-distributed-robot-perception---ay-20242025)
- [Project Overview:](#project-overview)
- [Index:](#index)
- [Tenchologies Used:](#tenchologies-used)
- [Simulation Scene:](#simulation-scene)
- [Components:](#components)
  - [Navigation System](#navigation-system)
  - [Vision System](#vision-system)
  - [Coordination System](#coordination-system)
  - [Main logic System](#main-logic-system)
- [Installation Instructions (USING DOCKER):](#installation-instructions-using-docker)
    - [Clone the Repository:](#clone-the-repository)
    - [Build the Docker Image:](#build-the-docker-image)
    - [Access the Docker Container:](#access-the-docker-container)
    - [Build ROS2 Workspace](#build-ros2-workspace)
- [Running the System](#running-the-system)
- [Project Structure](#project-structure)
- [Contributors](#contributors)

# Tenchologies Used:

- **Robot**: Shelfino
- **Programming language**: C++ (core ROS2 Nodes and navigation) Python (image stream processing, and mesh point cloud)
- **Robotic Framework**: ROS2 Humble Hawksbill
- **Simulation Environment**: Gazebo
- **Library**: 
  - [OpenCV](https://opencv.org/)
  - [Open3D](http://www.open3d.org/)
  - [Pytorch](https://pytorch.org/)
  - [Numpy](https://numpy.org/)
  - [OpenCV-Python](https://pypi.org/project/opencv-python/)
  - [Pytorch](https://pytorch.org/)
  - [NAV2](https://docs.nav2.org/)
  - [Ultralytics YOLO](https://ultralytics.com/)  
- **Control Algorithms**: Standard ROS2 navigation stack (NAV2) with SLAM and path planning capabilities.
- **3D Reconstruction Techniques**: Point cloud generation and merging using depth data from RGB-D cameras, object recognition using YOLO.
- **Communication Protocols**: ROS2 topics and services for inter-robot communication and data sharing. using ROS2 DDS-based communication.

# Simulation Scene:
The simulation environment is set up in Gazebo, featuring a warehouse-like setting with multiple objects to be scanned by the robots. Each Shelfino robot is equipped with an RGB-D camera for depth sensing and object recognition.

# Components:

## Navigation System

The System integrates 3 serviec for 3 differents type of navigation:
1. Random navigation:  The robot moves randomly around its surroundings.
2. Specific Point navigation: The robot navigates to a specific point in the environment.
3. Arc navigation: The robot moves in an arc trajectory around a specific point of interest, with a specific radius.

## Vision System

Leveraging an RGB-D camera, the system performs both object detection and 3D localization. The RGB stream is processed by the YOLO algorithm to identify and classify objects in the scene, while the depth data is used to estimate the 3D position of the object's and create point clouds for 3D reconstruction.


## Coordination System

After the robots have scanned the environment and created their local 3D maps, they share this data with a centralized node. This node is responsible for merging the individual point clouds into a comprehensive 3D model of the environment. The merging process involves aligning the point clouds based on common features and optimizing the overall structure to ensure accuracy and coherence.

## Main logic System

This is the core system that integrates all the other components. It manages the workflow of the robots, coordinating navigation, vision processing, and data sharing. The main logic system ensures that each robot follows the designated scanning strategy, processes the captured data, and contributes to the collective 3D reconstruction effort.

# Installation Instructions (USING DOCKER):

> Note: if you want to install everything manually without using docker, please see the Dockerfile to see the dependency and install mannualy, in that case is request Ubuntu 22 to use humble system.

> This project is tested on Ubuntu 22.04 with ROS2 Humble Hawksbill and Fedora Using Docker System.

> Note: Docker use 20 GB for this project

To simplify the setup process and ensure all ROS 2 and project dependencies are correctly installed, we recommend using Docker. You can also follow this step-by-step [tutorial video](link_tutorial).

### Clone the Repository: 

Clone our repository inside your workspace (also before create src file):

```bash
git clone https://github.com/Distributed-Robot-organization/RosProject.git src
```

### Build the Docker Image:
From the workspace root:

```bash
docker compose -f 'src/docker-compose.yml' up -d --build 'ros2'
```

### Access the Docker Container:
To access the Docker container, use the following command:

```bash
 docker exec -it ros_docker bash
```
if you system restart you can use:
```bash
 docker start -ai ros_docker
```

### Build ROS2 Workspace
Once inside the Docker container, navigate to the workspace directory and build the ROS2 workspace:
```bash
colcon build --symlink-install --parallel-workers 1 && source install/setup.bash 
```

# Running the System

> Note: When using Docker, all terminal commands should be executed inside the running container.

go in the workspace directory:

```bash
cd ~/ros2_ws && source install/setup.bash
```
To launch the entire system with multiple robots, use the following command:

```bash
ros2 launch dist_project spawn_shelfino.launch.py 
```

When also gazebo is running, open new terminal and launch the bring_up launcher:

```bash
ros2 launch main_logic bring_up.launch.py 
```

Now to start the main logic system, open a new terminal and run:
```bash
ros2 topic pub /main_logic/obj_to_detect std_msgs/msg/String "data: 'small_cone'" --once
```
>Note: You can change 'small_cone' with the object you want to scan. (see the **src/main_logic/config/object_params.yaml** to see all the available objects)

# Project Structure
```
.
└── workspace/
    ├── dist_project          --> Pkg where is saved the environemts
    ├── coorindation          --> Pkg for point cloud merging and coordination
    ├── Navigation_system     --> Pkg for robot navigation
    ├── vision_system         --> Pkg for object detection and 3D reconstruction
    ├── main_logic            --> Pkg for main logic system
    ├── ...                   --> Other Pkg used to run the shelfino robot
    └── working_directory     --> folder to save point clouds and meshes
```


# Contributors

This project was developed by:

    Ruben Malacarne - ruben.malacarne@studenti.unitn.it
    Emmanule Coppola - emmanule.coppola@studenti.unitn.it

Enjoy the project! :D