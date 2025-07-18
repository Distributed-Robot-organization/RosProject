cmake_minimum_required(VERSION 3.8)
project(dist_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav2_msgs REQUIRED)
#find_package(pcl_transforms REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs)
find_package(pcl_ros REQUIRED)
find_package(interfaces_pkg REQUIRED)


find_package(PCL REQUIRED COMPONENTS common io filters)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#<<<< Cluster Filter Node ####

add_executable (pcl_filter_node
   src/nodes/pcl_filter.cpp)

ament_target_dependencies(pcl_filter_node rclcpp tf2_ros tf2 tf2_eigen sensor_msgs visualization_msgs geometry_msgs pcl_ros)

target_link_libraries (pcl_filter_node ${PCL_LIBRARIES})
install(TARGETS
pcl_filter_node
  DESTINATION lib/${PROJECT_NAME})
#### Cluster Filter Node [END] >>>#

#<<<< Cluster Finder NodeS ####
add_executable (first_cluster_publisher
   src/nodes/first_cluster_publisher.cpp)

ament_target_dependencies(first_cluster_publisher rclcpp tf2_ros tf2 tf2_eigen sensor_msgs visualization_msgs geometry_msgs interfaces_pkg)

target_link_libraries (first_cluster_publisher)
install(TARGETS
first_cluster_publisher
  DESTINATION lib/${PROJECT_NAME})

#### Cluster Finder NodeS [END] >>>#


install(DIRECTORY
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME})

ament_package()
