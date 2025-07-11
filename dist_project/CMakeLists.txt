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

find_package(PCL REQUIRED COMPONENTS common io filters)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Test Files Compilation

#--------
add_executable (pcl_visualizer_demo src/cloud_viewer2.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})
install(TARGETS
pcl_visualizer_demo
  DESTINATION lib/${PROJECT_NAME})
#-------
add_executable (no_plane src/no_plane.cpp)
target_link_libraries (no_plane ${PCL_LIBRARIES})
install(TARGETS
no_plane
  DESTINATION lib/${PROJECT_NAME})
#------
add_executable (voxels src/voxels.cpp)
target_link_libraries (voxels ${PCL_LIBRARIES})
install(TARGETS
voxels
  DESTINATION lib/${PROJECT_NAME})
#------
add_executable (clustering src/generic_clustering.cpp)
target_link_libraries (clustering ${PCL_LIBRARIES})
install(TARGETS
clustering
  DESTINATION lib/${PROJECT_NAME})

#------

add_executable (pcl_filter_node
   src/nodes/pcl_filter.cpp)

ament_target_dependencies(pcl_filter_node rclcpp tf2_ros tf2 tf2_eigen sensor_msgs visualization_msgs geometry_msgs pcl_ros)

target_link_libraries (pcl_filter_node ${PCL_LIBRARIES})
install(TARGETS
pcl_filter_node
  DESTINATION lib/${PROJECT_NAME})




add_executable(viewer src/cloud_viewer.cpp)
ament_target_dependencies(viewer ${PCL_LIBRAREIS})
target_link_libraries (viewer ${PCL_LIBRARIES})
install(TARGETS
viewer
  DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME})

ament_package()
