cmake_minimum_required(VERSION 3.8)
project(navigation_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateToGoal.srv"
  "srv/NavigateArc.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(navigation_system_node src/navigation_system.cpp)

ament_target_dependencies(navigation_system_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(navigation_system_node "${cpp_typesupport_target}")

install(TARGETS
  navigation_system_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  builtin_interfaces
  geometry_msgs
)

ament_package()
