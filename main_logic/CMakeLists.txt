cmake_minimum_required(VERSION 3.8)
project(main_logic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_system REQUIRED)
find_package(coordination REQUIRED)
find_package(navigation_system REQUIRED)
include_directories(include)
# add here the include client ======================================

add_library(coordination_client_lib
  src/client_service/coordination_client.cpp
)
add_library(vision_client_lib
  src/client_service/vision_client.cpp
)
add_library(navigation_client_lib
  src/client_service/navigation_client.cpp
)

ament_target_dependencies(vision_client_lib
  rclcpp
  std_srvs
  std_msgs
  vision_system
)

ament_target_dependencies(coordination_client_lib
  rclcpp
  std_srvs
  std_msgs
  coordination
)

ament_target_dependencies(navigation_client_lib
  rclcpp
  std_srvs
  std_msgs
  geometry_msgs
  navigation_system
)

install(TARGETS coordination_client_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS vision_client_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS navigation_client_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# ==============================================
# example ======================================
add_executable(vision_client_example
  src/example/vision_client_example.cpp
)
add_executable(coordination_client_example
  src/example/coordination_client_example.cpp
)
add_executable(navigation_client_example
  src/example/navigation_client_example.cpp
)

target_link_libraries(vision_client_example
  vision_client_lib
)
target_link_libraries(coordination_client_example
  coordination_client_lib
)
target_link_libraries(navigation_client_example
  navigation_client_lib
)

ament_target_dependencies(vision_client_example
  rclcpp
)
ament_target_dependencies(coordination_client_example
  rclcpp
)
ament_target_dependencies(navigation_client_example
  rclcpp
  geometry_msgs
)

install(TARGETS 
  vision_client_example 
  coordination_client_example
  navigation_client_example

  DESTINATION lib/${PROJECT_NAME}
)
# ======================================

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(
    vision_client_lib 
    coordination_client_lib 
    navigation_client_lib)

    
ament_export_dependencies(
  rclcpp
  std_srvs
  std_msgs
  geometry_msgs
  vision_system
  coordination
  navigation_system
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
