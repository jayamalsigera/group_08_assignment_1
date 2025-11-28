cmake_minimum_required(VERSION 3.8)
project(group_8_assignment_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(apriltag_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)

find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(nav2_msgs REQUIRED)

add_executable(initial_pose_node 
src/initial_pose_node.cpp
)

add_executable(apriltag_center_node 
  src/apriltag_center_node.cpp
)

ament_target_dependencies(initial_pose_node 
  rclcpp 
  geometry_msgs
)

add_executable(camera_relay_node 
  src/camera_relay_node.cpp
)

ament_target_dependencies(apriltag_center_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
  apriltag_ros
  apriltag_msgs
  nav2_msgs
)

ament_target_dependencies(camera_relay_node
    rclcpp sensor_msgs
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS apriltag_center_node 
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS initial_pose_node 
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS camera_relay_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
