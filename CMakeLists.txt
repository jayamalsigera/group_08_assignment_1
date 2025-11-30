cmake_minimum_required(VERSION 3.8)
project(group_8_assignment_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
# ----------------------------------------
find_package(ament_cmake REQUIRED)

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
find_package(OpenCV REQUIRED)

find_package(nav2_msgs REQUIRED)

# Executables
# ----------------------------------------

# initial pose sender
add_executable(initial_pose_node
  src/initial_pose_node.cpp
)

ament_target_dependencies(initial_pose_node
  rclcpp
  geometry_msgs
)

# camera relay
add_executable(camera_relay_node
  src/camera_relay_node.cpp
)

ament_target_dependencies(camera_relay_node
  rclcpp
  sensor_msgs
)

# apriltag > center pose
add_executable(apriltag_center_node
  src/apriltag_center_node.cpp
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

target_link_libraries(apriltag_center_node
  ${OpenCV_LIBS}
)

# navigation > publish goal_pose
add_executable(navigation_goal_node
  src/navigation_goal_node.cpp
)

ament_target_dependencies(navigation_goal_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(table_detection_node
  src/table_detection_node.cpp
)

ament_target_dependencies(table_detection_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install
# ----------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    initial_pose_node
    camera_relay_node
    apriltag_center_node
    navigation_goal_node
    table_detection_node
  DESTINATION lib/${PROJECT_NAME}
)
# ----------------------------------------
ament_package()
