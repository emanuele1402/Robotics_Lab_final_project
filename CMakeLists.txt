cmake_minimum_required(VERSION 3.8)
project(final_project)

# Standard C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generazione Interfacce (Solo quelle che hai effettivamente in msg/ e action/)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionError.msg"
  "action/LinearTrajectory.action"
  DEPENDENCIES geometry_msgs
)

# Inclusione Header
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Ottieni il supporto per i tipi generati
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# --- ESEGUIBILE 1: CONVEYOR CONTROLLER ---
add_executable(conveyor_controller_node src/conveyor_controller.cpp)
ament_target_dependencies(conveyor_controller_node
  rclcpp sensor_msgs geometry_msgs ros_gz_interfaces tf2 std_msgs
)

# --- ESEGUIBILE 2: KUKA ACTION SERVER ---
add_executable(kuka_action_server 
  src/kuka_action_server.cpp 
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)

# 1. Link dei target non-Ament (Eigen e il typesupport dei tuoi messaggi)
target_link_libraries(kuka_action_server 
  "${cpp_typesupport_target}" 
  orocos-kdl 
  Eigen3::Eigen
)

# 2. Dipendenze Ament (Rimuovi Eigen3::Eigen da qui)
ament_target_dependencies(kuka_action_server
  rclcpp 
  rclcpp_action 
  sensor_msgs 
  geometry_msgs 
  std_msgs 
  kdl_parser 
  ros_gz_interfaces
)

# --- ESEGUIBILE 3: KUKA ACTION CLIENT ---
add_executable(kuka_action_client src/kuka_action_client.cpp)
target_link_libraries(kuka_action_client "${cpp_typesupport_target}")
ament_target_dependencies(kuka_action_client
  rclcpp 
  rclcpp_action 
  geometry_msgs 
  std_msgs 
  aruco_msgs
  tf2
  tf2_ros
  tf2_kdl
  tf2_geometry_msgs
  orocos_kdl
  Eigen3
)
add_executable(dynamic_tf_publisher src/odom_bl_tf.cpp)
ament_target_dependencies(dynamic_tf_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(nav_seq src/nav_seq.cpp)
ament_target_dependencies(nav_seq
  rclcpp
  rclcpp_action
  nav2_msgs
  ros_gz_interfaces
  geometry_msgs
  tf2
)

# 3. Installazione degli Eseguibili
install(TARGETS 
  conveyor_controller_node
  kuka_action_server
  kuka_action_client
  dynamic_tf_publisher
  nav_seq
  DESTINATION lib/${PROJECT_NAME})

# 4. Installazione delle Risorse
install(DIRECTORY launch worlds models meshes urdf config rviz maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
