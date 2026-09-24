cmake_minimum_required(VERSION 3.16)
project(commander_behaviors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

######################################################
# The behaviors, as a normal library. Anything owning a BT factory (the tests,
# for instance) can link against it and call commander_behaviors::registerNodes.

add_library(commander_behaviors SHARED
  src/follow_joint_trajectory.cpp
  src/get_active_controllers.cpp
  src/get_joint_positions.cpp
  src/nodes_model.cpp
  src/offset_joint_positions.cpp
  src/ports.cpp
  src/register_nodes.cpp
  src/switch_controller.cpp)

target_include_directories(commander_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(commander_behaviors PUBLIC
  behaviortree_ros2::behaviortree_ros2
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${control_msgs_TARGETS}
  ${controller_manager_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS})

######################################################
# The same behaviors, wrapped into a BehaviorTree.CPP plugin, which is how the
# commander server loads them at runtime.

add_library(commander_behaviors_plugin SHARED src/plugin.cpp)
target_compile_definitions(commander_behaviors_plugin PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(commander_behaviors_plugin PRIVATE commander_behaviors)

######################################################
# Writes the node models of the behaviors, for editors like Groot2:
#   ros2 run commander_behaviors write_nodes_model <file>

add_executable(write_nodes_model src/write_nodes_model.cpp)
target_link_libraries(write_nodes_model PRIVATE commander_behaviors)

######################################################
# INSTALL

install(DIRECTORY include/ DESTINATION include)

install(TARGETS commander_behaviors
  EXPORT export_commander_behaviors
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS write_nodes_model DESTINATION lib/${PROJECT_NAME})

# Plugins are looked up by the server as "<package>/<subfolder>", relative to
# the share directory of the package.
install(TARGETS commander_behaviors_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins)

ament_export_targets(export_commander_behaviors HAS_LIBRARY_TARGET)
ament_export_dependencies(
  behaviortree_cpp
  behaviortree_ros2
  control_msgs
  controller_manager_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs)

ament_package()
