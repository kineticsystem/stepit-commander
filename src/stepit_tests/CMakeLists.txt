cmake_minimum_required(VERSION 3.16)
project(stepit_tests)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(behaviortree_cpp REQUIRED)
  find_package(behaviortree_ros2 REQUIRED)
  find_package(stepit_behaviors REQUIRED)
  find_package(stepit_server REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)

  # The behaviors, ticked inside a behavior tree.
  ament_add_gtest(test_offset_vector tests/test_offset_vector.cpp)
  target_link_libraries(test_offset_vector
    stepit_behaviors::stepit_behaviors)

  # Looping over values, e.g. positions.
  ament_add_gtest(test_steps tests/test_steps.cpp)
  target_link_libraries(test_steps
    stepit_behaviors::stepit_behaviors)

  # The trajectory to a set of joint positions.
  ament_add_gtest(test_cubic_trajectory tests/test_cubic_trajectory.cpp)
  target_link_libraries(test_cubic_trajectory
    stepit_behaviors::stepit_behaviors
    ${trajectory_msgs_TARGETS})

  # The fastest trajectory the limits allow.
  ament_add_gtest(test_trapezoidal_trajectory tests/test_trapezoidal_trajectory.cpp)
  target_link_libraries(test_trapezoidal_trajectory
    stepit_behaviors::stepit_behaviors
    rclcpp::rclcpp
    ${trajectory_msgs_TARGETS})

  # Reading the joint positions, against a fake robot.
  ament_add_gtest(test_get_joint_positions tests/test_get_joint_positions.cpp)
  target_include_directories(test_get_joint_positions PRIVATE tests)
  target_link_libraries(test_get_joint_positions
    stepit_behaviors::stepit_behaviors
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${control_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${trajectory_msgs_TARGETS})

  # Reading a port that holds either a list of names or a single name.
  ament_add_gtest(test_ports tests/test_ports.cpp)
  target_link_libraries(test_ports stepit_behaviors::stepit_behaviors)

  # The strictness of a controller switch.
  ament_add_gtest(test_switch_controller tests/test_switch_controller.cpp)
  target_link_libraries(test_switch_controller stepit_behaviors::stepit_behaviors)

  # The committed node models match the behaviors.
  ament_add_gtest(test_nodes_model tests/test_nodes_model.cpp)
  target_link_libraries(test_nodes_model stepit_behaviors::stepit_behaviors)
  target_compile_definitions(test_nodes_model PRIVATE
    NODES_MODEL_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../stepit_objectives/objectives/stepit_behaviors.xml")

  # The payload of a command.
  ament_add_gtest(test_payload tests/test_payload.cpp)
  target_link_libraries(test_payload stepit_server::stepit_server)

  # The objectives, run end to end against a fake robot.
  ament_add_gtest(test_offset_joints_by_objective tests/test_offset_joints_by_objective.cpp)
  target_include_directories(test_offset_joints_by_objective PRIVATE tests)
  target_link_libraries(test_offset_joints_by_objective
    ament_index_cpp::ament_index_cpp
    stepit_behaviors::stepit_behaviors
    stepit_server::stepit_server
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${control_msgs_TARGETS}
    ${controller_manager_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${trajectory_msgs_TARGETS})

  ament_add_gtest(test_move_joints_to_objective
    tests/test_move_joints_to_objective.cpp)
  target_include_directories(test_move_joints_to_objective PRIVATE tests)
  target_link_libraries(test_move_joints_to_objective
    ament_index_cpp::ament_index_cpp
    stepit_behaviors::stepit_behaviors
    stepit_server::stepit_server
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${control_msgs_TARGETS}
    ${controller_manager_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${trajectory_msgs_TARGETS})

  ament_add_gtest(test_activate_controller_objective
    tests/test_activate_controller_objective.cpp)
  target_include_directories(test_activate_controller_objective PRIVATE tests)
  target_link_libraries(test_activate_controller_objective
    ament_index_cpp::ament_index_cpp
    stepit_behaviors::stepit_behaviors
    stepit_server::stepit_server
    rclcpp::rclcpp
    ${controller_manager_msgs_TARGETS})
endif()

ament_package()
