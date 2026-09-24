cmake_minimum_required(VERSION 3.16)
project(stepit_server)

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
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

######################################################
# The server, as a library, so that it can be tested. It has to be static,
# because behaviortree_ros2 is itself a static library built without -fPIC.

add_library(stepit_server STATIC
  src/stepit_server.cpp
  src/payload.cpp)

set_target_properties(stepit_server PROPERTIES POSITION_INDEPENDENT_CODE OFF)

target_include_directories(stepit_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(stepit_server PUBLIC
  behaviortree_ros2::behaviortree_ros2
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  ${btcpp_ros2_interfaces_TARGETS}
  PRIVATE
  ${YAML_CPP_TARGET})

######################################################
# The node.

add_executable(stepit_server_node src/main.cpp)
set_target_properties(stepit_server_node PROPERTIES OUTPUT_NAME stepit_server)
target_link_libraries(stepit_server_node PRIVATE stepit_server)

######################################################
# INSTALL

install(DIRECTORY include/ DESTINATION include)

install(TARGETS stepit_server
  EXPORT export_stepit_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS stepit_server_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

ament_export_targets(export_stepit_server HAS_LIBRARY_TARGET)
ament_export_dependencies(
  behaviortree_cpp
  behaviortree_ros2
  btcpp_ros2_interfaces
  rclcpp
  yaml-cpp)

ament_package()
