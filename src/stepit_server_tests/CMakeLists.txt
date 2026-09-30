cmake_minimum_required(VERSION 3.16)
project(stepit_server_tests)

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
  find_package(behaviortree_cpp REQUIRED)
  find_package(stepit_server REQUIRED)

  # Reading the objectives again when they change.
  ament_add_gtest(test_tree_loader tests/test_tree_loader.cpp)
  target_link_libraries(test_tree_loader stepit_server::stepit_server)

  # The status of the running tree, as feedback for a client.
  ament_add_gtest(test_execution_status tests/test_execution_status.cpp)
  target_link_libraries(test_execution_status stepit_server::stepit_server)

  # The payload of a command.
  ament_add_gtest(test_payload tests/test_payload.cpp)
  target_link_libraries(test_payload stepit_server::stepit_server)
endif()

ament_package()
