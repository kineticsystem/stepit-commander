// Copyright 2026 Giovanni Remigi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <stepit_server/payload.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stepit_tests
{

/// @brief Path of a tree shipped by stepit_objectives, e.g.
/// treePath("objectives", "offset_joints_by.xml").
inline std::filesystem::path treePath(const std::string& folder, const std::string& file)
{
  return std::filesystem::path{ ament_index_cpp::get_package_share_directory("stepit_objectives") } / folder / file;
}

/**
 * @brief Run an objective the way the commander server does.
 *
 * The payload is parsed and written into the global blackboard, the tree is
 * created below it, and ticked until it is done.
 *
 * @return The status the tree finished with, or RUNNING if it timed out.
 */
inline BT::NodeStatus runObjective(BT::BehaviorTreeFactory& factory, const std::string& objective,
                                   const std::string& payload,
                                   std::chrono::seconds timeout = std::chrono::seconds{ 20 })
{
  auto global_blackboard = BT::Blackboard::create();
  stepit_server::writeToBlackboard(stepit_server::parsePayload(payload), *global_blackboard);

  auto tree = factory.createTree(objective, BT::Blackboard::create(global_blackboard));

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING && rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    status = tree.tickExactlyOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });
  }
  return status;
}

}  // namespace stepit_tests
