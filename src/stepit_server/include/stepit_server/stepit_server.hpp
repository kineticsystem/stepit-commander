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

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_ros2/tree_execution_server.hpp>

#include "stepit_server/payload.hpp"
#include "stepit_server/tree_loader.hpp"

namespace stepit_server
{

/**
 * @brief The single action server through which the robot is commanded.
 *
 * A client sends the name of an objective (a behavior tree published by
 * stepit_objectives, or by any other package listed in the parameter
 * `behavior_trees`) together with a payload holding its parameters. The payload
 * is copied into the global blackboard of the tree, where the behaviors read it
 * through the '@' prefix, e.g. {@offset}.
 */
class CommanderServer : public BT::TreeExecutionServer
{
public:
  explicit CommanderServer(const rclcpp::NodeOptions& options);

protected:
  /**
   * @brief Reject the goal when its payload cannot be understood, or when its
   * objective does not exist. The trees are reloaded first if their files
   * changed, so that the goal runs them as they are on disk.
   */
  bool onGoalReceived(const std::string& tree_name, const std::string& payload) override;

  /// @brief Publish the parameters of the command into the global blackboard.
  void onTreeCreated(BT::Tree& tree) override;

  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) override;

private:
  /// @brief Reload the trees of the `behavior_trees` folders if their files changed.
  void reloadTrees();

  /// @brief Parameters of the goal being executed.
  Payload payload_;
  /// @brief Blackboard entries written for the previous goal.
  std::vector<std::string> written_keys_;
  std::shared_ptr<BT::StdCoutLogger> logger_;
  TreeLoader tree_loader_;
};

}  // namespace stepit_server
