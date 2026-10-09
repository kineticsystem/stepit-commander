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

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_ros2/tree_execution_server.hpp>
#include <std_msgs/msg/string.hpp>

#include "stepit_server/execution_status.hpp"
#include "stepit_server/payload.hpp"
#include "stepit_server/tree_loader.hpp"

namespace stepit_server
{

/**
 * @brief The single action server through which the robot is commanded.
 *
 * A client sends the name of an objective (a behavior tree published by any
 * package listed in the parameter `behavior_trees`) together with a payload
 * holding its parameters. The payload is copied into the global blackboard of
 * the tree, where the behaviors read it through the '@' prefix, e.g. {@offset}.
 *
 * An objective is the main tree of its file, its `main_tree_to_execute`. A goal
 * for any other tree, a subtree, is rejected: a subtree only runs inside
 * another tree, which includes it with a SubTree node.
 *
 * With the parameter `preempt` true, the default, a goal accepted while an
 * objective runs replaces it: the running tree is halted at its next tick and
 * its goal aborted, and the new objective starts. With `preempt` false, the new
 * goal waits for the running objective to end.
 *
 * Whoever sent the goal, every client can follow the run: the server publishes
 * the running objective on `~/objective`, and the whole run, every node with
 * its status, on `~/execution`, both latched, see ExecutionStatus::snapshot().
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

  /// @brief Publish the parameters of the command into the global blackboard,
  /// and the name of the objective on `~/objective`.
  void onTreeCreated(BT::Tree& tree) override;

  /// @brief Remember whether the tick ended the tree, for onLoopFeedback, and
  /// end it if a new goal preempts it.
  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;

  /// @brief The status of the nodes that changed, see ExecutionStatus, and the
  /// whole run on `~/execution` when it changed.
  std::optional<std::string> onLoopFeedback() override;

  /// @brief Publish how the run ended on `~/execution`, and on `~/objective` that
  /// no objective runs any more.
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
  std::unique_ptr<ExecutionStatus> execution_status_;
  /// @brief The status of the last tick.
  BT::NodeStatus tick_status_ = BT::NodeStatus::IDLE;

  /// @brief The objective running, latched, empty when none: a client connecting
  /// at any time knows what runs, whoever sent it.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr objective_publisher_;

  /// @brief Publish the name of the objective running, or "" for none.
  void publishObjective(const std::string& name);

  /// @brief The last run, running or ended, latched, see ExecutionStatus::snapshot().
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_publisher_;
  /// @brief The number of the last run: the first is 1.
  std::uint64_t runs_ = 0;

  /// @brief Whether a new goal replaces the running objective, the parameter `preempt`.
  bool preempt_;
  /// @brief Whether an objective is running: set when its tree is created, cleared when it ends,
  /// a tree that throws included: BehaviorTree.ROS2, our fork, calls onTreeExecutionCompleted then.
  std::atomic<bool> running_{ false };
  /// @brief Set by a new goal, read by the running tree's loop, which then ends it.
  std::atomic<bool> preempt_requested_{ false };
  /// @brief Whether the running tree is ending because it was preempted.
  bool preempted_ = false;
  /// @brief The objective that preempts the running one, for the result of the latter.
  std::mutex preempting_mutex_;
  std::string preempting_objective_;
};

}  // namespace stepit_server
