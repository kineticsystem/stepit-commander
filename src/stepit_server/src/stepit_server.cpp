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

#include "stepit_server/stepit_server.hpp"

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_utils.hpp>

namespace stepit_server
{

CommanderServer::CommanderServer(const rclcpp::NodeOptions& options) : BT::TreeExecutionServer(options)
{
  preempt_ = node()->declare_parameter<bool>("preempt", true);
  // Latched: the last value reaches a client that subscribes later.
  objective_publisher_ =
      node()->create_publisher<std_msgs::msg::String>("~/objective", rclcpp::QoS{ 1 }.reliable().transient_local());
  publishObjective("");
  execution_publisher_ =
      node()->create_publisher<std_msgs::msg::String>("~/execution", rclcpp::QoS{ 1 }.reliable().transient_local());
}

void CommanderServer::publishObjective(const std::string& name)
{
  std_msgs::msg::String message;
  message.data = name;
  objective_publisher_->publish(message);
}

bool CommanderServer::onGoalReceived(const std::string& tree_name, const std::string& payload)
{
  try
  {
    payload_ = parsePayload(payload);
  }
  catch (const PayloadError& ex)
  {
    RCLCPP_ERROR(node()->get_logger(), "Rejecting objective '%s': %s", tree_name.c_str(), ex.what());
    return false;
  }

  reloadTrees();
  const auto trees = factory().registeredBehaviorTrees();
  if (std::find(trees.begin(), trees.end(), tree_name) == trees.end())
  {
    RCLCPP_ERROR(node()->get_logger(), "Rejecting objective '%s': no behavior tree has this ID", tree_name.c_str());
    return false;
  }
  if (!tree_loader_.isObjective(tree_name))
  {
    RCLCPP_ERROR(node()->get_logger(),
                 "Rejecting objective '%s': it is a subtree, which only runs inside another tree. Make it the "
                 "main_tree_to_execute of its file to run it on its own",
                 tree_name.c_str());
    return false;
  }

  // The server waits for the running objective to end before it starts this
  // one: have the running tree end at its next tick.
  if (preempt_ && running_)
  {
    {
      const std::lock_guard<std::mutex> lock{ preempting_mutex_ };
      preempting_objective_ = tree_name;
    }
    preempt_requested_ = true;
    RCLCPP_INFO(node()->get_logger(), "Objective '%s' preempts the running objective", tree_name.c_str());
  }

  RCLCPP_INFO(node()->get_logger(), "Executing objective '%s' with %zu parameter(s)", tree_name.c_str(),
              payload_.size());
  return true;
}

void CommanderServer::reloadTrees()
{
  // Kept in a variable: as_string_array() returns a reference into the parameter.
  const auto parameter = node()->get_parameter("behavior_trees");
  std::vector<std::filesystem::path> folders;
  for (const auto& value : parameter.as_string_array())
  {
    if (const auto folder = BT::GetDirectoryPath(value); !folder.empty())
    {
      folders.emplace_back(folder);
    }
  }

  const auto result = tree_loader_.reloadIfChanged(factory(), folders);
  if (result.reloaded && !result.first)
  {
    std::string changed;
    for (const auto& file : result.changed)
    {
      changed += (changed.empty() ? "" : ", ") + file;
    }
    RCLCPP_INFO(node()->get_logger(), "Reloaded the behavior trees, changed: %s", changed.c_str());
  }
  for (const auto& error : result.errors)
  {
    RCLCPP_ERROR(node()->get_logger(), "Failed to load a behavior tree: %s", error.c_str());
  }
}

void CommanderServer::onTreeCreated(BT::Tree& tree)
{
  // The parameters of a previous goal must not leak into this one.
  for (const auto& key : written_keys_)
  {
    globalBlackboard()->unset(key);
  }
  written_keys_.clear();

  writeToBlackboard(payload_, *globalBlackboard());
  for (const auto& [key, value] : payload_)
  {
    written_keys_.push_back(key);
  }

  publishObjective(tree.subtrees.empty() ? "" : tree.subtrees.front()->tree_ID);

  logger_ = std::make_shared<BT::StdCoutLogger>(tree);
  execution_status_ = std::make_unique<ExecutionStatus>(tree, ExecutionStatus::kDefaultPeriod, ++runs_);
  tick_status_ = BT::NodeStatus::IDLE;

  // A request made while the previous tree was ending was meant for that tree:
  // the server created this one only once the previous one had ended.
  preempt_requested_ = false;
  preempted_ = false;
  running_ = true;
}

std::optional<BT::NodeStatus> CommanderServer::onLoopAfterTick(BT::NodeStatus status)
{
  tick_status_ = status;
  if (status == BT::NodeStatus::RUNNING && preempt_requested_.exchange(false))
  {
    // The server halts the tree and aborts its goal.
    preempted_ = true;
    return BT::NodeStatus::FAILURE;
  }
  return std::nullopt;
}

std::optional<std::string> CommanderServer::onLoopFeedback()
{
  if (!execution_status_)
  {
    return std::nullopt;
  }
  auto feedback = execution_status_->feedback(tick_status_ != BT::NodeStatus::RUNNING);
  if (feedback)
  {
    // The run changed: the whole of it, for a client that follows every run.
    std_msgs::msg::String message;
    message.data = execution_status_->snapshot();
    execution_publisher_->publish(message);
  }
  return feedback;
}

std::optional<std::string> CommanderServer::onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled)
{
  std::optional<std::string> result;
  if (preempted_)
  {
    const std::lock_guard<std::mutex> lock{ preempting_mutex_ };
    result = "Preempted by objective '" + preempting_objective_ + "'";
  }
  if (execution_status_)
  {
    std_msgs::msg::String message;
    message.data = execution_status_->snapshot(ExecutionStatus::Ending{ status, was_cancelled, result.value_or("") });
    execution_publisher_->publish(message);
  }
  running_ = false;
  publishObjective("");
  logger_.reset();
  execution_status_.reset();
  return result;
}

}  // namespace stepit_server
