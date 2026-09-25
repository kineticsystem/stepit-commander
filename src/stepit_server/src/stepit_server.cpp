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

  logger_ = std::make_shared<BT::StdCoutLogger>(tree);
}

std::optional<std::string> CommanderServer::onTreeExecutionCompleted(BT::NodeStatus, bool)
{
  logger_.reset();
  return std::nullopt;
}

}  // namespace stepit_server
