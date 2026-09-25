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

#include "stepit_behaviors/get_joint_positions.hpp"

#include <algorithm>

namespace stepit_behaviors
{

GetJointPositions::GetJointPositions(const std::string& name, const BT::NodeConfig& config,
                                     const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config), node_(params.nh), logger_(rclcpp::get_logger("GetJointPositions"))
{
  if (const auto node = node_.lock())
  {
    logger_ = node->get_logger();
  }
  // Subscribe now, so that a message has usually arrived by the first tick. A
  // topic name read from the blackboard is only known when ticked.
  subscribe();
}

BT::PortsList GetJointPositions::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "/joint_states", "topic the joint states are published on"),
    BT::InputPort<std::vector<std::string>>("joint_names", "joints to read"),
    BT::InputPort<double>("timeout", 2.0, "how long to wait for the first joint state, in seconds"),
    BT::OutputPort<std::vector<double>>("positions", "current joint positions, in radians"),
  };
}

bool GetJointPositions::subscribe()
{
  if (subscription_)
  {
    return true;
  }
  const auto topic_name = getInput<std::string>("topic_name");
  if (!topic_name)
  {
    return false;
  }
  auto node = node_.lock();
  if (!node)
  {
    throw BT::RuntimeError("GetJointPositions: the ROS node went out of scope");
  }

  // A callback group of our own, spun only when ticked, as the nodes of
  // BehaviorTree.ROS2 do: the tree is ticked from a single thread.
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
      topic_name.value(), rclcpp::QoS{ 1 },
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { last_msg_ = msg; }, options);
  return true;
}

BT::NodeStatus GetJointPositions::onStart()
{
  if (!subscribe())
  {
    throw BT::RuntimeError("GetJointPositions: ", getInput<std::string>("topic_name").error());
  }
  const auto timeout = getInput<double>("timeout");
  if (!timeout)
  {
    throw BT::RuntimeError("GetJointPositions: ", timeout.error());
  }
  timeout_ = timeout.value();
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_));
  return read();
}

BT::NodeStatus GetJointPositions::onRunning()
{
  const auto status = read();
  if (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() >= deadline_)
  {
    RCLCPP_ERROR(logger_, "%s: no joint state received on %s within %.1f s", name().c_str(),
                 subscription_->get_topic_name(), timeout_);
    return BT::NodeStatus::FAILURE;
  }
  return status;
}

void GetJointPositions::onHalted()
{
}

BT::NodeStatus GetJointPositions::read()
{
  executor_.spin_some();
  if (!last_msg_)
  {
    return BT::NodeStatus::RUNNING;
  }

  const auto joint_names = getInput<std::vector<std::string>>("joint_names");
  if (!joint_names)
  {
    throw BT::RuntimeError("GetJointPositions: ", joint_names.error());
  }

  std::vector<double> positions;
  positions.reserve(joint_names.value().size());
  for (const auto& joint_name : joint_names.value())
  {
    const auto it = std::find(last_msg_->name.cbegin(), last_msg_->name.cend(), joint_name);
    if (it == last_msg_->name.cend())
    {
      RCLCPP_ERROR(logger_, "%s: joint '%s' is not published on %s", name().c_str(), joint_name.c_str(),
                   subscription_->get_topic_name());
      return BT::NodeStatus::FAILURE;
    }
    const auto index = static_cast<std::size_t>(std::distance(last_msg_->name.cbegin(), it));
    if (index >= last_msg_->position.size())
    {
      RCLCPP_ERROR(logger_, "%s: no position published for joint '%s'", name().c_str(), joint_name.c_str());
      return BT::NodeStatus::FAILURE;
    }
    positions.push_back(last_msg_->position[index]);
  }

  setOutput("positions", positions);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace stepit_behaviors
