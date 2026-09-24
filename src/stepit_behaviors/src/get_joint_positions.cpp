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
  : BT::RosTopicSubNode<sensor_msgs::msg::JointState>(name, config, params)
{
}

BT::PortsList GetJointPositions::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::vector<std::string>>("joint_names", "joints to read"),
      BT::OutputPort<std::vector<double>>("positions", "current joint positions, in radians"),
  });
}

BT::NodeStatus GetJointPositions::onTick(const std::shared_ptr<sensor_msgs::msg::JointState>& msg)
{
  if (!msg)
  {
    RCLCPP_DEBUG(logger(), "%s: no joint state received yet", name().c_str());
    return BT::NodeStatus::FAILURE;
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
    const auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_name);
    if (it == msg->name.cend())
    {
      RCLCPP_ERROR(logger(), "%s: joint '%s' is not published on the joint state topic", name().c_str(),
                   joint_name.c_str());
      return BT::NodeStatus::FAILURE;
    }
    const auto index = static_cast<std::size_t>(std::distance(msg->name.cbegin(), it));
    if (index >= msg->position.size())
    {
      RCLCPP_ERROR(logger(), "%s: no position published for joint '%s'", name().c_str(), joint_name.c_str());
      return BT::NodeStatus::FAILURE;
    }
    positions.push_back(msg->position[index]);
  }

  setOutput("positions", positions);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace stepit_behaviors
