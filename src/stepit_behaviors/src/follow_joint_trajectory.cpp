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

#include "stepit_behaviors/follow_joint_trajectory.hpp"

namespace stepit_behaviors
{

FollowJointTrajectory::FollowJointTrajectory(const std::string& name, const BT::NodeConfig& config,
                                             const BT::RosNodeParams& params)
  : BT::RosActionNode<control_msgs::action::FollowJointTrajectory>(name, config, params)
{
}

BT::PortsList FollowJointTrajectory::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<trajectory_msgs::msg::JointTrajectory>("trajectory", "the trajectory to follow"),
  });
}

bool FollowJointTrajectory::setGoal(Goal& goal)
{
  const auto trajectory = getInput<trajectory_msgs::msg::JointTrajectory>("trajectory");
  if (!trajectory)
  {
    throw BT::RuntimeError("FollowJointTrajectory: ", trajectory.error());
  }
  if (trajectory->joint_names.empty() || trajectory->points.empty())
  {
    throw BT::RuntimeError("FollowJointTrajectory: the trajectory has no joint or no point");
  }

  goal.trajectory = trajectory.value();

  RCLCPP_INFO(logger(), "%s: moving %zu joint(s) through %zu point(s)", name().c_str(),
              goal.trajectory.joint_names.size(), goal.trajectory.points.size());

  return true;
}

BT::NodeStatus FollowJointTrajectory::onResultReceived(const WrappedResult& result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(logger(), "%s: the trajectory was aborted or cancelled", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (result.result->error_code != control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
  {
    RCLCPP_ERROR(logger(), "%s: the controller returned error %d: %s", name().c_str(), result.result->error_code,
                 result.result->error_string.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger(), "%s: the trajectory was executed", name().c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowJointTrajectory::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

void FollowJointTrajectory::onHalt()
{
  RCLCPP_INFO(logger(), "%s: halted", name().c_str());
}

}  // namespace stepit_behaviors
