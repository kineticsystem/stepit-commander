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

#include <cmath>

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
      BT::InputPort<std::vector<std::string>>("joint_names", "joints to move"),
      BT::InputPort<std::vector<double>>("positions", "absolute target positions, in radians"),
      BT::InputPort<double>("duration", "time to reach the target, in seconds (default 5.0)"),
  });
}

bool FollowJointTrajectory::setGoal(Goal& goal)
{
  const auto joint_names = getInput<std::vector<std::string>>("joint_names");
  if (!joint_names)
  {
    throw BT::RuntimeError("FollowJointTrajectory: ", joint_names.error());
  }

  const auto positions = getInput<std::vector<double>>("positions");
  if (!positions)
  {
    throw BT::RuntimeError("FollowJointTrajectory: ", positions.error());
  }

  if (joint_names.value().size() != positions.value().size())
  {
    throw BT::RuntimeError("FollowJointTrajectory: ", std::to_string(joint_names->size()), " joints but ",
                           std::to_string(positions->size()), " positions were given");
  }
  if (joint_names.value().empty())
  {
    throw BT::RuntimeError("FollowJointTrajectory: no joint to move");
  }

  // The duration is optional: when the port is not set, or the blackboard entry
  // it points at does not exist, fall back to the default.
  const double duration = getInput<double>("duration").value_or(kDefaultDuration);
  if (!(duration > 0.0))
  {
    throw BT::RuntimeError("FollowJointTrajectory: the duration must be positive");
  }

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions.value();
  // Come to a full stop on the target.
  point.velocities.assign(positions.value().size(), 0.0);
  double seconds = 0.0;
  const double fraction = std::modf(duration, &seconds);
  point.time_from_start.sec = static_cast<std::int32_t>(seconds);
  point.time_from_start.nanosec = static_cast<std::uint32_t>(std::lround(fraction * 1e9));

  goal.trajectory.joint_names = joint_names.value();
  goal.trajectory.points = { point };

  RCLCPP_INFO(logger(), "%s: moving %zu joint(s) in %.2f s", name().c_str(), joint_names.value().size(), duration);

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
