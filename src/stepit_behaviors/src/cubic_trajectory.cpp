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

#include "stepit_behaviors/cubic_trajectory.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace stepit_behaviors
{

CubicTrajectory::CubicTrajectory(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList CubicTrajectory::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("joint_names", "joints to move"),
    BT::InputPort<std::vector<double>>("positions", "absolute target positions, in radians"),
    BT::InputPort<double>("duration", "time to reach the target, in seconds (default 5.0)"),
    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory", "the trajectory, for FollowJointTrajectory"),
  };
}

BT::NodeStatus CubicTrajectory::tick()
{
  const auto joint_names = getInput<std::vector<std::string>>("joint_names");
  if (!joint_names)
  {
    throw BT::RuntimeError("CubicTrajectory: ", joint_names.error());
  }

  const auto positions = getInput<std::vector<double>>("positions");
  if (!positions)
  {
    throw BT::RuntimeError("CubicTrajectory: ", positions.error());
  }

  if (joint_names.value().size() != positions.value().size())
  {
    throw BT::RuntimeError("CubicTrajectory: ", std::to_string(joint_names->size()), " joints but ",
                           std::to_string(positions->size()), " positions were given");
  }
  if (joint_names.value().empty())
  {
    throw BT::RuntimeError("CubicTrajectory: no joint to move");
  }

  // The duration is optional: when the port is not set, or the blackboard entry
  // it points at does not exist, fall back to the default.
  const double duration = getInput<double>("duration").value_or(kDefaultDuration);
  if (!(duration > 0.0))
  {
    throw BT::RuntimeError("CubicTrajectory: the duration must be positive");
  }

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions.value();
  // Come to a full stop on the target.
  point.velocities.assign(positions.value().size(), 0.0);
  double seconds = 0.0;
  const double fraction = std::modf(duration, &seconds);
  point.time_from_start.sec = static_cast<std::int32_t>(seconds);
  point.time_from_start.nanosec = static_cast<std::uint32_t>(std::lround(fraction * 1e9));

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joint_names.value();
  trajectory.points = { point };
  setOutput("trajectory", trajectory);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace stepit_behaviors
