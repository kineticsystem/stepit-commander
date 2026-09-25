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

#include "stepit_behaviors/trapezoidal_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <rclcpp/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "stepit_behaviors/ports.hpp"

namespace stepit_behaviors
{
namespace
{

/// @brief One value per joint, from a port holding one for every joint or one per joint.
std::vector<double> perJoint(const std::vector<double>& values, std::size_t joints, const std::string& port)
{
  if (values.size() == 1)
  {
    return std::vector<double>(joints, values.front());
  }
  if (values.size() != joints)
  {
    throw BT::RuntimeError("TrapezoidalTrajectory: ", std::to_string(values.size()), " values of [", port, "] for ",
                           std::to_string(joints), " joints: give one for every joint, or one per joint");
  }
  return values;
}

/// @brief A number as text that parses back to exactly the same number: a port
/// default is text, and the default precision would round a limit up.
std::string exactly(double value)
{
  std::ostringstream text;
  text.precision(std::numeric_limits<double>::max_digits10);
  text << value;
  return text.str();
}

/// @brief A time as a message, rounded up to the nanosecond: a phase is never
/// shortened by the rounding, so it never accelerates beyond the limit.
builtin_interfaces::msg::Duration toDuration(double seconds)
{
  return rclcpp::Duration::from_nanoseconds(static_cast<std::int64_t>(std::ceil(seconds * 1e9)));
}

}  // namespace

TrapezoidalTrajectory::TrapezoidalTrajectory(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList TrapezoidalTrajectory::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("joint_names", "joints to move"),
    BT::InputPort<std::vector<double>>("start_positions", "where the joints are, in radians"),
    BT::InputPort<std::vector<double>>("positions", "absolute target positions, in radians"),
    BT::InputPort<BT::AnyTypeAllowed>("max_velocity", exactly(kMaxVelocity),
                                      "top speed, in rad/s: one for every joint, or one per joint; "
                                      "by default 90% of the limit of the StepIt motors, 2.7 turns/s"),
    BT::InputPort<BT::AnyTypeAllowed>("max_acceleration", exactly(kMaxAcceleration),
                                      "acceleration, in rad/s²: one for every joint, or one per joint; "
                                      "by default 90% of the limit of the StepIt motors, 1.8 turns/s²"),
    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory", "the trajectory, for FollowJointTrajectory"),
  };
}

BT::NodeStatus TrapezoidalTrajectory::tick()
{
  const auto joint_names = getInput<std::vector<std::string>>("joint_names");
  if (!joint_names)
  {
    throw BT::RuntimeError("TrapezoidalTrajectory: ", joint_names.error());
  }
  const auto start = getInput<std::vector<double>>("start_positions");
  if (!start)
  {
    throw BT::RuntimeError("TrapezoidalTrajectory: ", start.error());
  }
  const auto target = getInput<std::vector<double>>("positions");
  if (!target)
  {
    throw BT::RuntimeError("TrapezoidalTrajectory: ", target.error());
  }

  const auto joints = joint_names->size();
  if (joints == 0)
  {
    throw BT::RuntimeError("TrapezoidalTrajectory: no joint to move");
  }
  if (start->size() != joints || target->size() != joints)
  {
    throw BT::RuntimeError("TrapezoidalTrajectory: ", std::to_string(joints), " joints but ",
                           std::to_string(start->size()), " start positions and ", std::to_string(target->size()),
                           " positions were given");
  }

  const auto max_velocity = perJoint(getNumbersOr(*this, "max_velocity", kMaxVelocity), joints, "max_velocity");
  const auto max_acceleration =
      perJoint(getNumbersOr(*this, "max_acceleration", kMaxAcceleration), joints, "max_acceleration");
  for (std::size_t i = 0; i < joints; ++i)
  {
    if (!(max_velocity[i] > 0.0) || !(max_acceleration[i] > 0.0))
    {
      throw BT::RuntimeError("TrapezoidalTrajectory: the limits must be positive");
    }
  }

  // Every joint follows the same profile, scaled by its distance: its position
  // is start + s * delta, where s goes from 0 to 1. The limits of s are the
  // tightest of the joints' limits divided by their distance, so that no joint
  // exceeds its own.
  std::vector<double> delta(joints);
  double v = std::numeric_limits<double>::infinity();
  double a = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < joints; ++i)
  {
    delta[i] = (*target)[i] - (*start)[i];
    const double distance = std::abs(delta[i]);
    if (distance > 0.0)
    {
      v = std::min(v, max_velocity[i] / distance);
      a = std::min(a, max_acceleration[i] / distance);
    }
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joint_names.value();

  // A waypoint of the profile: at time t, s of the way, moving at s_dot.
  auto add_point = [&](double t, double s, double s_dot) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (std::size_t i = 0; i < joints; ++i)
    {
      point.positions.push_back((*start)[i] + s * delta[i]);
      point.velocities.push_back(s_dot * delta[i]);
    }
    point.time_from_start = toDuration(t);
    trajectory.points.push_back(point);
  };

  if (std::isinf(v))
  {
    // Nothing moves: the joints are already on their targets.
    add_point(kStandStillDuration, 1.0, 0.0);
  }
  else if (v * v / a >= 1.0)
  {
    // Triangle: half the way accelerating, half braking, never at top speed.
    const double ta = std::sqrt(1.0 / a);
    add_point(ta, 0.5, a * ta);
    add_point(2.0 * ta, 1.0, 0.0);
  }
  else
  {
    // Trapezoid: accelerate to top speed, cruise, brake.
    const double ta = v / a;
    const double duration = 1.0 / v + ta;
    const double s_ramp = 0.5 * a * ta * ta;
    add_point(ta, s_ramp, v);
    add_point(duration - ta, 1.0 - s_ramp, v);
    add_point(duration, 1.0, 0.0);
  }

  setOutput("trajectory", trajectory);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace stepit_behaviors
