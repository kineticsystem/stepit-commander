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

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace stepit_behaviors
{

/**
 * @brief Builds the fastest trajectory to a set of absolute joint positions that
 * the speed and acceleration limits allow: a trapezoidal velocity profile.
 *
 * Each joint accelerates at its limit, cruises at its top speed, and brakes at
 * its limit. A move too short to reach the top speed becomes a triangle:
 * accelerate, then brake at once. All joints start and stop together, moving in
 * proportion to their distance: the joint that needs the most time sets the
 * pace, and the others follow the same profile, scaled down.
 *
 * The trajectory holds 2 or 3 waypoints, with positions and velocities: the end
 * of the acceleration, the start of the braking, and the arrival, at rest. The
 * controller joins them with cubics, which follow a constant acceleration
 * exactly, so the waypoints describe the whole profile.
 *
 * Unlike CubicTrajectory, it needs the positions the joints start from,
 * e.g. from GetJointPositions: the profile depends on the distance.
 */
class TrapezoidalTrajectory : public BT::SyncActionNode
{
public:
  /// @brief Top speed of the StepIt motors, in rad/s: 3 turns/s.
  static constexpr double kMotorMaxVelocity = 18.8495559215388;

  /// @brief Acceleration limit of the StepIt motors, in rad/s²: 2 turns/s².
  static constexpr double kMotorMaxAcceleration = 12.5663706143592;

  /// @brief Share of the motors' limits used by default. The microcontroller
  /// follows the commanded positions with its own ramp, limited to the same
  /// values, and trails them: at 100% a motor cannot catch up, and arrives too
  /// late for the controller's goal tolerance (measured with SpinTest, joint 5).
  static constexpr double kDefaultShareOfLimits = 0.9;

  /// @brief Default top speed, in rad/s: 90% of the motors', 2.7 turns/s.
  static constexpr double kMaxVelocity = kDefaultShareOfLimits * kMotorMaxVelocity;

  /// @brief Default acceleration, in rad/s²: 90% of the motors', 1.8 turns/s².
  static constexpr double kMaxAcceleration = kDefaultShareOfLimits * kMotorMaxAcceleration;

  /// @brief Duration of a move that goes nowhere, in seconds: the controller needs one.
  static constexpr double kStandStillDuration = 0.01;

  TrapezoidalTrajectory(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace stepit_behaviors
