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

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/duration.hpp>
#include <stepit_behaviors/trapezoidal_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace stepit_behaviors::test
{
namespace
{
using trajectory_msgs::msg::JointTrajectory;

constexpr double kTurn = 2.0 * M_PI;
constexpr double kVelocity = TrapezoidalTrajectory::kMaxVelocity;
constexpr double kAcceleration = TrapezoidalTrajectory::kMaxAcceleration;

/// @brief The tree around the node: the joints and positions come from the blackboard.
BT::Tree makeTree(const std::string& limits, BT::Blackboard::Ptr blackboard)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TrapezoidalTrajectory>("TrapezoidalTrajectory");
  return factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main">)"
                                    R"(<TrapezoidalTrajectory joint_names="{joints}" start_positions="{start}" )"
                                    R"(positions="{target}" trajectory="{trajectory}" )" +
                                        limits + R"(/></BehaviorTree></root>)",
                                    blackboard);
}

BT::Blackboard::Ptr blackboard(const std::vector<double>& start, const std::vector<double>& target)
{
  std::vector<std::string> joints;
  for (std::size_t i = 0; i < start.size(); ++i)
  {
    joints.push_back("joint" + std::to_string(i + 1));
  }
  auto bb = BT::Blackboard::create();
  bb->set("joints", joints);
  bb->set("start", start);
  bb->set("target", target);
  return bb;
}

/// @brief The trajectory from start to target, with the given limit attributes.
JointTrajectory build(const std::vector<double>& start, const std::vector<double>& target,
                      const std::string& limits = "")
{
  auto bb = blackboard(start, target);
  auto tree = makeTree(limits, bb);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  return bb->get<JointTrajectory>("trajectory");
}

void expectRejected(const std::vector<double>& start, const std::vector<double>& target, const std::string& limits)
{
  auto bb = blackboard(start, target);
  auto tree = makeTree(limits, bb);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

/// @brief Tolerance on times, which the node rounds up to the nanosecond.
constexpr double kTimeTolerance = 2e-9;

double seconds(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return rclcpp::Duration(point.time_from_start).seconds();
}

/// @brief Every joint stays within its limits, between rest at the start and each waypoint.
void expectWithinLimits(const JointTrajectory& trajectory, const std::vector<double>& start,
                        const std::vector<double>& max_velocity, const std::vector<double>& max_acceleration)
{
  constexpr double kTolerance = 1e-9;
  std::vector<double> velocity(start.size(), 0.0);
  double time = 0.0;
  for (const auto& point : trajectory.points)
  {
    const double dt = seconds(point) - time;
    ASSERT_GT(dt, 0.0);
    for (std::size_t i = 0; i < start.size(); ++i)
    {
      EXPECT_LE(std::abs(point.velocities[i]), max_velocity[i] + kTolerance) << "joint " << i;
      EXPECT_LE(std::abs(point.velocities[i] - velocity[i]) / dt, max_acceleration[i] + kTolerance) << "joint " << i;
      velocity[i] = point.velocities[i];
    }
    time = seconds(point);
  }
}
}  // namespace

// Five turns: long enough to reach the top speed, which takes 1.5 s and about
// 2 turns at the default acceleration, so the rest of the way is a cruise.
TEST(TrapezoidalTrajectoryNode, ALongMoveCruisesAtTopSpeed)
{
  const double distance = 5.0 * kTurn;
  const auto trajectory = build({ 0.0 }, { -distance });

  ASSERT_EQ(trajectory.points.size(), 3u);
  const double ta = kVelocity / kAcceleration;
  const double ramp = 0.5 * kAcceleration * ta * ta;
  const double duration = distance / kVelocity + ta;

  EXPECT_NEAR(seconds(trajectory.points[0]), ta, kTimeTolerance);
  EXPECT_NEAR(trajectory.points[0].positions[0], -ramp, 1e-9);
  EXPECT_NEAR(trajectory.points[0].velocities[0], -kVelocity, 1e-9);

  EXPECT_NEAR(seconds(trajectory.points[1]), duration - ta, kTimeTolerance);
  EXPECT_NEAR(trajectory.points[1].positions[0], -distance + ramp, 1e-9);
  EXPECT_NEAR(trajectory.points[1].velocities[0], -kVelocity, 1e-9);

  EXPECT_NEAR(seconds(trajectory.points[2]), duration, kTimeTolerance);
  EXPECT_NEAR(trajectory.points[2].positions[0], -distance, 1e-9);
  EXPECT_EQ(trajectory.points[2].velocities[0], 0.0);

  expectWithinLimits(trajectory, { 0.0 }, { kVelocity }, { kAcceleration });
}

// One radian is too short to reach the top speed: accelerate, then brake.
TEST(TrapezoidalTrajectoryNode, AShortMoveIsATriangle)
{
  const auto trajectory = build({ 1.0 }, { 2.0 });

  ASSERT_EQ(trajectory.points.size(), 2u);
  const double ta = std::sqrt(1.0 / kAcceleration);
  EXPECT_NEAR(seconds(trajectory.points[0]), ta, kTimeTolerance);
  EXPECT_NEAR(trajectory.points[0].positions[0], 1.5, 1e-9);
  EXPECT_NEAR(trajectory.points[0].velocities[0], kAcceleration * ta, 1e-9);
  EXPECT_LT(trajectory.points[0].velocities[0], kVelocity);
  EXPECT_NEAR(seconds(trajectory.points[1]), 2.0 * ta, kTimeTolerance);
  EXPECT_NEAR(trajectory.points[1].positions[0], 2.0, 1e-9);
  EXPECT_EQ(trajectory.points[1].velocities[0], 0.0);
}

// The joint with the longest way sets the pace; the others move in proportion,
// so they all start and stop together.
TEST(TrapezoidalTrajectoryNode, AllJointsArriveTogether)
{
  const std::vector<double> start{ 0.0, 1.0, 2.0 };
  const std::vector<double> target{ -5.0 * kTurn, 1.0 + kTurn, 2.0 };
  const auto trajectory = build(start, target);

  ASSERT_EQ(trajectory.points.size(), 3u);
  EXPECT_NEAR(seconds(trajectory.points.back()), 5.0 * kTurn / kVelocity + kVelocity / kAcceleration, kTimeTolerance);
  for (std::size_t i = 0; i < start.size(); ++i)
  {
    EXPECT_NEAR(trajectory.points.back().positions[i], target[i], 1e-9);
  }
  for (const auto& point : trajectory.points)
  {
    EXPECT_NEAR(point.velocities[1], -point.velocities[0] / 5.0, 1e-9);
    EXPECT_EQ(point.velocities[2], 0.0);
    EXPECT_EQ(point.positions[2], 2.0);
  }
  expectWithinLimits(trajectory, start, std::vector<double>(3, kVelocity), std::vector<double>(3, kAcceleration));
}

// A joint with lower limits can set the pace even with the shorter way.
TEST(TrapezoidalTrajectoryNode, EachJointHasItsOwnLimits)
{
  const std::vector<double> start{ 0.0, 0.0 };
  const std::vector<double> target{ 4.0, 2.0 };
  const std::vector<double> max_velocity{ 10.0, 1.0 };
  const std::vector<double> max_acceleration{ 10.0, 1.0 };
  const auto trajectory = build(start, target, R"(max_velocity="10.0;1.0" max_acceleration="10.0;1.0")");

  // Joint 2 alone: 2 rad at 1 rad/s and 1 rad/s² takes 3 s.
  EXPECT_NEAR(seconds(trajectory.points.back()), 3.0, kTimeTolerance);
  expectWithinLimits(trajectory, start, max_velocity, max_acceleration);
}

TEST(TrapezoidalTrajectoryNode, TheLimitsCanBeLowered)
{
  const auto trajectory = build({ 0.0 }, { 10.0 }, R"(max_velocity="2.0" max_acceleration="1.0")");
  // 2 s to reach 2 rad/s, covering 2 rad; 6 rad of cruise; 2 s of braking.
  ASSERT_EQ(trajectory.points.size(), 3u);
  EXPECT_NEAR(seconds(trajectory.points[0]), 2.0, kTimeTolerance);
  EXPECT_NEAR(seconds(trajectory.points[1]), 5.0, kTimeTolerance);
  EXPECT_NEAR(seconds(trajectory.points[2]), 7.0, kTimeTolerance);
}

// A limit missing from the payload falls back on the motors' limit.
TEST(TrapezoidalTrajectoryNode, AMissingLimitFallsBackOnTheDefault)
{
  const auto trajectory = build({ 0.0 }, { 1.0 }, R"(max_acceleration="{@max_acceleration}")");
  EXPECT_NEAR(seconds(trajectory.points.back()), 2.0 * std::sqrt(1.0 / kAcceleration), kTimeTolerance);
}

TEST(TrapezoidalTrajectoryNode, NothingToMove)
{
  const auto trajectory = build({ 0.5, -1.0 }, { 0.5, -1.0 });
  ASSERT_EQ(trajectory.points.size(), 1u);
  EXPECT_EQ(trajectory.points[0].positions, (std::vector<double>{ 0.5, -1.0 }));
  EXPECT_EQ(trajectory.points[0].velocities, (std::vector<double>{ 0.0, 0.0 }));
  EXPECT_GT(seconds(trajectory.points[0]), 0.0);
}

TEST(TrapezoidalTrajectoryNode, OneStartAndOneTargetPerJoint)
{
  auto bb = blackboard({ 0.0, 0.0 }, { 1.0 });
  auto tree = makeTree("", bb);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

TEST(TrapezoidalTrajectoryNode, OneLimitForEveryJointOrOnePerJoint)
{
  expectRejected({ 0.0, 0.0, 0.0 }, { 1.0, 1.0, 1.0 }, R"(max_velocity="1.0;2.0")");
}

TEST(TrapezoidalTrajectoryNode, PositiveLimits)
{
  expectRejected({ 0.0 }, { 1.0 }, R"(max_acceleration="0.0")");
}

TEST(TrapezoidalTrajectoryNode, LimitsThatAreNotNumbers)
{
  expectRejected({ 0.0 }, { 1.0 }, R"(max_velocity="fast")");
}

}  // namespace stepit_behaviors::test
