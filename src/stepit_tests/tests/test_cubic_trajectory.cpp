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

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <stepit_behaviors/cubic_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace stepit_behaviors::test
{
namespace
{
using trajectory_msgs::msg::JointTrajectory;

/// @brief Tick CubicTrajectory once with the given attributes, and return what it wrote.
JointTrajectory build(const std::string& attributes)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CubicTrajectory>("CubicTrajectory");
  auto tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main">)"
                                         R"(<CubicTrajectory trajectory="{trajectory}" )" +
                                         attributes + R"(/></BehaviorTree></root>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  return tree.rootBlackboard()->get<JointTrajectory>("trajectory");
}

/// @brief Expect ticking CubicTrajectory with the given attributes to throw.
void expectRejected(const std::string& attributes, BT::Blackboard::Ptr blackboard = BT::Blackboard::create())
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CubicTrajectory>("CubicTrajectory");
  auto tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main">)"
                                         R"(<CubicTrajectory trajectory="{trajectory}" )" +
                                             attributes + R"(/></BehaviorTree></root>)",
                                         blackboard);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}
}  // namespace

TEST(CubicTrajectoryNode, OneWaypointReachedAtRest)
{
  const auto trajectory = build(R"(joint_names="joint1;joint3" positions="-6.28;1.5" duration="2.5")");

  EXPECT_EQ(trajectory.joint_names, (std::vector<std::string>{ "joint1", "joint3" }));
  ASSERT_EQ(trajectory.points.size(), 1u);
  const auto& point = trajectory.points.front();
  EXPECT_EQ(point.positions, (std::vector<double>{ -6.28, 1.5 }));
  EXPECT_EQ(point.velocities, (std::vector<double>{ 0.0, 0.0 }));
  EXPECT_EQ(point.time_from_start.sec, 2);
  EXPECT_EQ(point.time_from_start.nanosec, 500000000u);
}

TEST(CubicTrajectoryNode, TheDurationIsOptional)
{
  const auto trajectory = build(R"(joint_names="joint1" positions="0.0")");
  ASSERT_EQ(trajectory.points.size(), 1u);
  EXPECT_EQ(trajectory.points.front().time_from_start.sec, 5);
  EXPECT_EQ(trajectory.points.front().time_from_start.nanosec, 0u);
}

TEST(CubicTrajectoryNode, OnePositionPerJoint)
{
  expectRejected(R"(joint_names="joint1;joint2" positions="0.0")");
}

TEST(CubicTrajectoryNode, AtLeastOneJoint)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("joints", std::vector<std::string>{});
  blackboard->set("positions", std::vector<double>{});
  expectRejected(R"(joint_names="{joints}" positions="{positions}")", blackboard);
}

TEST(CubicTrajectoryNode, APositiveDuration)
{
  expectRejected(R"(joint_names="joint1" positions="0.0" duration="0.0")");
}

}  // namespace stepit_behaviors::test
