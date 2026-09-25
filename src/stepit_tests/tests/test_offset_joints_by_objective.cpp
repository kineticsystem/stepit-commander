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

// End to end test of the OffsetJointsBy objective: it loads the XML shipped by
// stepit_objectives, registers the real behaviors, and runs the tree against
// a fake robot. Only the action server itself (which is provided by
// BehaviorTree.ROS2) is left out.

#include <chrono>
#include <cmath>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <stepit_behaviors/register_nodes.hpp>
#include <stepit_behaviors/trapezoidal_trajectory.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fake/fake_controller_manager.hpp"
#include "fake/fake_robot.hpp"
#include "objective.hpp"

namespace stepit_tests
{
namespace
{
constexpr auto kJointStateTopic = "/joint_states";
constexpr auto kActionName = "/joint_trajectory_controller/follow_joint_trajectory";
constexpr auto kObjective = "OffsetJointsBy";

/// @brief The joints of the StepIt robot, and where they start from.
const std::vector<std::string> kJointNames{ "joint1", "joint2", "joint3", "joint4", "joint5" };
const std::vector<double> kJointPositions{ 0.5, 1.0, 0.0, -1.0, 2.0 };

double seconds(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return rclcpp::Duration(point.time_from_start).seconds();
}

}  // namespace

class OffsetJointsByObjective : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("stepit_tests");
    robot_ = std::make_unique<FakeRobot>(kJointStateTopic, kActionName, kJointNames, kJointPositions);

    // The objective makes sure the trajectory controller is the one driving the
    // robot before it moves, so the controller manager has to answer too.
    manager_ = std::make_unique<FakeControllerManager>(
        std::vector<FakeControllerManager::Controller>{ { "joint_trajectory_controller", "inactive", true },
                                                        { "joint_state_broadcaster", "active", false },
                                                        { "velocity_controller", "active", true } });

    BT::RosNodeParams params;
    params.nh = node_;
    params.server_timeout = std::chrono::milliseconds{ 2000 };
    params.wait_for_server_timeout = std::chrono::milliseconds{ 2000 };

    stepit_behaviors::registerNodes(factory_, params);
    factory_.registerBehaviorTreeFromFile(treePath("objectives", "ensure_controllers.xml").string());
    factory_.registerBehaviorTreeFromFile(treePath("objectives", "offset_joints_by.xml").string());
  }

  void TearDown() override
  {
    manager_.reset();
    robot_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<FakeRobot> robot_;
  std::unique_ptr<FakeControllerManager> manager_;
  BT::BehaviorTreeFactory factory_;
};

// One turn is too short to reach the top speed at the motors' acceleration
// limit: the motion is a triangle, accelerate then brake.
TEST_F(OffsetJointsByObjective, ANegativeOffsetOnTwoJoints)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{joints: [joint1, joint2], offset: -6.28}"), BT::NodeStatus::SUCCESS);

  const auto trajectory = robot_->lastTrajectory();
  ASSERT_TRUE(trajectory.has_value());
  EXPECT_EQ(trajectory->joint_names, (std::vector<std::string>{ "joint1", "joint2" }));

  ASSERT_EQ(trajectory->points.size(), 2u);
  const auto& arrival = trajectory->points.back();
  ASSERT_EQ(arrival.positions.size(), 2u);
  // A negative offset decreases the joint position, i.e. turns clockwise.
  EXPECT_NEAR(arrival.positions[0], 0.5 - 6.28, 1e-9);
  EXPECT_NEAR(arrival.positions[1], 1.0 - 6.28, 1e-9);
  EXPECT_EQ(arrival.velocities, (std::vector<double>{ 0.0, 0.0 }));
  EXPECT_NEAR(seconds(arrival), 2.0 * std::sqrt(6.28 / stepit_behaviors::TrapezoidalTrajectory::kMaxAcceleration), 1e-6);
}

TEST_F(OffsetJointsByObjective, APositiveOffsetOnOneJoint)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{joints: [joint4], offset: 1.57}"), BT::NodeStatus::SUCCESS);

  const auto trajectory = robot_->lastTrajectory();
  ASSERT_TRUE(trajectory.has_value());
  EXPECT_EQ(trajectory->joint_names, (std::vector<std::string>{ "joint4" }));
  ASSERT_FALSE(trajectory->points.empty());
  ASSERT_EQ(trajectory->points.back().positions.size(), 1u);
  EXPECT_NEAR(trajectory->points.back().positions[0], -1.0 + 1.57, 1e-9);
}

// Lower limits in the payload slow the motion down: 3 rad at 1 rad/s and
// 1 rad/s² is 1 s accelerating over 0.5 rad, 2 s cruising over 2 rad, and 1 s
// braking over the last 0.5 rad.
TEST_F(OffsetJointsByObjective, ThePayloadCanLowerTheLimits)
{
  ASSERT_EQ(runObjective(factory_, kObjective,
                         "{joints: [joint1], offset: 3.0, max_velocity: 1.0, max_acceleration: 1.0}"),
            BT::NodeStatus::SUCCESS);

  const auto trajectory = robot_->lastTrajectory();
  ASSERT_TRUE(trajectory.has_value());
  ASSERT_EQ(trajectory->points.size(), 3u);
  EXPECT_NEAR(trajectory->points[0].velocities[0], 1.0, 1e-9);
  EXPECT_NEAR(trajectory->points.back().positions[0], 0.5 + 3.0, 1e-9);
  EXPECT_NEAR(seconds(trajectory->points.back()), 4.0, 1e-6);
}

TEST_F(OffsetJointsByObjective, OneOffsetPerJoint)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{joints: [joint1, joint3], offset: [-6.28, 3.14]}"),
            BT::NodeStatus::SUCCESS);

  const auto trajectory = robot_->lastTrajectory();
  ASSERT_TRUE(trajectory.has_value());
  EXPECT_EQ(trajectory->joint_names, (std::vector<std::string>{ "joint1", "joint3" }));
  ASSERT_FALSE(trajectory->points.empty());
  const auto& arrival = trajectory->points.back();
  ASSERT_EQ(arrival.positions.size(), 2u);
  EXPECT_NEAR(arrival.positions[0], 0.5 - 6.28, 1e-9);
  EXPECT_NEAR(arrival.positions[1], 0.0 + 3.14, 1e-9);
  // Joint 3 has half the way, so it moves at half the speed, the other way.
  EXPECT_NEAR(trajectory->points.front().velocities[1], -0.5 * trajectory->points.front().velocities[0], 1e-9);
}

// The server aborts the goal with the message of the exception, which says
// what is wrong with the command.
TEST_F(OffsetJointsByObjective, AWrongNumberOfOffsetsAbortsTheObjective)
{
  EXPECT_THROW(runObjective(factory_, kObjective, "{joints: [joint1, joint3], offset: [-6.28, 3.14, 1.0]}"),
               BT::RuntimeError);
  EXPECT_FALSE(robot_->lastTrajectory().has_value());
}

// The objective needs the trajectory controller: it activates it, and stops
// whatever else was driving the robot.
TEST_F(OffsetJointsByObjective, TheTrajectoryControllerIsActivatedBeforeMoving)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{joints: [joint1], offset: -1.0}"), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(manager_->stateOf("joint_trajectory_controller"), "active");
  EXPECT_EQ(manager_->stateOf("velocity_controller"), "inactive");
  EXPECT_EQ(manager_->stateOf("joint_state_broadcaster"), "active");
}

TEST_F(OffsetJointsByObjective, AnUnknownJointFailsTheObjective)
{
  EXPECT_EQ(runObjective(factory_, kObjective, "{joints: [joint9], offset: -1.0}"), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(robot_->lastTrajectory().has_value());
}

TEST_F(OffsetJointsByObjective, AControllerErrorFailsTheObjective)
{
  robot_->failNextTrajectory();

  EXPECT_EQ(runObjective(factory_, kObjective, "{joints: [joint1], offset: -1.0}"), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(robot_->lastTrajectory().has_value());
}

}  // namespace stepit_tests
