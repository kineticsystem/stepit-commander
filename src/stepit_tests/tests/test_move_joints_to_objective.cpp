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

// End to end test of the MoveJointsTo objective: it loads the XML shipped by
// stepit_objectives and runs it against a fake robot.

#include <chrono>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <stepit_behaviors/register_nodes.hpp>
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
constexpr auto kObjective = "MoveJointsTo";

const std::vector<std::string> kJointNames{ "joint1", "joint2", "joint3", "joint4", "joint5" };
const std::vector<double> kJointPositions{ 0.5, 1.0, 0.0, -1.0, 2.0 };
}  // namespace

class MoveJointsToObjective : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("stepit_tests_absolute");
    robot_ = std::make_unique<FakeRobot>(kJointStateTopic, kActionName, kJointNames, kJointPositions);
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
    factory_.registerBehaviorTreeFromFile(treePath("objectives", "move_joints_to.xml").string());
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

// The positions are the targets: where the joints start from does not matter.
TEST_F(MoveJointsToObjective, TheJointsAreSentToTheGivenPositions)
{
  ASSERT_EQ(runObjective(factory_, kObjective,
                         "{joints: [joint1, joint2], positions: [0.0, 1.57], "
                         "duration: 3.0}"),
            BT::NodeStatus::SUCCESS);

  const auto trajectory = robot_->lastTrajectory();
  ASSERT_TRUE(trajectory.has_value());
  EXPECT_EQ(trajectory->joint_names, (std::vector<std::string>{ "joint1", "joint2" }));

  ASSERT_EQ(trajectory->points.size(), 1u);
  const auto& point = trajectory->points.front();
  EXPECT_EQ(point.positions, (std::vector<double>{ 0.0, 1.57 }));
  EXPECT_EQ(point.velocities, (std::vector<double>{ 0.0, 0.0 }));
  EXPECT_EQ(point.time_from_start.sec, 3);
}

TEST_F(MoveJointsToObjective, NegativePositionsAreTargetsLikeAnyOther)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{joints: [joint4], positions: [-2.5]}"), BT::NodeStatus::SUCCESS);

  const auto trajectory = robot_->lastTrajectory();
  ASSERT_TRUE(trajectory.has_value());
  ASSERT_EQ(trajectory->points.size(), 1u);
  EXPECT_EQ(trajectory->points.front().positions, (std::vector<double>{ -2.5 }));
  // The duration is optional and falls back to its default.
  EXPECT_EQ(trajectory->points.front().time_from_start.sec, 5);
}

TEST_F(MoveJointsToObjective, TheTrajectoryControllerIsActivatedBeforeMoving)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{joints: [joint1], positions: [0.0]}"), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(manager_->stateOf("joint_trajectory_controller"), "active");
  EXPECT_EQ(manager_->stateOf("velocity_controller"), "inactive");
}

// A position missing for one of the joints would move the robot somewhere
// nobody asked for: the objective must refuse it.
TEST_F(MoveJointsToObjective, AsManyPositionsAsJointsAreRequired)
{
  EXPECT_ANY_THROW(runObjective(factory_, kObjective, "{joints: [joint1, joint2], positions: [0.0]}"));
  EXPECT_FALSE(robot_->lastTrajectory().has_value());
}

}  // namespace stepit_tests
