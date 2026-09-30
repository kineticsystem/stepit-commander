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

// End to end tests of MoveJointsDirectlyTo and OffsetJointsDirectlyBy: they
// load the XML shipped by stepit_objectives, register the real behaviors, and
// run the trees against a fake robot that follows the commands of a position
// controller.

#include <memory>
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
constexpr auto kCommandTopic = "/position_controller/commands";

/// @brief The joints of the StepIt robot, and where they start from.
const std::vector<std::string> kJointNames{ "joint1", "joint2", "joint3", "joint4", "joint5" };
const std::vector<double> kJointPositions{ 0.5, 1.0, 0.0, -1.0, 2.0 };

}  // namespace

class DirectObjectives : public testing::Test
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
    robot_->followPositionCommands(kCommandTopic, 10.0);

    // The objectives make sure the position controller is the one driving the
    // robot before they move it.
    manager_ = std::make_unique<FakeControllerManager>(
        std::vector<FakeControllerManager::Controller>{ { "joint_trajectory_controller", "active", true },
                                                        { "joint_state_broadcaster", "active", false },
                                                        { "position_controller", "inactive", true } });

    BT::RosNodeParams params;
    params.nh = node_;
    params.server_timeout = std::chrono::milliseconds{ 2000 };
    params.wait_for_server_timeout = std::chrono::milliseconds{ 2000 };

    stepit_behaviors::registerNodes(factory_, params);
    factory_.registerBehaviorTreeFromFile(treePath("objectives", "ensure_controllers.xml").string());
    factory_.registerBehaviorTreeFromFile(treePath("objectives", "move_joints_directly_to.xml").string());
    factory_.registerBehaviorTreeFromFile(treePath("objectives", "offset_joints_directly_by.xml").string());
  }

  void TearDown() override
  {
    manager_.reset();
    robot_.reset();
    node_.reset();
  }

  /// @brief Whether the position controller was switched on, and the trajectory controller off.
  void expectPositionController()
  {
    const auto request = manager_->lastSwitch();
    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(request->activate_controllers, (std::vector<std::string>{ "position_controller" }));
    EXPECT_EQ(manager_->stateOf("position_controller"), "active");
    EXPECT_EQ(manager_->stateOf("joint_trajectory_controller"), "inactive");
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<FakeRobot> robot_;
  std::unique_ptr<FakeControllerManager> manager_;
  BT::BehaviorTreeFactory factory_;
};

// Every joint of the controller is sent: the others where they are.
TEST_F(DirectObjectives, MovesJointsToAbsolutePositions)
{
  ASSERT_EQ(runObjective(factory_, "MoveJointsDirectlyTo", "{joints: [joint2, joint4], positions: [1.57, 0.5]}"),
            BT::NodeStatus::SUCCESS);

  expectPositionController();
  const auto commands = robot_->positionCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front(), (std::vector<double>{ 0.5, 1.57, 0.0, 0.5, 2.0 }));
  EXPECT_NEAR(robot_->positions()[1], 1.57, 0.01);
  EXPECT_NEAR(robot_->positions()[3], 0.5, 0.01);
}

TEST_F(DirectObjectives, MovesOneJointGivenWithoutBrackets)
{
  ASSERT_EQ(runObjective(factory_, "MoveJointsDirectlyTo", "{joints: joint3, positions: 1.0}"), BT::NodeStatus::SUCCESS);

  const auto commands = robot_->positionCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front(), (std::vector<double>{ 0.5, 1.0, 1.0, -1.0, 2.0 }));
}

// A negative offset decreases the joint position, i.e. turns clockwise.
TEST_F(DirectObjectives, OffsetsJointsFromWhereTheyAre)
{
  ASSERT_EQ(runObjective(factory_, "OffsetJointsDirectlyBy", "{joints: [joint1, joint2], offset: -6.28}"),
            BT::NodeStatus::SUCCESS);

  expectPositionController();
  const auto commands = robot_->positionCommands();
  ASSERT_EQ(commands.size(), 1u);
  const std::vector<double> expected{ 0.5 - 6.28, 1.0 - 6.28, 0.0, -1.0, 2.0 };
  ASSERT_EQ(commands.front().size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i)
  {
    EXPECT_NEAR(commands.front()[i], expected[i], 1e-9) << "joint" << i + 1;
  }
}

TEST_F(DirectObjectives, OffsetsEachJointByItsOwnAmount)
{
  ASSERT_EQ(runObjective(factory_, "OffsetJointsDirectlyBy", "{joints: [joint5, joint1], offset: [1.0, -0.5]}"),
            BT::NodeStatus::SUCCESS);

  const auto commands = robot_->positionCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_NEAR(commands.front()[4], 3.0, 1e-9);
  EXPECT_NEAR(commands.front()[0], 0.0, 1e-9);
}

// A joint the robot does not have fails the objective, and nothing moves.
TEST_F(DirectObjectives, FailsForAnUnknownJoint)
{
  EXPECT_EQ(runObjective(factory_, "OffsetJointsDirectlyBy", "{joints: [joint9], offset: 1.0}"),
            BT::NodeStatus::FAILURE);
  EXPECT_THROW(runObjective(factory_, "MoveJointsDirectlyTo", "{joints: [joint9], positions: 1.0}"), BT::RuntimeError);
  EXPECT_TRUE(robot_->positionCommands().empty());
}

}  // namespace stepit_tests
