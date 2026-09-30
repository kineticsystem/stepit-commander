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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <stepit_behaviors/register_nodes.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fake/fake_controller_manager.hpp"
#include "fake/fake_robot.hpp"

namespace stepit_tests
{
namespace
{
constexpr auto kJointStateTopic = "/command_joint_positions/joint_states";
constexpr auto kActionName = "/command_joint_positions/follow_joint_trajectory";
constexpr auto kCommandTopic = "/command_joint_positions/commands";

const std::vector<std::string> kJointNames{ "joint1", "joint2", "joint3" };
const std::vector<double> kJointPositions{ 0.5, 1.0, -1.0 };

/// @brief A tree made of CommandJointPositions alone, on the fake robot, with the given attributes.
std::string treeXml(const std::string& attributes)
{
  return std::string(R"(<root BTCPP_format="4"><BehaviorTree ID="Command"><CommandJointPositions )") +
         R"(topic_name=")" + kCommandTopic + R"(" joint_states_topic=")" + kJointStateTopic +
         R"(" controller_joints="joint1;joint2;joint3" )" + attributes + R"(/></BehaviorTree></root>)";
}

}  // namespace

class CommandJointPositionsTest : public testing::Test
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
    manager_ = std::make_unique<FakeControllerManager>(std::vector<FakeControllerManager::Controller>{
        { "position_controller", "active", true }, { "joint_state_broadcaster", "active", false } });
    stepit_behaviors::registerNodes(factory_, BT::RosNodeParams{ node_ });
  }

  void TearDown() override
  {
    manager_.reset();
    robot_.reset();
    node_.reset();
  }

  /// @brief Tick the tree until it is done, or `limit` has passed.
  BT::NodeStatus run(BT::Tree& tree, std::chrono::milliseconds limit = std::chrono::seconds{ 10 })
  {
    const auto start = std::chrono::steady_clock::now();
    auto status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() - start < limit)
    {
      status = tree.tickExactlyOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });
    }
    return status;
  }

  BT::NodeStatus run(const std::string& attributes)
  {
    auto tree = factory_.createTreeFromText(treeXml(attributes));
    return run(tree);
  }

  rclcpp::Node::SharedPtr node_;
  /// @brief Wait until the position controller is inactive, as a stop leaves it. Returns whether it is.
  bool waitUntilDeactivated()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{ 2 };
    while (manager_->stateOf("position_controller") != "inactive" && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });
    }
    return manager_->stateOf("position_controller") == "inactive";
  }

  std::unique_ptr<FakeRobot> robot_;
  std::unique_ptr<FakeControllerManager> manager_;
  BT::BehaviorTreeFactory factory_;
};

// The controller takes every joint, so the others are sent where they are.
TEST_F(CommandJointPositionsTest, MovesOneJointAndHoldsTheOthers)
{
  ASSERT_EQ(run(R"(joint_names="joint2" positions="2.0")"), BT::NodeStatus::SUCCESS);

  const auto commands = robot_->positionCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front(), (std::vector<double>{ 0.5, 2.0, -1.0 }));
  const auto positions = robot_->positions();
  EXPECT_NEAR(positions[1], 2.0, 0.01);
  EXPECT_DOUBLE_EQ(positions[0], 0.5);
  EXPECT_DOUBLE_EQ(positions[2], -1.0);
}

TEST_F(CommandJointPositionsTest, TakesOnePositionPerJoint)
{
  ASSERT_EQ(run(R"(joint_names="joint3;joint1" positions="3.0;-2.0")"), BT::NodeStatus::SUCCESS);

  const auto commands = robot_->positionCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front(), (std::vector<double>{ -2.0, 1.0, 3.0 }));
}

TEST_F(CommandJointPositionsTest, TakesOnePositionForEveryJoint)
{
  ASSERT_EQ(run(R"(joint_names="joint1;joint3" positions="0.0")"), BT::NodeStatus::SUCCESS);

  const auto commands = robot_->positionCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front(), (std::vector<double>{ 0.0, 1.0, 0.0 }));
}

// A joint that never gets there fails the node, which stops the robot: it
// deactivates the position controller, for the hardware to brake the joints to
// rest, and sends no position.
TEST_F(CommandJointPositionsTest, FailsAfterTheTimeoutAndStops)
{
  robot_->freeze();

  ASSERT_EQ(run(R"(joint_names="joint2" positions="2.0" timeout="0.5")"), BT::NodeStatus::FAILURE);

  EXPECT_TRUE(waitUntilDeactivated());
  const auto request = manager_->lastSwitch();
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(request->deactivate_controllers, (std::vector<std::string>{ "position_controller" }));
  EXPECT_TRUE(request->activate_controllers.empty());
  EXPECT_EQ(robot_->positionCommands().size(), 1u);
}

// Halting it, e.g. cancelling the objective, stops the robot the same way.
// Sending the joints where they are would not do: a joint moving at speed
// would brake past it and come back.
TEST_F(CommandJointPositionsTest, StopsTheRobotWhenHalted)
{
  robot_->followPositionCommands(kCommandTopic, 1.0);
  auto tree = factory_.createTreeFromText(treeXml(R"(joint_names="joint2" positions="11.0")"));
  ASSERT_EQ(run(tree, std::chrono::milliseconds{ 500 }), BT::NodeStatus::RUNNING);

  tree.haltTree();

  EXPECT_TRUE(waitUntilDeactivated());
  const auto request = manager_->lastSwitch();
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(request->deactivate_controllers, (std::vector<std::string>{ "position_controller" }));
  EXPECT_EQ(robot_->positionCommands().size(), 1u);
}

TEST_F(CommandJointPositionsTest, RefusesAJointTheControllerDoesNotHave)
{
  auto tree = factory_.createTreeFromText(treeXml(R"(joint_names="joint9" positions="1.0")"));
  EXPECT_THROW(tree.tickExactlyOnce(), BT::RuntimeError);
  EXPECT_TRUE(robot_->positionCommands().empty());
}

TEST_F(CommandJointPositionsTest, RefusesAsManyPositionsAsJointsButOne)
{
  auto tree = factory_.createTreeFromText(treeXml(R"(joint_names="joint1;joint2;joint3" positions="1.0;2.0")"));
  EXPECT_THROW(tree.tickExactlyOnce(), BT::RuntimeError);
  EXPECT_TRUE(robot_->positionCommands().empty());
}

}  // namespace stepit_tests
