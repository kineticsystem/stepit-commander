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

// End to end test of the ActivateController objective: it loads the XML shipped
// by commander_objectives and runs it against a fake controller manager.

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <commander_behaviors/register_nodes.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fake/fake_controller_manager.hpp"
#include "objective.hpp"

namespace commander_tests
{
namespace
{
constexpr auto kObjective = "ActivateController";

}  // namespace

class ActivateControllerObjective : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("commander_tests_controllers");

    // The robot as StepIt starts it: the trajectory controller drives the
    // motors, the broadcaster only reads them.
    manager_ = std::make_unique<FakeControllerManager>(
        std::vector<FakeControllerManager::Controller>{ { "joint_trajectory_controller", "active", true },
                                                        { "joint_state_broadcaster", "active", false },
                                                        { "velocity_controller", "inactive", true },
                                                        { "position_controller", "inactive", true } });

    BT::RosNodeParams params;
    params.nh = node_;
    params.server_timeout = std::chrono::milliseconds{ 2000 };
    params.wait_for_server_timeout = std::chrono::milliseconds{ 2000 };

    commander_behaviors::registerNodes(factory_, params);
    factory_.registerBehaviorTreeFromFile(treePath("objectives", "ensure_controllers.xml").string());
    factory_.registerBehaviorTreeFromFile(treePath("objectives", "activate_controller.xml").string());
  }

  void TearDown() override
  {
    manager_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<FakeControllerManager> manager_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(ActivateControllerObjective, TheRunningControllerIsStoppedAndTheNewOneStarted)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{controllers: [velocity_controller]}"), BT::NodeStatus::SUCCESS);

  const auto request = manager_->lastSwitch();
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(request->activate_controllers, (std::vector<std::string>{ "velocity_controller" }));
  EXPECT_EQ(request->deactivate_controllers, (std::vector<std::string>{ "joint_trajectory_controller" }));

  EXPECT_EQ(manager_->stateOf("velocity_controller"), "active");
  EXPECT_EQ(manager_->stateOf("joint_trajectory_controller"), "inactive");
}

// A broadcaster owns no command interface: stopping it would silence
// /joint_states and blind every other behavior.
TEST_F(ActivateControllerObjective, TheBroadcasterKeepsRunning)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{controllers: [velocity_controller]}"), BT::NodeStatus::SUCCESS);

  const auto request = manager_->lastSwitch();
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(std::count(request->deactivate_controllers.cbegin(), request->deactivate_controllers.cend(),
                       "joint_state_broadcaster"),
            0);
  EXPECT_EQ(manager_->stateOf("joint_state_broadcaster"), "active");
}

TEST_F(ActivateControllerObjective, ASingleNameNeedsNoList)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{controllers: velocity_controller}"), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(manager_->stateOf("velocity_controller"), "active");
  EXPECT_EQ(manager_->stateOf("joint_trajectory_controller"), "inactive");
}

TEST_F(ActivateControllerObjective, SeveralControllersCanBeActivatedAtOnce)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{controllers: [velocity_controller, position_controller]}"),
            BT::NodeStatus::SUCCESS);

  EXPECT_EQ(manager_->stateOf("velocity_controller"), "active");
  EXPECT_EQ(manager_->stateOf("position_controller"), "active");
  EXPECT_EQ(manager_->stateOf("joint_trajectory_controller"), "inactive");
}

// Asking for the controller that is already running must not stop it.
TEST_F(ActivateControllerObjective, ActivatingTheRunningControllerIsHarmless)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{controllers: [joint_trajectory_controller]}"), BT::NodeStatus::SUCCESS);

  const auto request = manager_->lastSwitch();
  ASSERT_TRUE(request.has_value());
  EXPECT_TRUE(request->deactivate_controllers.empty());
  EXPECT_EQ(manager_->stateOf("joint_trajectory_controller"), "active");
}

TEST_F(ActivateControllerObjective, AnUnknownControllerFailsTheObjective)
{
  EXPECT_EQ(runObjective(factory_, kObjective, "{controllers: [no_such_controller]}"), BT::NodeStatus::FAILURE);

  // The running controller must survive a command that could not be honoured.
  EXPECT_EQ(manager_->stateOf("joint_trajectory_controller"), "active");
}

}  // namespace commander_tests
