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

#include "fake/fake_robot.hpp"

namespace stepit_tests
{
namespace
{
constexpr auto kJointStateTopic = "/get_joint_positions/joint_states";
constexpr auto kActionName = "/get_joint_positions/follow_joint_trajectory";

const std::vector<std::string> kJointNames{ "joint1", "joint2", "joint3" };
const std::vector<double> kJointPositions{ 0.5, 1.0, -1.0 };

/// @brief A tree made of GetJointPositions alone, with the given attributes.
std::string treeXml(const std::string& attributes)
{
  return R"(<root BTCPP_format="4"><BehaviorTree ID="Read">)"
         R"(<GetJointPositions positions="{positions}" )" +
         attributes + R"(/></BehaviorTree></root>)";
}

}  // namespace

class GetJointPositionsTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("stepit_tests");
    stepit_behaviors::registerNodes(factory_, BT::RosNodeParams{ node_ });
  }

  /// @brief Tick the tree until it is done, and measure how long it took.
  BT::NodeStatus run(const std::string& attributes)
  {
    auto tree = factory_.createTreeFromText(treeXml(attributes));
    const auto start = std::chrono::steady_clock::now();
    auto status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() - start < std::chrono::seconds{ 10 })
    {
      status = tree.tickExactlyOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds{ 10 });
    }
    elapsed_ = std::chrono::steady_clock::now() - start;
    if (!tree.rootBlackboard()->get("positions", positions_))
    {
      positions_.clear();
    }
    return status;
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
  std::chrono::steady_clock::duration elapsed_{};
  std::vector<double> positions_;
};

// The first tick comes right after the tree is created, before any joint state
// can have arrived: the node waits for one, with no retry around it.
TEST_F(GetJointPositionsTest, WaitsForTheFirstJointState)
{
  FakeRobot robot{ kJointStateTopic, kActionName, kJointNames, kJointPositions };

  ASSERT_EQ(run(std::string{ "topic_name=\"" } + kJointStateTopic + "\" joint_names=\"joint3;joint1\""),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(positions_, (std::vector<double>{ -1.0, 0.5 }));
}

TEST_F(GetJointPositionsTest, FailsWhenNoJointStateArrivesInTime)
{
  EXPECT_EQ(run(R"(topic_name="/get_joint_positions/silent" joint_names="joint1" timeout="0.3")"),
            BT::NodeStatus::FAILURE);
  EXPECT_GE(elapsed_, std::chrono::milliseconds{ 300 });
  EXPECT_LT(elapsed_, std::chrono::seconds{ 2 });
}

// A joint that is not published is a mistake in the tree: waiting would not fix it.
TEST_F(GetJointPositionsTest, FailsAtOnceForAnUnknownJoint)
{
  FakeRobot robot{ kJointStateTopic, kActionName, kJointNames, kJointPositions };

  EXPECT_EQ(run(std::string{ "topic_name=\"" } + kJointStateTopic + "\" joint_names=\"joint9\" timeout=\"5.0\""),
            BT::NodeStatus::FAILURE);
  EXPECT_LT(elapsed_, std::chrono::seconds{ 2 });
}

}  // namespace stepit_tests
