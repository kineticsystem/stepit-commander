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

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <stepit_behaviors/offset_joint_positions.hpp>

namespace stepit_behaviors::test
{
namespace
{
constexpr auto kTree = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <OffsetJointPositions offset="{@offset}"
                          current_positions="{@current_positions}"
                          target_positions="{target_positions}"/>
  </BehaviorTree>
</root>)";

/// @brief Tick the node once, with the given command parameters on the blackboard.
BT::Tree makeTree(BT::Blackboard::Ptr blackboard)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<OffsetJointPositions>("OffsetJointPositions");
  factory.registerBehaviorTreeFromText(kTree);
  return factory.createTree("MainTree", BT::Blackboard::create(blackboard));
}

std::vector<double> offsetBy(double offset, const std::vector<double>& current_positions)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("offset", offset);
  blackboard->set("current_positions", current_positions);

  auto tree = makeTree(blackboard);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  return tree.rootBlackboard()->get<std::vector<double>>("target_positions");
}
}  // namespace

// On the StepIt motors a clockwise rotation decreases the joint position, so a
// clockwise command is simply a negative offset.
TEST(OffsetJointPositionsNode, ANegativeOffsetDecreasesEveryJointPosition)
{
  const auto targets = offsetBy(-6.28, { 0.0, 1.0 });
  ASSERT_EQ(targets.size(), 2u);
  EXPECT_DOUBLE_EQ(targets[0], -6.28);
  EXPECT_DOUBLE_EQ(targets[1], -5.28);
}

TEST(OffsetJointPositionsNode, APositiveOffsetIncreasesEveryJointPosition)
{
  const auto targets = offsetBy(0.5, { 0.0, 1.0, -2.0 });
  ASSERT_EQ(targets.size(), 3u);
  EXPECT_DOUBLE_EQ(targets[0], 0.5);
  EXPECT_DOUBLE_EQ(targets[1], 1.5);
  EXPECT_DOUBLE_EQ(targets[2], -1.5);
}

TEST(OffsetJointPositionsNode, AZeroOffsetLeavesTheJointsWhereTheyAre)
{
  EXPECT_EQ(offsetBy(0.0, { 0.5, -1.5 }), (std::vector<double>{ 0.5, -1.5 }));
}

TEST(OffsetJointPositionsNode, NoJointToMove)
{
  EXPECT_TRUE(offsetBy(1.0, {}).empty());
}

TEST(OffsetJointPositionsNode, AMissingOffsetIsRejected)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("current_positions", std::vector<double>{ 0.0 });

  auto tree = makeTree(blackboard);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

TEST(OffsetJointPositionsNode, MissingCurrentPositionsAreRejected)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("offset", 1.0);

  auto tree = makeTree(blackboard);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

}  // namespace stepit_behaviors::test
