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
#include <stepit_behaviors/offset_vector.hpp>

namespace stepit_behaviors::test
{
namespace
{
constexpr auto kTree = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <OffsetVector input="{@input}" offset="{@offset}" output="{output}"/>
  </BehaviorTree>
</root>)";

/// @brief Tick the node once, with the given command parameters on the blackboard.
BT::Tree makeTree(BT::Blackboard::Ptr blackboard)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<OffsetVector>("OffsetVector");
  factory.registerBehaviorTreeFromText(kTree);
  return factory.createTree("MainTree", BT::Blackboard::create(blackboard));
}

/// @brief The output, for an offset that is a number or a list of numbers.
template <typename Offset>
std::vector<double> offsetBy(const Offset& offset, const std::vector<double>& input)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("offset", offset);
  blackboard->set("input", input);

  auto tree = makeTree(blackboard);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  return tree.rootBlackboard()->get<std::vector<double>>("output");
}
}  // namespace

TEST(OffsetVectorNode, ANegativeOffsetDecreasesEveryElement)
{
  const auto output = offsetBy(-6.28, { 0.0, 1.0 });
  ASSERT_EQ(output.size(), 2u);
  EXPECT_DOUBLE_EQ(output[0], -6.28);
  EXPECT_DOUBLE_EQ(output[1], -5.28);
}

TEST(OffsetVectorNode, APositiveOffsetIncreasesEveryElement)
{
  const auto output = offsetBy(0.5, { 0.0, 1.0, -2.0 });
  ASSERT_EQ(output.size(), 3u);
  EXPECT_DOUBLE_EQ(output[0], 0.5);
  EXPECT_DOUBLE_EQ(output[1], 1.5);
  EXPECT_DOUBLE_EQ(output[2], -1.5);
}

TEST(OffsetVectorNode, AZeroOffsetLeavesTheElementsAsTheyAre)
{
  EXPECT_EQ(offsetBy(0.0, { 0.5, -1.5 }), (std::vector<double>{ 0.5, -1.5 }));
}

TEST(OffsetVectorNode, AnEmptyVector)
{
  EXPECT_TRUE(offsetBy(1.0, {}).empty());
}

TEST(OffsetVectorNode, AListGivesEachElementItsOwnOffset)
{
  const auto output = offsetBy(std::vector<double>{ -6.28, 3.14 }, { 0.0, 1.0 });
  ASSERT_EQ(output.size(), 2u);
  EXPECT_DOUBLE_EQ(output[0], -6.28);
  EXPECT_DOUBLE_EQ(output[1], 4.14);
}

TEST(OffsetVectorNode, AListOfOneOffsetIsAddedToEveryElement)
{
  EXPECT_EQ(offsetBy(std::vector<double>{ 1.0 }, { 0.0, 1.0 }), (std::vector<double>{ 1.0, 2.0 }));
}

TEST(OffsetVectorNode, AListOfTheWrongLengthIsRejected)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("offset", std::vector<double>{ 1.0, 2.0 });
  blackboard->set("input", std::vector<double>{ 0.0, 0.0, 0.0 });

  auto tree = makeTree(blackboard);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

// In the XML, a list of offsets is written with ';' between them.
TEST(OffsetVectorNode, AListWrittenInTheXml)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<OffsetVector>("OffsetVector");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("input", std::vector<double>{ 0.0, 1.0 });
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <OffsetVector input="{input}" offset="-1.5;2.5" output="{output}"/>
      </BehaviorTree>
    </root>)",
                                         blackboard);

  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard->get<std::vector<double>>("output"), (std::vector<double>{ -1.5, 3.5 }));
}

TEST(OffsetVectorNode, AMissingOffsetIsRejected)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("input", std::vector<double>{ 0.0 });

  auto tree = makeTree(blackboard);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

TEST(OffsetVectorNode, AMissingInputIsRejected)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("offset", 1.0);

  auto tree = makeTree(blackboard);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

}  // namespace stepit_behaviors::test
