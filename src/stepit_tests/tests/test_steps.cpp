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

#include <cctype>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <stepit_behaviors/steps.hpp>

namespace stepit_behaviors::test
{
namespace
{

/// @brief One iteration, as seen by the child: which Steps, its index and its value.
struct Seen
{
  std::string who;
  int index;
  std::vector<double> value;
};

std::vector<Seen> seen;

/// @brief Records the value and index it is given, then succeeds or fails as told.
class Record : public BT::SyncActionNode
{
public:
  Record(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("who"), BT::InputPort<std::vector<double>>("value"),
             BT::InputPort<int>("index"), BT::InputPort<int>("fail_at", -1, "fail at this index") };
  }

  BT::NodeStatus tick() override
  {
    const auto index = getInput<int>("index").value();
    seen.push_back({ getInput<std::string>("who").value_or(""), index, getInput<std::vector<double>>("value").value() });
    return index == getInput<int>("fail_at").value() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

/// @brief Like a motion: RUNNING on its first tick, SUCCESS on the next.
class Move : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<double>>("value"), BT::InputPort<int>("index") };
  }

  BT::NodeStatus onStart() override
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    seen.push_back({ "move", getInput<int>("index").value(), getInput<std::vector<double>>("value").value() });
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
  }
};

BT::Tree makeTree(const std::string& body, BT::Blackboard::Ptr blackboard = BT::Blackboard::create())
{
  seen.clear();
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<Steps>("Steps");
  factory.registerNodeType<Record>("Record");
  factory.registerNodeType<Move>("Move");
  factory.registerBehaviorTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">)" + body +
                                       "</BehaviorTree></root>");
  return factory.createTree("MainTree", blackboard);
}

std::vector<std::vector<double>> values()
{
  std::vector<std::vector<double>> out;
  for (const auto& s : seen)
  {
    out.push_back(s.value);
  }
  return out;
}

std::vector<int> indices()
{
  std::vector<int> out;
  for (const auto& s : seen)
  {
    out.push_back(s.index);
  }
  return out;
}

constexpr auto kRecord = R"(<Record value="{value}" index="{index}"/>)";

}  // namespace

TEST(StepsNode, StepsEvenlyFromStartToEndBothIncluded)
{
  auto tree = makeTree(std::string(R"(<Steps start="0" end="1" count="5" value="{value}" index="{index}">)") + kRecord +
                       "</Steps>");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(values(), (std::vector<std::vector<double>>{ { 0.0 }, { 0.25 }, { 0.5 }, { 0.75 }, { 1.0 } }));
  EXPECT_EQ(indices(), (std::vector<int>{ 0, 1, 2, 3, 4 }));
}

TEST(StepsNode, StepsBackwardsWhenTheEndIsBelowTheStart)
{
  auto tree = makeTree(std::string(R"(<Steps start="2" end="-2" count="3" value="{value}" index="{index}">)") +
                       kRecord + "</Steps>");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(values(), (std::vector<std::vector<double>>{ { 2.0 }, { 0.0 }, { -2.0 } }));
}

TEST(StepsNode, StepsSeveralJointsTogether)
{
  auto tree = makeTree(std::string(R"(<Steps start="0;10" end="1;0" count="3" value="{value}" index="{index}">)") +
                       kRecord + "</Steps>");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(values(), (std::vector<std::vector<double>>{ { 0.0, 10.0 }, { 0.5, 5.0 }, { 1.0, 0.0 } }));
}

TEST(StepsNode, EndsExactlyOnTheEnd)
{
  auto tree = makeTree(std::string(R"(<Steps start="0" end="0.3" count="7" value="{value}" index="{index}">)") +
                       kRecord + "</Steps>");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(seen.size(), 7u);
  EXPECT_EQ(seen.back().value, (std::vector<double>{ 0.3 }));
}

TEST(StepsNode, ACountOfOneGivesTheStart)
{
  auto tree = makeTree(std::string(R"(<Steps start="4" end="8" count="1" value="{value}" index="{index}">)") +
                       kRecord + "</Steps>");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(values(), (std::vector<std::vector<double>>{ { 4.0 } }));
}

// A payload gives numbers as doubles, and a single joint as a single number.
TEST(StepsNode, TakesItsParametersFromThePayload)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("start", 1.0);
  blackboard->set("end", 2.0);
  blackboard->set("count", 3.0);
  auto tree = makeTree(std::string(R"(<Steps start="{@start}" end="{@end}" count="{@count}" value="{value}" index="{index}">)") +
                           kRecord + "</Steps>",
                       blackboard);
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(values(), (std::vector<std::vector<double>>{ { 1.0 }, { 1.5 }, { 2.0 } }));
}

TEST(StepsNode, StepsThroughAListOfValues)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("values", std::vector<double>{ 0.0, 0.1, 0.15, 0.175 });
  auto tree = makeTree(std::string(R"(<Steps values="{@values}" value="{value}" index="{index}">)") + kRecord +
                           "</Steps>",
                       blackboard);
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(values(), (std::vector<std::vector<double>>{ { 0.0 }, { 0.1 }, { 0.15 }, { 0.175 } }));
  EXPECT_EQ(indices(), (std::vector<int>{ 0, 1, 2, 3 }));
}

TEST(StepsNode, StopsAndFailsWhenTheChildFails)
{
  auto tree = makeTree(R"(<Steps start="0" end="4" count="5" value="{value}" index="{index}">
                            <Record value="{value}" index="{index}" fail_at="2"/>
                          </Steps>)");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(indices(), (std::vector<int>{ 0, 1, 2 }));

  // Run again, it starts over.
  seen.clear();
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(indices(), (std::vector<int>{ 0, 1, 2 }));
}

TEST(StepsNode, WaitsForAChildThatIsRunning)
{
  auto tree = makeTree(R"(<Steps start="0" end="1" count="2" value="{value}" index="{index}">
                            <Move value="{value}" index="{index}"/>
                          </Steps>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(seen.empty());
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(values(), (std::vector<std::vector<double>>{ { 0.0 }, { 1.0 } }));
}

TEST(StepsNode, StartsOverAfterBeingHalted)
{
  auto tree = makeTree(R"(<Steps start="0" end="2" count="3" value="{value}" index="{index}">
                            <Move value="{value}" index="{index}"/>
                          </Steps>)");
  tree.tickOnce();
  tree.tickOnce();  // The first move ends, the second starts.
  ASSERT_EQ(indices(), (std::vector<int>{ 0 }));
  tree.haltTree();

  seen.clear();
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(indices(), (std::vector<int>{ 0, 1, 2 }));
}

// Nested: the inner Steps runs through all of its values at every value of
// the outer one, and goes back to its first value every time.
TEST(StepsNode, NestedStepsMakeAGrid)
{
  auto tree = makeTree(R"(
    <Steps start="0" end="1" count="2" value="{outer}" index="{i}">
      <Sequence>
        <Record who="outer" value="{outer}" index="{i}"/>
        <Steps start="10" end="30" count="3" value="{inner}" index="{j}">
          <Record who="inner" value="{inner}" index="{j}"/>
        </Steps>
      </Sequence>
    </Steps>)");
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);

  std::vector<std::string> steps;
  for (const auto& s : seen)
  {
    steps.push_back(s.who + " " + std::to_string(static_cast<int>(s.value[0])));
  }
  EXPECT_EQ(steps, (std::vector<std::string>{ "outer 0", "inner 10", "inner 20", "inner 30",  //
                                              "outer 1", "inner 10", "inner 20", "inner 30" }));
}

class StepsRejects : public ::testing::TestWithParam<std::pair<std::string, std::string>>
{
};

TEST_P(StepsRejects, BadParameters)
{
  auto tree = makeTree(std::string("<Steps ") + GetParam().second + R"( value="{value}" index="{index}">)" + kRecord +
                       "</Steps>");
  EXPECT_THROW(tree.tickWhileRunning(), BT::RuntimeError) << GetParam().first;
  EXPECT_TRUE(seen.empty());
}

INSTANTIATE_TEST_SUITE_P(
    StepsNode, StepsRejects,
    ::testing::Values(std::make_pair("no count", R"(start="0" end="1")"),
                      std::make_pair("a count of zero", R"(start="0" end="1" count="0")"),
                      std::make_pair("a fractional count", R"(start="0" end="1" count="2.5")"),
                      std::make_pair("no end", R"(start="0" count="2")"),
                      std::make_pair("lengths that differ", R"(start="0;1" end="1" count="2")"),
                      std::make_pair("values and a range", R"(values="1;2" start="0" end="1" count="2")"),
                      std::make_pair("a missing payload entry", R"(values="{@nope}")")),
    [](const auto& test) {
      std::string name = test.param.first;
      for (auto& c : name)
      {
        if (!std::isalnum(static_cast<unsigned char>(c)))
        {
          c = '_';
        }
      }
      return name;
    });

}  // namespace stepit_behaviors::test
