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

#include <chrono>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/contrib/json.hpp>
#include <stepit_server/execution_status.hpp>

namespace stepit_server::test
{
namespace
{
using namespace std::chrono_literals;

// A Sequence of an action that runs for one tick, then a subtree. Its uids, in
// creation order: 1 Sequence, 2 Wait, 3 SubTree, 4 Fallback, 5 AlwaysFailure,
// 6 AlwaysSuccess.
constexpr auto kTrees = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Sequence>
      <Wait/>
      <SubTree ID="Inner"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Inner">
    <Fallback>
      <AlwaysFailure/>
      <AlwaysSuccess/>
    </Fallback>
  </BehaviorTree>
</root>)";

/// @brief Like a motion: RUNNING on its first tick, SUCCESS on the next.
class Wait : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
  }
};

class ExecutionStatusTest : public testing::Test
{
protected:
  void SetUp() override
  {
    factory_.registerNodeType<Wait>("Wait");
    factory_.registerBehaviorTreeFromText(kTrees);
    tree_ = factory_.createTree("Main");
  }

  static nlohmann::json parse(const std::optional<std::string>& message)
  {
    EXPECT_TRUE(message.has_value());
    return message ? nlohmann::json::parse(*message) : nlohmann::json{};
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  ExecutionStatus::Clock::time_point start_ = ExecutionStatus::Clock::now();
};

}  // namespace

TEST_F(ExecutionStatusTest, TheFirstMessageCarriesTheTreeWithItsUids)
{
  ExecutionStatus status(tree_);
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  const auto message = parse(status.feedback(false, start_));
  const auto xml = message.at("tree").get<std::string>();
  EXPECT_NE(xml.find(R"(<BehaviorTree ID="Inner")"), std::string::npos) << xml;
  EXPECT_NE(xml.find(R"(_uid="5")"), std::string::npos) << xml;
  EXPECT_EQ(message.at("nodes"), (nlohmann::json{ { "1", "RUNNING" }, { "2", "RUNNING" } }));
}

TEST_F(ExecutionStatusTest, LaterMessagesCarryOnlyTheChanges)
{
  ExecutionStatus status(tree_);
  tree_.tickExactlyOnce();
  status.feedback(false, start_);

  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  const auto message = parse(status.feedback(true, start_ + 10ms));
  EXPECT_FALSE(message.contains("tree"));
  // Every node ended back in IDLE, reset by its parent: each keeps how it ended.
  EXPECT_EQ(message.at("nodes"), (nlohmann::json{ { "1", "SUCCESS" },
                                                  { "2", "SUCCESS" },
                                                  { "3", "SUCCESS" },
                                                  { "4", "SUCCESS" },
                                                  { "5", "FAILURE" },
                                                  { "6", "SUCCESS" } }));
}

TEST_F(ExecutionStatusTest, NothingIsSentWithoutChanges)
{
  ExecutionStatus status(tree_);
  tree_.tickExactlyOnce();
  status.feedback(false, start_);

  EXPECT_FALSE(status.feedback(false, start_ + 1s).has_value());
  EXPECT_FALSE(status.feedback(true, start_ + 1s).has_value());
}

// Changes wait for the end of the period, unless the tree is finished: the
// last status of every node must reach the client.
TEST_F(ExecutionStatusTest, ChangesAreSentOncePerPeriodAndAlwaysAtTheEnd)
{
  ExecutionStatus status(tree_, 50ms);
  tree_.tickExactlyOnce();
  status.feedback(false, start_);

  tree_.tickExactlyOnce();
  EXPECT_FALSE(status.feedback(false, start_ + 10ms).has_value());
  EXPECT_TRUE(status.feedback(false, start_ + 50ms).has_value());
}

TEST_F(ExecutionStatusTest, TheLastChangesAreSentWhenTheTreeFinishes)
{
  ExecutionStatus status(tree_, 50ms);
  tree_.tickExactlyOnce();
  status.feedback(false, start_);

  tree_.tickExactlyOnce();
  EXPECT_TRUE(status.feedback(true, start_ + 1ms).has_value());
}

}  // namespace stepit_server::test
