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
#include <optional>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/contrib/json.hpp>
#include <stepit_server/execution_status.hpp>
#include <stepit_server/progress.hpp>

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

// A reactive parent halts its running child when a condition fails: the child
// is reported HALTED, not left RUNNING.
TEST(ExecutionStatus, AHaltedNodeIsReportedAsHalted)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<Wait>("Wait");
  int checks = 0;
  factory.registerSimpleCondition("Check", [&checks](BT::TreeNode&) {
    return ++checks == 1 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  // uids: 1 ReactiveSequence, 2 Check, 3 Wait.
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <ReactiveSequence>
          <Check/>
          <Wait/>
        </ReactiveSequence>
      </BehaviorTree>
    </root>)");
  ExecutionStatus status(tree);
  const auto start = ExecutionStatus::Clock::now();

  ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  status.feedback(false, start);
  ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::FAILURE);

  const auto message = nlohmann::json::parse(status.feedback(true, start + 10ms).value());
  EXPECT_EQ(message.at("nodes"), (nlohmann::json{ { "1", "FAILURE" }, { "2", "FAILURE" }, { "3", "HALTED" } }));
}

namespace
{

/// @brief Runs for three ticks, reporting how many it has done.
class Count : public BT::StatefulActionNode, public ProgressReporter
{
public:
  using BT::StatefulActionNode::StatefulActionNode;

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    ticks_ = 0;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    return ++ticks_ == 3 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
  }

  std::optional<Progress> progress() const override
  {
    return Progress{ static_cast<double>(ticks_), 3.0 };
  }

private:
  int ticks_ = 0;
};

class ExecutionProgressTest : public testing::Test
{
protected:
  void SetUp() override
  {
    factory_.registerNodeType<Count>("Count");
    factory_.registerNodeType<Wait>("Wait");
    // uids: 1 Sequence, 2 Count, 3 Wait.
    tree_ = factory_.createTreeFromText(R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="Main">
          <Sequence>
            <Count/>
            <Wait/>
          </Sequence>
        </BehaviorTree>
      </root>)");
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  ExecutionStatus::Clock::time_point start_ = ExecutionStatus::Clock::now();
};

}  // namespace

TEST_F(ExecutionProgressTest, ARunningReporterSendsItsProgress)
{
  ExecutionStatus status(tree_);
  ASSERT_EQ(tree_.tickExactlyOnce(), BT::NodeStatus::RUNNING);

  const auto message = nlohmann::json::parse(status.feedback(false, start_).value());
  EXPECT_EQ(message.at("progress"), (nlohmann::json{ { "2", { { "done", 0.0 }, { "total", 3.0 } } } }));
}

// A motion runs for many ticks without changing status: its progress alone
// must make a message, or the client sees it only when it ends.
TEST_F(ExecutionProgressTest, AChangedProgressIsSentWithoutAChangedStatus)
{
  ExecutionStatus status(tree_, 50ms);
  tree_.tickExactlyOnce();
  status.feedback(false, start_);

  tree_.tickExactlyOnce();
  EXPECT_FALSE(status.feedback(false, start_ + 10ms).has_value()) << "once per period";
  const auto message = nlohmann::json::parse(status.feedback(false, start_ + 50ms).value());
  EXPECT_TRUE(message.at("nodes").empty());
  EXPECT_EQ(message.at("progress"), (nlohmann::json{ { "2", { { "done", 1.0 }, { "total", 3.0 } } } }));
}

TEST_F(ExecutionProgressTest, AnUnchangedProgressIsNotSentAgain)
{
  ExecutionStatus status(tree_);
  tree_.tickExactlyOnce();
  status.feedback(false, start_);

  EXPECT_FALSE(status.feedback(false, start_ + 1s).has_value());
}

TEST_F(ExecutionProgressTest, AnEndedReporterSendsNoProgress)
{
  ExecutionStatus status(tree_);
  for (int i = 0; i < 4; ++i)
  {
    tree_.tickExactlyOnce();
  }
  ASSERT_EQ(tree_.rootNode()->status(), BT::NodeStatus::RUNNING) << "Wait runs";

  const auto message = nlohmann::json::parse(status.feedback(false, start_).value());
  EXPECT_FALSE(message.contains("progress"));
  EXPECT_EQ(message.at("nodes").at("2"), "SUCCESS");
}

TEST(ExecutionStatus, ANodeWithoutProgressSendsNone)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<Wait>("Wait");
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4"><BehaviorTree ID="Main"><Wait/></BehaviorTree></root>)");
  ExecutionStatus status(tree);
  tree.tickExactlyOnce();

  EXPECT_FALSE(nlohmann::json::parse(status.feedback(false).value()).contains("progress"));
}

TEST_F(ExecutionStatusTest, TheFirstMessageCarriesTheRun)
{
  ExecutionStatus status(tree_, ExecutionStatus::kDefaultPeriod, 7);
  tree_.tickExactlyOnce();
  EXPECT_EQ(parse(status.feedback(false, start_)).at("run"), 7);

  tree_.tickExactlyOnce();
  EXPECT_FALSE(parse(status.feedback(true, start_ + 10ms)).contains("run"));
}

TEST_F(ExecutionStatusTest, ASnapshotIsTheWholeRun)
{
  ExecutionStatus status(tree_, ExecutionStatus::kDefaultPeriod, 7);
  tree_.tickExactlyOnce();
  status.feedback(false, start_);
  tree_.tickExactlyOnce();
  status.feedback(true, start_ + 10ms);

  // Every node, though the last feedback carried the changes only.
  const auto snapshot = nlohmann::json::parse(status.snapshot());
  EXPECT_EQ(snapshot.at("run"), 7);
  EXPECT_EQ(snapshot.at("objective"), "Main");
  EXPECT_NE(snapshot.at("tree").get<std::string>().find(R"(_uid="5")"), std::string::npos);
  EXPECT_EQ(snapshot.at("nodes"), (nlohmann::json{ { "1", "SUCCESS" },
                                                   { "2", "SUCCESS" },
                                                   { "3", "SUCCESS" },
                                                   { "4", "SUCCESS" },
                                                   { "5", "FAILURE" },
                                                   { "6", "SUCCESS" } }));
  EXPECT_TRUE(snapshot.at("running"));
  EXPECT_FALSE(snapshot.contains("status"));
}

TEST_F(ExecutionStatusTest, TheLastSnapshotTellsHowTheRunEnded)
{
  ExecutionStatus status(tree_);
  tree_.tickExactlyOnce();

  const auto snapshot = nlohmann::json::parse(
      status.snapshot(ExecutionStatus::Ending{ BT::NodeStatus::FAILURE, false, "Preempted by objective 'Other'" }));
  EXPECT_FALSE(snapshot.at("running"));
  EXPECT_EQ(snapshot.at("status"), "FAILURE");
  EXPECT_FALSE(snapshot.at("cancelled"));
  EXPECT_EQ(snapshot.at("message"), "Preempted by objective 'Other'");
}

TEST_F(ExecutionProgressTest, ASnapshotHasTheProgressOfTheRunningReporters)
{
  ExecutionStatus status(tree_);
  tree_.tickExactlyOnce();

  const auto snapshot = nlohmann::json::parse(status.snapshot());
  ASSERT_TRUE(snapshot.at("progress").contains("2")) << snapshot.dump();
  EXPECT_TRUE(snapshot.at("progress").at("2").contains("done"));

  const auto ended =
      nlohmann::json::parse(status.snapshot(ExecutionStatus::Ending{ BT::NodeStatus::FAILURE, true, "" }));
  EXPECT_TRUE(ended.at("progress").empty()) << "nothing runs once it ended";
}

TEST(SnapshotPacer, PublishesTheFirstChangeAtOnce)
{
  SnapshotPacer pacer{ 200ms };
  const auto start = SnapshotPacer::Clock::now();
  EXPECT_FALSE(pacer.due(false, start)) << "nothing changed";
  EXPECT_TRUE(pacer.due(true, start));
}

TEST(SnapshotPacer, PublishesAtMostOncePerPeriod)
{
  SnapshotPacer pacer{ 200ms };
  const auto start = SnapshotPacer::Clock::now();
  ASSERT_TRUE(pacer.due(true, start));
  EXPECT_FALSE(pacer.due(true, start + 50ms));
  EXPECT_FALSE(pacer.due(true, start + 150ms));
  EXPECT_TRUE(pacer.due(true, start + 200ms));
}

// A change held back is not lost: it goes out once the period has passed,
// though nothing changes after it, e.g. while a long move runs.
TEST(SnapshotPacer, PublishesAHeldBackChangeOnceThePeriodHasPassed)
{
  SnapshotPacer pacer{ 200ms };
  const auto start = SnapshotPacer::Clock::now();
  ASSERT_TRUE(pacer.due(true, start));
  EXPECT_FALSE(pacer.due(true, start + 100ms));
  EXPECT_FALSE(pacer.due(false, start + 150ms));
  EXPECT_TRUE(pacer.due(false, start + 210ms));
  EXPECT_FALSE(pacer.due(false, start + 500ms)) << "nothing more to publish";
}

}  // namespace stepit_server::test
