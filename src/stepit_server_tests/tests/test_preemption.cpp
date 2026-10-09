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

// End to end test of preemption: the real CommanderServer runs test objectives,
// built from BehaviorTree.CPP's own nodes, and a client sends it goals the way
// the editor or a gamepad does.

#include <unistd.h>

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <behaviortree_cpp/contrib/json.hpp>

#include <btcpp_ros2_interfaces/action/execute_tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "stepit_server/stepit_server.hpp"

namespace stepit_server::test
{
namespace
{
namespace fs = std::filesystem;
using namespace std::chrono_literals;
using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTree>;

constexpr auto kPackage = "preemption_trees";

/// @brief An objective made of a single node, e.g. <Sleep msec="10000"/>.
std::string objective(const std::string& id, const std::string& node)
{
  return R"(<root BTCPP_format="4" main_tree_to_execute=")" + id + R"(">
  <BehaviorTree ID=")" +
         id + R"(">)" + node + R"(</BehaviorTree>
</root>
)";
}

/**
 * @brief Keep a server alive until the process exits.
 *
 * TreeExecutionServer's destructor does not join the thread that ran the last
 * goal, so destroying a server that ran one terminates the process. The
 * servers are therefore never destroyed: each test starts its own, on an
 * action and a Groot2 port of its own, which the server keeps open.
 */
void keepAlive(std::shared_ptr<CommanderServer> server)
{
  static auto* servers = new std::vector<std::shared_ptr<CommanderServer>>();
  servers->push_back(std::move(server));
}

}  // namespace

class PreemptionTest : public testing::Test
{
protected:
  void SetUp() override
  {
    // The server finds its trees as "package/subfolder", through the package
    // index: give it a package of its own, in a temporary folder.
    root_ = fs::temp_directory_path() / ("preemption_" + std::to_string(::getpid()));
    fs::create_directories(root_ / "share" / "ament_index" / "resource_index" / "packages");
    std::ofstream{ root_ / "share" / "ament_index" / "resource_index" / "packages" / kPackage };
    const auto trees = root_ / "share" / kPackage / "trees";
    fs::create_directories(trees);
    std::ofstream{ trees / "long.xml" } << objective("Long", R"(<Sleep msec="10000"/>)");
    std::ofstream{ trees / "medium.xml" } << objective("Medium", R"(<Sleep msec="1000"/>)");
    std::ofstream{ trees / "quick.xml" } << objective("Quick", "<AlwaysSuccess/>");
    // A node that throws while the tree runs: the script reads an entry nobody wrote.
    std::ofstream{ trees / "throws.xml" }
        << objective("Throws", R"(<Sequence><Sleep msec="300"/><Script code="x := @missing + 1"/></Sequence>)");
    const char* prefix = std::getenv("AMENT_PREFIX_PATH");
    ::setenv("AMENT_PREFIX_PATH", (root_.string() + (prefix ? std::string{ ":" } + prefix : "")).c_str(), 1);

    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (client_)
    {
      client_->async_cancel_all_goals();
    }
    if (executor_)
    {
      // The server ends a cancelled tree at its next tick.
      std::this_thread::sleep_for(200ms);
      executor_->cancel();
      spinner_.join();
    }
    client_.reset();
    client_node_.reset();
    if (server_)
    {
      keepAlive(std::move(server_));
    }
    executor_.reset();
    fs::remove_all(root_);
  }

  /// @brief Start the server, with the given value of the parameter `preempt`.
  void startServer(bool preempt)
  {
    static int count = 0;
    const auto action = "preemption_test_" + std::to_string(::getpid()) + "_" + std::to_string(count);
    // Groot2 takes the port and the next one.
    const int groot2_port = 17670 + 2 * count++;
    rclcpp::NodeOptions options;
    // A name of its own: the servers of the previous tests are still alive, and
    // their latched topics would reach this test's subscriptions.
    options.arguments({ "--ros-args", "-r", "__node:=" + action });
    options.parameter_overrides({ { "action_name", action },
                                  { "groot2_port", groot2_port },
                                  { "behavior_trees", std::vector<std::string>{ std::string{ kPackage } + "/trees" } },
                                  { "preempt", preempt } });
    server_ = std::make_shared<CommanderServer>(options);

    client_node_ = std::make_shared<rclcpp::Node>("preemption_test_client");
    client_ = rclcpp_action::create_client<ExecuteTree>(client_node_, action);

    // As in main.cpp.
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 0, false,
                                                                           std::chrono::milliseconds(250));
    executor_->add_node(server_->node());
    executor_->add_node(client_node_);
    spinner_ = std::thread{ [this]() { executor_->spin(); } };
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  /// @brief Send an objective and wait for the server to accept it.
  GoalHandle::SharedPtr send(const std::string& objective)
  {
    ExecuteTree::Goal goal;
    goal.target_tree = objective;
    auto handle = client_->async_send_goal(goal);
    if (handle.wait_for(5s) != std::future_status::ready)
    {
      return nullptr;
    }
    return handle.get();
  }

  /// @brief The result of a goal, or none if it did not end within the timeout.
  std::optional<GoalHandle::WrappedResult> result(const GoalHandle::SharedPtr& handle, std::chrono::milliseconds timeout)
  {
    auto future = client_->async_get_result(handle);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
      return std::nullopt;
    }
    return future.get();
  }

  fs::path root_;
  std::shared_ptr<CommanderServer> server_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<ExecuteTree>::SharedPtr client_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
};

TEST_F(PreemptionTest, ANewGoalReplacesTheRunningObjective)
{
  startServer(true);
  const auto running = send("Long");
  ASSERT_TRUE(running);
  std::this_thread::sleep_for(300ms);

  const auto start = std::chrono::steady_clock::now();
  const auto next = send("Quick");
  ASSERT_TRUE(next);

  const auto preempted = result(running, 3s);
  ASSERT_TRUE(preempted.has_value());
  EXPECT_EQ(preempted->code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(preempted->result->return_message, "Preempted by objective 'Quick'");

  const auto finished = result(next, 3s);
  ASSERT_TRUE(finished.has_value());
  EXPECT_EQ(finished->code, rclcpp_action::ResultCode::SUCCEEDED);
  // Well before the 10 s the running objective would have taken.
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
}

// A request is meant for the objective running when it was made, not for the
// one that replaces it.
TEST_F(PreemptionTest, TheNewObjectiveRunsToItsEnd)
{
  startServer(true);
  const auto first = send("Long");
  ASSERT_TRUE(first);
  std::this_thread::sleep_for(300ms);
  const auto second = send("Medium");
  ASSERT_TRUE(second);

  const auto finished = result(second, 5s);
  ASSERT_TRUE(finished.has_value());
  EXPECT_EQ(finished->code, rclcpp_action::ResultCode::SUCCEEDED);
}

// Any client, connecting at any time, knows which objective runs: the server
// publishes its name, latched, and an empty one when it ends.
TEST_F(PreemptionTest, TheRunningObjectiveIsPublished)
{
  startServer(true);
  std::mutex mutex;
  std::vector<std::string> names;
  const auto topic = std::string{ server_->node()->get_fully_qualified_name() } + "/objective";
  const auto subscribe = [&]() {
    return client_node_->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS{ 1 }.reliable().transient_local(), [&](const std_msgs::msg::String& message) {
          const std::lock_guard<std::mutex> lock{ mutex };
          names.push_back(message.data);
        });
  };
  const auto last = [&]() {
    const std::lock_guard<std::mutex> lock{ mutex };
    return names.empty() ? std::string{ "<none>" } : names.back();
  };
  const auto waitFor = [&](const std::string& name) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (last() != name && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(20ms);
    }
    return last();
  };

  auto subscription = subscribe();
  EXPECT_EQ(waitFor(""), "");

  const auto goal = send("Medium");
  ASSERT_TRUE(goal);
  EXPECT_EQ(waitFor("Medium"), "Medium");

  // A client that comes while it runs gets its name at once.
  {
    const std::lock_guard<std::mutex> lock{ mutex };
    names.clear();
  }
  subscription = subscribe();
  EXPECT_EQ(waitFor("Medium"), "Medium");

  ASSERT_TRUE(result(goal, 5s).has_value());
  EXPECT_EQ(waitFor(""), "");
}

// A tree that throws ends like any other: its goal aborted with the message of
// the exception, and its name no longer published as running.
TEST_F(PreemptionTest, ATreeThatThrowsEndsItsRun)
{
  startServer(true);
  std::mutex mutex;
  std::vector<std::string> names;
  const auto subscription = client_node_->create_subscription<std_msgs::msg::String>(
      std::string{ server_->node()->get_fully_qualified_name() } + "/objective",
      rclcpp::QoS{ 1 }.reliable().transient_local(), [&](const std_msgs::msg::String& message) {
        const std::lock_guard<std::mutex> lock{ mutex };
        names.push_back(message.data);
      });

  const auto goal = send("Throws");
  ASSERT_TRUE(goal);
  const auto ended = result(goal, 5s);
  ASSERT_TRUE(ended.has_value());
  EXPECT_EQ(ended->code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(ended->result->return_message.find("Exception in node"), std::string::npos)
      << ended->result->return_message;
  EXPECT_EQ(ended->result->node_status.status, btcpp_ros2_interfaces::msg::NodeStatus::FAILURE);

  const auto deadline = std::chrono::steady_clock::now() + 3s;
  std::vector<std::string> published;
  do
  {
    std::this_thread::sleep_for(20ms);
    const std::lock_guard<std::mutex> lock{ mutex };
    published = names;
  } while ((published.empty() || published.back() != "") && std::chrono::steady_clock::now() < deadline);
  EXPECT_EQ(published, (std::vector<std::string>{ "", "Throws", "" }));
}

TEST_F(PreemptionTest, EveryRunIsPublishedWhole)
{
  startServer(true);
  std::mutex mutex;
  std::vector<nlohmann::json> snapshots;
  const auto topic = std::string{ server_->node()->get_fully_qualified_name() } + "/execution";
  const auto subscribe = [&]() {
    return client_node_->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS{ 1 }.reliable().transient_local(), [&](const std_msgs::msg::String& message) {
          const std::lock_guard<std::mutex> lock{ mutex };
          snapshots.push_back(nlohmann::json::parse(message.data));
        });
  };
  // The last snapshot received that matches, waiting up to 3 s.
  const auto waitFor = [&](const std::function<bool(const nlohmann::json&)>& matches) -> std::optional<nlohmann::json> {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        const std::lock_guard<std::mutex> lock{ mutex };
        if (!snapshots.empty() && matches(snapshots.back()))
        {
          return snapshots.back();
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return std::nullopt;
  };

  const auto goal = send("Medium");
  ASSERT_TRUE(goal);

  // A client that comes while it runs gets the whole run at once.
  std::this_thread::sleep_for(300ms);
  auto subscription = subscribe();
  const auto running = waitFor([](const nlohmann::json& snapshot) { return snapshot.at("running") == true; });
  ASSERT_TRUE(running);
  EXPECT_EQ(running->at("objective"), "Medium");
  EXPECT_EQ(running->at("nodes"), (nlohmann::json{ { "1", "RUNNING" } }));
  EXPECT_NE(running->at("tree").get<std::string>().find("Sleep"), std::string::npos);
  const auto run = running->at("run").get<std::uint64_t>();

  // Preempted: its end, then the next run.
  const auto next = send("Quick");
  ASSERT_TRUE(next);
  ASSERT_TRUE(result(goal, 5s).has_value());
  ASSERT_TRUE(result(next, 5s).has_value());
  const auto ended = waitFor([&](const nlohmann::json& snapshot) { return snapshot.at("run") == run + 1; });
  ASSERT_TRUE(ended);
  EXPECT_EQ(ended->at("objective"), "Quick");
  EXPECT_EQ(ended->at("running"), false);
  EXPECT_EQ(ended->at("status"), "SUCCESS");
  {
    const std::lock_guard<std::mutex> lock{ mutex };
    const auto preempted = std::find_if(snapshots.begin(), snapshots.end(), [&](const nlohmann::json& snapshot) {
      return snapshot.at("run") == run && snapshot.at("running") == false;
    });
    ASSERT_NE(preempted, snapshots.end());
    EXPECT_EQ(preempted->at("status"), "FAILURE");
    EXPECT_EQ(preempted->at("message"), "Preempted by objective 'Quick'");
  }
}

TEST_F(PreemptionTest, AGoalWithNothingRunningIsNotAPreemption)
{
  startServer(true);
  const auto goal = send("Quick");
  ASSERT_TRUE(goal);

  const auto finished = result(goal, 3s);
  ASSERT_TRUE(finished.has_value());
  EXPECT_EQ(finished->code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(finished->result->return_message, "Tree finished with status: SUCCESS");
}

TEST_F(PreemptionTest, WithoutPreemptionTheNewGoalWaits)
{
  startServer(false);
  const auto running = send("Medium");
  ASSERT_TRUE(running);
  std::this_thread::sleep_for(300ms);
  const auto next = send("Quick");
  ASSERT_TRUE(next);

  const auto finished = result(running, 3s);
  ASSERT_TRUE(finished.has_value());
  EXPECT_EQ(finished->code, rclcpp_action::ResultCode::SUCCEEDED);
  const auto queued = result(next, 3s);
  ASSERT_TRUE(queued.has_value());
  EXPECT_EQ(queued->code, rclcpp_action::ResultCode::SUCCEEDED);
}

}  // namespace stepit_server::test
