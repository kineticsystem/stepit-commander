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
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <btcpp_ros2_interfaces/action/execute_tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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
