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
#include <stepit_behaviors/ports.hpp>

namespace stepit_behaviors::test
{
namespace
{
/// @brief A node that does nothing but read one "list or single name" port.
class NameReader : public BT::SyncActionNode
{
public:
  NameReader(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<std::string>>("names") };
  }

  BT::NodeStatus tick() override
  {
    names = getNames(*this, "names");
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<std::string> names;
};

constexpr auto kTree = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <NameReader names="{@names}"/>
  </BehaviorTree>
</root>)";

std::vector<std::string> read(const std::function<void(BT::Blackboard&)>& set_names)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<NameReader>("NameReader");
  factory.registerBehaviorTreeFromText(kTree);

  auto global = BT::Blackboard::create();
  set_names(*global);

  auto tree = factory.createTree("MainTree", BT::Blackboard::create(global));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  return dynamic_cast<NameReader*>(tree.rootNode())->names;
}
}  // namespace

TEST(Ports, AListOfNames)
{
  const auto names = read([](BT::Blackboard& bb) {
    bb.set("names", std::vector<std::string>{ "velocity_controller", "position_controller" });
  });
  EXPECT_EQ(names, (std::vector<std::string>{ "velocity_controller", "position_controller" }));
}

// A command carrying one controller does not have to write it as a list.
TEST(Ports, ASingleNameIsAListOfOne)
{
  const auto names = read([](BT::Blackboard& bb) { bb.set("names", std::string{ "velocity_controller" }); });
  EXPECT_EQ(names, (std::vector<std::string>{ "velocity_controller" }));
}

TEST(Ports, AnUnsetPortIsEmpty)
{
  const auto names = read([](BT::Blackboard&) {});
  EXPECT_TRUE(names.empty());
}

TEST(Ports, AnEmptyNameIsEmpty)
{
  const auto names = read([](BT::Blackboard& bb) { bb.set("names", std::string{ "" }); });
  EXPECT_TRUE(names.empty());
}

}  // namespace stepit_behaviors::test
