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
#include <stepit_behaviors/switch_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

namespace stepit_behaviors::test
{

using Request = controller_manager_msgs::srv::SwitchController::Request;

TEST(Strictness, TheStrictnessOfTheSwitchIsNamed)
{
  EXPECT_EQ(parseStrictness("best_effort"), Request::BEST_EFFORT);
  EXPECT_EQ(parseStrictness("strict"), Request::STRICT);
  EXPECT_EQ(parseStrictness("auto"), Request::AUTO);
  EXPECT_EQ(parseStrictness("force_auto"), Request::FORCE_AUTO);
  EXPECT_EQ(parseStrictness("BEST_EFFORT"), Request::BEST_EFFORT);
}

TEST(Strictness, AnUnknownStrictnessIsRejected)
{
  EXPECT_FALSE(parseStrictness("whenever").has_value());
  EXPECT_FALSE(parseStrictness("").has_value());
}

}  // namespace stepit_behaviors::test
