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

#include <stepit_server/payload.hpp>

namespace stepit_server::test
{

TEST(Payload, TheRotationCommand)
{
  const auto payload = parsePayload("{joints: [joint1, joint3], direction: clockwise, rotation: 6.28, duration: 2.5}");

  ASSERT_EQ(payload.size(), 4u);
  EXPECT_EQ(std::get<std::vector<std::string>>(payload.at("joints")), (std::vector<std::string>{ "joint1", "joint3" }));
  EXPECT_EQ(std::get<std::string>(payload.at("direction")), "clockwise");
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("rotation")), 6.28);
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("duration")), 2.5);
}

TEST(Payload, JsonIsValidYamlAndIsAccepted)
{
  const auto payload = parsePayload(R"({"direction": "clockwise", "rotation": 1.5})");

  ASSERT_EQ(payload.size(), 2u);
  EXPECT_EQ(std::get<std::string>(payload.at("direction")), "clockwise");
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("rotation")), 1.5);
}

TEST(Payload, AnEmptyPayloadHasNoParameter)
{
  EXPECT_TRUE(parsePayload("").empty());
}

TEST(Payload, NumbersAreParsedAsNumbers)
{
  const auto payload = parsePayload("{a: 1, b: -2.5, c: 1e3}");
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("a")), 1.0);
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("b")), -2.5);
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("c")), 1000.0);
}

TEST(Payload, AQuotedNumberIsAString)
{
  const auto payload = parsePayload("{a: '1', b: [\"2\", \"3\"]}");
  EXPECT_EQ(std::get<std::string>(payload.at("a")), "1");
  EXPECT_EQ(std::get<std::vector<std::string>>(payload.at("b")), (std::vector<std::string>{ "2", "3" }));
}

TEST(Payload, AListOfNumbersIsAListOfNumbers)
{
  const auto payload = parsePayload("{positions: [0.0, 1.5, -2]}");
  EXPECT_EQ(std::get<std::vector<double>>(payload.at("positions")), (std::vector<double>{ 0.0, 1.5, -2.0 }));
}

TEST(Payload, AnEmptyListIsAListOfStrings)
{
  const auto payload = parsePayload("{joints: []}");
  EXPECT_TRUE(std::get<std::vector<std::string>>(payload.at("joints")).empty());
}

TEST(Payload, TheCommandParametersMustBeAMap)
{
  EXPECT_THROW(parsePayload("[1, 2, 3]"), PayloadError);
  EXPECT_THROW(parsePayload("clockwise"), PayloadError);
}

TEST(Payload, NestedValuesAreNotSupported)
{
  EXPECT_THROW(parsePayload("{joint1: {rotation: 1.0}}"), PayloadError);
}

TEST(Payload, InvalidYamlIsRejected)
{
  EXPECT_THROW(parsePayload("{joints: [joint1"), PayloadError);
}

TEST(Payload, TheParametersAreReadableFromTheBlackboard)
{
  const auto payload = parsePayload("{joints: [joint1], direction: clockwise, rotation: 6.28}");

  auto blackboard = BT::Blackboard::create();
  writeToBlackboard(payload, *blackboard);

  EXPECT_EQ(blackboard->get<std::vector<std::string>>("joints"), (std::vector<std::string>{ "joint1" }));
  EXPECT_EQ(blackboard->get<std::string>("direction"), "clockwise");
  EXPECT_DOUBLE_EQ(blackboard->get<double>("rotation"), 6.28);
}

}  // namespace stepit_server::test
