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

#include "stepit_server/payload.hpp"

#include <optional>

#include <yaml-cpp/yaml.h>

namespace stepit_server
{
namespace
{

/// @brief Parse a number, but only if the whole text is a number.
std::optional<double> toNumber(const std::string& text)
{
  if (text.empty())
  {
    return std::nullopt;
  }
  try
  {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size())
    {
      return std::nullopt;
    }
    return value;
  }
  catch (const std::exception&)
  {
    return std::nullopt;
  }
}

/// @brief True when the scalar was quoted in the source, e.g. "5" or '5'.
bool isQuoted(const YAML::Node& node)
{
  return node.Tag() == "!";
}

PayloadValue parseScalar(const YAML::Node& node)
{
  const auto text = node.as<std::string>();
  if (!isQuoted(node))
  {
    if (const auto number = toNumber(text))
    {
      return *number;
    }
  }
  return text;
}

PayloadValue parseSequence(const std::string& key, const YAML::Node& node)
{
  std::vector<std::string> texts;
  std::vector<double> numbers;
  bool all_numbers = node.size() > 0;

  for (const auto& item : node)
  {
    if (!item.IsScalar())
    {
      throw PayloadError("the list '" + key + "' must only contain scalars");
    }
    const auto text = item.as<std::string>();
    texts.push_back(text);
    if (all_numbers)
    {
      const auto number = isQuoted(item) ? std::nullopt : toNumber(text);
      if (number)
      {
        numbers.push_back(*number);
      }
      else
      {
        all_numbers = false;
      }
    }
  }

  if (all_numbers)
  {
    return numbers;
  }
  return texts;
}

}  // namespace

Payload parsePayload(const std::string& text)
{
  YAML::Node root;
  try
  {
    root = YAML::Load(text);
  }
  catch (const YAML::Exception& ex)
  {
    throw PayloadError(std::string{ "the payload is not valid YAML: " } + ex.what());
  }

  if (!root.IsDefined() || root.IsNull())
  {
    return {};
  }
  if (!root.IsMap())
  {
    throw PayloadError("the payload must be a map, for example "
                       "{joints: [joint1], direction: clockwise, rotation: 6.28}");
  }

  Payload payload;
  for (const auto& entry : root)
  {
    const auto key = entry.first.as<std::string>();
    const auto& value = entry.second;
    if (value.IsScalar())
    {
      payload.emplace(key, parseScalar(value));
    }
    else if (value.IsSequence())
    {
      payload.emplace(key, parseSequence(key, value));
    }
    else
    {
      throw PayloadError("the value of '" + key + "' must be a scalar or a list");
    }
  }
  return payload;
}

void writeToBlackboard(const Payload& payload, BT::Blackboard& blackboard)
{
  for (const auto& [key, value] : payload)
  {
    std::visit([&blackboard, &key = key](const auto& v) { blackboard.set(key, v); }, value);
  }
}

}  // namespace stepit_server
