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

#include "stepit_server/tree_loader.hpp"

#include <fstream>
#include <iterator>
#include <regex>
#include <set>
#include <system_error>
#include <utility>

namespace stepit_server
{
namespace
{

namespace fs = std::filesystem;

std::string readFile(const fs::path& file)
{
  std::ifstream in(file);
  return { std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
}

/// @brief Whether the file defines a tree to run, and not only e.g. node models.
bool hasBehaviorTree(const fs::path& file)
{
  return readFile(file).find("<BehaviorTree") != std::string::npos;
}

}  // namespace

std::vector<fs::path> TreeLoader::treeFiles(const std::vector<fs::path>& folders)
{
  // Every folder to read: the given ones, and those their links point to.
  std::vector<fs::path> directories;
  std::set<fs::path> seen;
  const auto add = [&](const fs::path& directory) {
    std::error_code error;
    const auto canonical = fs::canonical(directory, error);
    if (!error && fs::is_directory(canonical) && seen.insert(canonical).second)
    {
      directories.push_back(canonical);
    }
  };
  for (const auto& folder : folders)
  {
    add(folder);
    std::error_code error;
    for (const auto& entry : fs::directory_iterator(folder, error))
    {
      if (entry.is_symlink() && entry.path().extension() == ".xml")
      {
        add(fs::canonical(entry.path(), error).parent_path());
      }
    }
  }

  // Each file once, however many ways lead to it.
  std::set<fs::path> files;
  for (const auto& directory : directories)
  {
    std::error_code error;
    for (const auto& entry : fs::directory_iterator(directory, error))
    {
      if (entry.path().extension() != ".xml")
      {
        continue;
      }
      const auto file = fs::canonical(entry.path(), error);
      if (!error && fs::is_regular_file(file) && hasBehaviorTree(file))
      {
        files.insert(file);
      }
    }
  }
  return { files.begin(), files.end() };
}

std::string TreeLoader::mainTree(const fs::path& file)
{
  // The attribute of the <root> tag, whatever comments come before it.
  static const std::regex comment{ R"(<!--[\s\S]*?-->)" };
  static const std::regex root{ R"(<root\b[^>]*>)" };
  static const std::regex main{ R"(\bmain_tree_to_execute\s*=\s*["']([^"']*)["'])" };
  const auto text = std::regex_replace(readFile(file), comment, "");
  std::smatch tag;
  std::smatch attribute;
  if (!std::regex_search(text, tag, root))
  {
    return {};
  }
  const std::string root_tag = tag.str();
  return std::regex_search(root_tag, attribute, main) ? attribute[1].str() : std::string{};
}

bool TreeLoader::isObjective(const std::string& tree_id) const
{
  return objectives_.count(tree_id) > 0;
}

TreeLoader::Result TreeLoader::reloadIfChanged(BT::BehaviorTreeFactory& factory, const std::vector<fs::path>& folders)
{
  std::map<fs::path, Stamp> stamps;
  for (const auto& file : treeFiles(folders))
  {
    std::error_code time_error;
    std::error_code size_error;
    Stamp stamp{ fs::last_write_time(file, time_error), fs::file_size(file, size_error) };
    if (!time_error && !size_error)
    {
      stamps.emplace(file, stamp);
    }
  }

  Result result;
  for (const auto& [file, stamp] : stamps)
  {
    const auto old = stamps_.find(file);
    if (old == stamps_.end() || !(old->second == stamp))
    {
      result.changed.push_back(file.filename().string());
    }
  }
  for (const auto& [file, stamp] : stamps_)
  {
    if (stamps.find(file) == stamps.end())
    {
      result.changed.push_back(file.filename().string());
    }
  }
  if (loaded_ && result.changed.empty())
  {
    return result;
  }

  factory.clearRegisteredBehaviorTrees();
  objectives_.clear();
  for (const auto& [file, stamp] : stamps)
  {
    try
    {
      factory.registerBehaviorTreeFromFile(file.string());
      if (auto main_tree = mainTree(file); !main_tree.empty())
      {
        objectives_.insert(std::move(main_tree));
      }
    }
    catch (const std::exception& ex)
    {
      result.errors.push_back(file.filename().string() + ": " + ex.what());
    }
  }
  stamps_ = std::move(stamps);
  result.first = !loaded_;
  loaded_ = true;
  result.reloaded = true;
  return result;
}

}  // namespace stepit_server
