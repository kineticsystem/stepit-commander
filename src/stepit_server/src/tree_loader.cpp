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
#include <set>
#include <system_error>
#include <utility>

namespace stepit_server
{
namespace
{

namespace fs = std::filesystem;

/// @brief Whether the file defines a tree to run, and not only e.g. node models.
bool hasBehaviorTree(const fs::path& file)
{
  std::ifstream in(file);
  const std::string text{ std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
  return text.find("<BehaviorTree") != std::string::npos;
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
  for (const auto& [file, stamp] : stamps)
  {
    try
    {
      factory.registerBehaviorTreeFromFile(file.string());
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
