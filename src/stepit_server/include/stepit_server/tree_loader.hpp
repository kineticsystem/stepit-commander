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

#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

namespace stepit_server
{

/**
 * @brief Keeps the behavior trees of a factory in step with their files.
 *
 * The server loads its trees once, when it starts. A TreeLoader, asked before
 * every goal, reloads them when any file changed since, so that a goal always
 * runs the trees as they are on disk, e.g. just saved from an editor.
 *
 * The folders are the installed ones, listed in the `behavior_trees` parameter.
 * With `colcon build --symlink-install`, each installed file is a link to the
 * source file, so edits show through, but a file added to the source folder has
 * no link until the next build. The loader therefore also reads the folders the
 * links point to. A file with no `<BehaviorTree>`, such as the node models for
 * editors, is left out.
 */
class TreeLoader
{
public:
  struct Result
  {
    /// @brief Whether the trees were loaded again.
    bool reloaded = false;
    /// @brief Whether this was the first load, when every file counts as changed.
    bool first = false;
    /// @brief The files added, changed or removed since the last load.
    std::vector<std::string> changed;
    /// @brief The files that failed to load, each with its error.
    std::vector<std::string> errors;
  };

  /**
   * @brief Reload every tree of the factory if any file changed since the last
   * call: all the registered trees are replaced by those of the files.
   *
   * The first call always loads, as nothing is known of the files yet.
   */
  Result reloadIfChanged(BT::BehaviorTreeFactory& factory, const std::vector<std::filesystem::path>& folders);

  /// @brief The files with behavior trees in the folders, and in the folders their links point to.
  static std::vector<std::filesystem::path> treeFiles(const std::vector<std::filesystem::path>& folders);

private:
  struct Stamp
  {
    std::filesystem::file_time_type time;
    std::uintmax_t size = 0;

    bool operator==(const Stamp& other) const
    {
      return time == other.time && size == other.size;
    }
  };

  bool loaded_ = false;
  std::map<std::filesystem::path, Stamp> stamps_;
};

}  // namespace stepit_server
