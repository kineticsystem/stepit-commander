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

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

#include <behaviortree_cpp/bt_factory.h>
#include <stepit_server/tree_loader.hpp>

namespace stepit_server::test
{
namespace
{
namespace fs = std::filesystem;

std::string tree(const std::string& id)
{
  return R"(<root BTCPP_format="4"><BehaviorTree ID=")" + id + R"("><AlwaysSuccess/></BehaviorTree></root>)";
}

/**
 * @brief Folders laid out like `colcon build --symlink-install` does: the
 * installed folder holds one link per file of the source folder.
 */
class TreeLoaderTest : public testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() / ("tree_loader_" + std::to_string(::getpid()) + "_" +
                                         testing::UnitTest::GetInstance()->current_test_info()->name());
    source_ = root_ / "src" / "objectives";
    installed_ = root_ / "install" / "objectives";
    ASSERT_TRUE(source_.is_absolute() && installed_.is_absolute());
    fs::remove_all(root_);
    fs::create_directories(source_);
    fs::create_directories(installed_);
  }

  void TearDown() override
  {
    fs::remove_all(root_);
  }

  /// @brief Write a file in the source folder, later than any before it.
  void write(const std::string& name, const std::string& text)
  {
    const auto path = source_ / name;
    std::ofstream(path) << text;
    fs::last_write_time(path, fs::file_time_type::clock::now() + std::chrono::seconds{ ++writes_ });
  }

  /// @brief Write a file and link it from the installed folder, as a build does.
  void install(const std::string& name, const std::string& text)
  {
    write(name, text);
    fs::create_symlink(source_ / name, installed_ / name);
  }

  std::vector<std::string> registered()
  {
    auto ids = factory_.registeredBehaviorTrees();
    std::sort(ids.begin(), ids.end());
    return ids;
  }

  TreeLoader::Result reload()
  {
    return loader_.reloadIfChanged(factory_, { installed_ });
  }

  fs::path root_;
  fs::path source_;
  fs::path installed_;
  int writes_ = 0;
  BT::BehaviorTreeFactory factory_;
  TreeLoader loader_;
};

}  // namespace

TEST_F(TreeLoaderTest, TheFirstCallLoadsEveryTree)
{
  install("a.xml", tree("A"));
  install("b.xml", tree("B"));

  const auto result = reload();
  EXPECT_TRUE(result.reloaded);
  EXPECT_TRUE(result.first);
  EXPECT_TRUE(result.errors.empty());
  EXPECT_EQ(registered(), (std::vector<std::string>{ "A", "B" }));
}

TEST_F(TreeLoaderTest, NothingIsReloadedWhenNothingChanged)
{
  install("a.xml", tree("A"));
  reload();

  const auto result = reload();
  EXPECT_FALSE(result.reloaded);
  EXPECT_TRUE(result.changed.empty());
  EXPECT_EQ(registered(), (std::vector<std::string>{ "A" }));
}

// The link of an installed file shows the edits of the source file, e.g. a
// node disabled in an editor.
TEST_F(TreeLoaderTest, AnEditedFileIsReloaded)
{
  install("a.xml", tree("A"));
  reload();

  write("a.xml", tree("Renamed"));
  const auto result = reload();
  EXPECT_TRUE(result.reloaded);
  EXPECT_FALSE(result.first);
  EXPECT_EQ(result.changed, (std::vector<std::string>{ "a.xml" }));
  EXPECT_EQ(registered(), (std::vector<std::string>{ "Renamed" }));
}

// A file added to the source folder has no link in the installed folder until
// the next build, but its neighbours' links lead to it.
TEST_F(TreeLoaderTest, AFileAddedToTheSourceFolderIsFound)
{
  install("a.xml", tree("A"));
  reload();

  write("new.xml", tree("New"));
  const auto result = reload();
  EXPECT_EQ(result.changed, (std::vector<std::string>{ "new.xml" }));
  EXPECT_EQ(registered(), (std::vector<std::string>{ "A", "New" }));
}

TEST_F(TreeLoaderTest, TheTreesOfARemovedFileAreDropped)
{
  install("a.xml", tree("A"));
  write("b.xml", tree("B"));
  reload();

  fs::remove(source_ / "b.xml");
  const auto result = reload();
  EXPECT_EQ(result.changed, (std::vector<std::string>{ "b.xml" }));
  EXPECT_EQ(registered(), (std::vector<std::string>{ "A" }));
}

// A file reached both through its link and in the source folder is loaded
// once: twice would register its trees twice.
TEST_F(TreeLoaderTest, EachFileIsLoadedOnce)
{
  install("a.xml", tree("A"));
  EXPECT_EQ(TreeLoader::treeFiles({ installed_ }).size(), 1u);
  EXPECT_TRUE(reload().errors.empty());
}

TEST_F(TreeLoaderTest, AFileWithoutTreesIsLeftOut)
{
  install("a.xml", tree("A"));
  write("models.xml", R"(<root BTCPP_format="4"><TreeNodesModel><Action ID="Move"/></TreeNodesModel></root>)");

  const auto files = TreeLoader::treeFiles({ installed_ });
  ASSERT_EQ(files.size(), 1u);
  EXPECT_EQ(files.front().filename(), "a.xml");
}

// A file that cannot be read is reported, and the others are still loaded.
TEST_F(TreeLoaderTest, ABrokenFileIsReportedAndTheOthersLoaded)
{
  install("a.xml", tree("A"));
  install("broken.xml", "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Broken\"><Sequence></root>");

  const auto result = reload();
  ASSERT_EQ(result.errors.size(), 1u);
  EXPECT_EQ(result.errors.front().rfind("broken.xml: ", 0), 0u) << result.errors.front();
  EXPECT_EQ(registered(), (std::vector<std::string>{ "A" }));
}

// A folder that does not exist, e.g. a package not built yet, loads nothing.
TEST_F(TreeLoaderTest, AMissingFolderLoadsNothing)
{
  const auto result = loader_.reloadIfChanged(factory_, { root_ / "nowhere" });
  EXPECT_TRUE(result.errors.empty());
  EXPECT_TRUE(registered().empty());
}

}  // namespace stepit_server::test
