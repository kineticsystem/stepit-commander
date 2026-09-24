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

#include <commander_behaviors/nodes_model.hpp>
#include <fstream>
#include <sstream>

// The committed model must match the behaviors, or editors would show stale
// ports. NODES_MODEL_FILE is set by CMakeLists.txt.
TEST(NodesModel, TheCommittedFileIsUpToDate)
{
  std::ifstream file(NODES_MODEL_FILE);
  ASSERT_TRUE(file) << "Missing " << NODES_MODEL_FILE;
  std::stringstream content;
  content << file.rdbuf();

  EXPECT_EQ(content.str(), commander_behaviors::nodesModel())
      << NODES_MODEL_FILE << " does not match the behaviors. Regenerate it with:\n"
      << "  ros2 run commander_behaviors write_nodes_model " << NODES_MODEL_FILE;
}
