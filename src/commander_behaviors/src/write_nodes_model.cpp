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

// Writes the <TreeNodesModel> of the behaviors to the given file, or to the
// standard output. See commander_behaviors/nodes_model.hpp.

#include <fstream>
#include <iostream>

#include "commander_behaviors/nodes_model.hpp"

int main(int argc, char** argv)
{
  if (argc > 2)
  {
    std::cerr << "Usage: write_nodes_model [file]" << std::endl;
    return 2;
  }
  const auto model = commander_behaviors::nodesModel();
  if (argc == 1)
  {
    std::cout << model;
    return 0;
  }
  std::ofstream file(argv[1]);
  file << model;
  if (!file)
  {
    std::cerr << "Cannot write " << argv[1] << std::endl;
    return 1;
  }
  return 0;
}
