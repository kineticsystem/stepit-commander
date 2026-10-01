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

#include <optional>

namespace stepit_server
{

/**
 * @brief How far a running node is: done out of total, in a unit of its own,
 * e.g. 3 of 11 iterations, or 2.1 of 5 seconds.
 */
struct Progress
{
  double done = 0.0;
  double total = 0.0;
};

/**
 * @brief A node that can tell how far it is while it runs.
 *
 * Optional: a node of a plugin implements it next to its BehaviorTree.CPP base
 * class, and ExecutionStatus reports its progress with its status, in the
 * feedback of the goal. A node without it, or returning nothing, has no
 * progress. progress() is called from the thread ticking the tree, after a tick.
 *
 * Header only, so that a plugin needs nothing of the server but this header:
 * link against stepit_server::progress.
 */
class ProgressReporter
{
public:
  virtual ~ProgressReporter() = default;

  /// @brief The progress of the running node, if it knows it.
  virtual std::optional<Progress> progress() const = 0;
};

}  // namespace stepit_server
