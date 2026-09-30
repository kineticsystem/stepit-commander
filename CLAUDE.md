# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Working environment

Everything is built, tested and run **inside the Docker container**, never on the host: the
ament linters, `colcon` and the ROS 2 Jazzy environment only exist there.

```bash
./docker/dock.sh stepit-commander build   # create image + container (also picks up Dockerfile changes)
./docker/dock.sh stepit-commander start   # start it and open a shell
```

The repo is bind-mounted at `~/ws`, so host edits are visible immediately and no rebuild is
needed for code changes. `~/ws/bin` is on the `PATH` and `docker/bashrc` defines the aliases
`build`, `test` and `update`, usable from any directory. The scripts `cd` to the workspace
root themselves.

The submodule must be present or the build fails:
`git submodule update --init --recursive`.

## Commands

```bash
update            # ./bin/update.sh -- rosdep install; run once after a dependency changes
build             # ./bin/build.sh  -- colcon build, Debug, --symlink-install
test              # ./bin/test.sh   -- colcon test + colcon test-result --all --verbose

# One test target (targets are named after the files in src/stepit_server_tests/tests)
colcon test --packages-select stepit_server_tests --ctest-args -R test_payload \
  --event-handlers console_direct+

source install/setup.bash
ros2 launch stepit_server commander.launch.py params_file:=/path/to/robot.yaml

pre-commit run -a  # in the container; on the host: SKIP=ament_copyright,ament_lint_cmake,ament_cpplint
```

## Architecture

One action server executes *objectives* written as BehaviorTree.CPP XML. It is generic: it
knows nothing about any robot, and must stay that way.

| Package | Rule |
|---|---|
| `stepit_server` | Payload parsing plus `BT::TreeExecutionServer`. Names no robot topic, action, service, behavior or objective. |
| `stepit_server_tests` | The tests of the server; `stepit_server` carries none. |

**The robot plugs in through parameters.** A robot's behaviors (a `BT_PLUGIN_EXPORT` plugin)
and objectives (XML) live in its own packages, usually a workspace built on top of this one.
Its parameter file lists their folders in `plugins` and `behavior_trees`, and is passed as
`params_file` to `commander.launch.py`, which loads it after `config/stepit_server.yaml`. The
server's own file lists no folder: never add a robot's packages to it, nor a dependency on
them to `package.xml`. The server reads the XML files again before each goal whenever one was
added, changed or removed (`TreeLoader`, called from `CommanderServer::onGoalReceived`), also
following the links of an installed folder back to the source folder, so a new or edited XML
runs on the next goal with no build or restart. A new behavior needs a restart: plugins are
loaded once.

**Objectives and subtrees.** Only the main tree of a file (`main_tree_to_execute` on its `<root>`)
can be the `target_tree` of a goal: `TreeLoader` records these at each reload (`isObjective`,
`mainTree`), and `onGoalReceived` rejects any other tree, a subtree, which only runs inside another
tree. The StepIt Editor applies the same rule to its lists and its Run button.

**Payload to blackboard.** A goal carries `target_tree` (the `<BehaviorTree>` ID) and `payload`
(a YAML/JSON map). `parsePayload` types the values — number → `double`, other scalar →
`std::string`, list of numbers → `std::vector<double>`, other list → `std::vector<std::string>`,
a quoted scalar always a string — so behavior ports read them with no conversion. They go into
the **global** blackboard, which is why objectives reference them with the `@` prefix
(`{@joints}`), while values passed between nodes of one tree have no prefix
(`{current_positions}`). `CommanderServer::onTreeCreated` unsets the previous goal's keys first;
a payload that is not a map of scalars and lists is rejected before the tree is created.

**Tests** of the server need no robot: they exercise the payload, the execution status and the
tree loader directly.

## Conventions

- Every source file carries the MIT copyright header (`ament_copyright` enforces it).
- Packages compile with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`; C++17.
- `cpplint` runs with `--linelength=121`; `clang-format` uses the repo `.clang-format`.
- `TODO.md` records deferred decisions about the server.
