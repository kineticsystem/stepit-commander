# StepIt Commander

[![CI](https://github.com/kineticsystem/stepit-commander/actions/workflows/industrial_ci.yml/badge.svg)](https://github.com/kineticsystem/stepit-commander/actions/workflows/industrial_ci.yml)
[![Format](https://github.com/kineticsystem/stepit-commander/actions/workflows/ci-format.yml/badge.svg)](https://github.com/kineticsystem/stepit-commander/actions/workflows/ci-format.yml)
[![Linters](https://github.com/kineticsystem/stepit-commander/actions/workflows/ci-ros-lint.yml/badge.svg)](https://github.com/kineticsystem/stepit-commander/actions/workflows/ci-ros-lint.yml)

A single ROS 2 action server that commands a robot by executing *objectives*,
written as [BehaviorTree.CPP](https://www.behaviortree.dev) trees.

A client never talks to a controller directly. It sends the **name** of an
objective and a **payload** holding its parameters to one action:

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: MoveJointsTo,
    payload: '{joints: [joint1, joint2], positions: [0.0, 1.57]}'}"
```

The action server itself comes from [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)
(`BT::TreeExecutionServer`). It knows nothing about the robot: the behaviors
and the objectives are the robot's, in packages of its own, and the server loads
whatever the folders listed in its parameters hold. See
[Plugging in a Robot](#plugging-in-a-robot).

## Table of Contents <!-- omit in toc -->

- [Packages](#packages)
- [The Command](#the-command)
  - [One Objective at a Time](#one-objective-at-a-time)
- [Plugging in a Robot](#plugging-in-a-robot)
- [Build and Run](#build-and-run)
- [Tests](#tests)
- [Continuous Integration](#continuous-integration)

## Packages

| Package | Role |
|---|---|
| `stepit_server` | The single action server, its parameters and its launch file. It knows nothing about the robot. |
| `stepit_server_tests` | Tests of the server: the payload of a command, the status it reports while a tree runs, reading the objectives again when they change, and a new goal replacing the running objective. |

`BehaviorTree.ROS2` is not released as a Debian package, so it is checked out as
a git submodule under [`modules`](modules), next to `src`. Colcon builds every
package it finds under the workspace root, so its packages are built together
with ours.

The submodule is our fork,
[kineticsystem/BehaviorTree.ROS2](https://github.com/kineticsystem/BehaviorTree.ROS2),
branch `humble`: upstream's `humble`, with the fixes we have proposed upstream
and not yet released there. Today one: a node that throws while its tree is
ticked halts the tree and ends the run through `onTreeExecutionCompleted`, as
any other end of a run, where upstream aborts the goal and returns at once.
When upstream releases a fix, the fork follows it, and we go back to upstream
once the fork carries nothing of its own.

## The Command

The goal of the action is `btcpp_ros2_interfaces/action/ExecuteTree`:

| Field | Meaning |
|---|---|
| `target_tree` | The name of the objective, i.e. the `ID` of a `<BehaviorTree>`. |
| `payload` | Its parameters, as a YAML (and therefore also JSON) map. |

The server parses the payload and writes every parameter into the **global
blackboard** of the tree, where the behaviors read it through the `@` prefix,
e.g. `{@offset}`. A payload that is not a map of scalars and lists is refused,
and the goal is rejected before the tree is created.

Values are typed as follows, so that the ports of the behaviors read them
without any further conversion:

| Payload | Blackboard |
|---|---|
| `offset: -6.28` | `double` |
| `max_velocity: 3` | `double` |
| `controllers: velocity_controller` | `std::string` |
| `controllers: '5'` (quoted) | `std::string` |
| `joints: [joint1, joint2]` | `std::vector<std::string>` |
| `positions: [0.0, 1.5]` | `std::vector<double>` |

### One Objective at a Time

The server runs one objective at a time. **A goal sent while an objective runs
replaces it**: the running tree is halted at its next tick, at most 10 ms
later, which stops whatever its behaviors were doing, its goal ends as aborted
with the message `Preempted by objective '<name>'`, and the new objective
starts. Asking for something else is therefore enough to stop the robot, e.g.
from a gamepad or a Stop button that runs an objective of its own.

To make a new goal wait for the running objective to end instead, set the
parameter `preempt` to `false` in the robot's parameter file:

```yaml
/**:
  ros__parameters:
    preempt: false
```

To stop the running objective without starting another one, cancel every goal
of the action, whoever sent it, e.g. from the command line:

```bash
ros2 service call /commander/execute_objective/_action/cancel_goal \
  action_msgs/srv/CancelGoal "{}"
```

A client can also cancel its own goal, as the StepIt Editor's Stop button does.

**Any client can see what runs.** The server publishes the name of the running objective on `~/objective` (`std_msgs/String`), e.g. `/stepit_server/objective`, and an empty name when it ends. The topic is latched, reliable and transient local, so a client that subscribes at any time, e.g. a web page opened on a second device, gets the current value at once, whoever sent the goal:

```bash
ros2 topic echo /stepit_server/objective --qos-durability transient_local --qos-reliability reliable
```

**Any client can follow a run.** While a tree runs, the feedback of its goal tells the status of each node, the first message with the tree and the number of the run, the next ones only what changed: the StepIt Editor shows it, see [`ExecutionStatus`](src/stepit_server/include/stepit_server/execution_status.hpp). The feedback reaches only the client that sent the goal, so the server also publishes the whole run on `~/execution` (`std_msgs/String`), e.g. `/stepit_server/execution`, latched like `~/objective`: a JSON object with the number of the run, the objective, the tree, the last status of every node that has run, and the progress of the running ones. It is published whenever the run changes, at most every 50 ms, and once more when it ends, with how it ended, e.g. preempted. A client that comes in the middle of a run, or after, gets it whole at once:

```bash
ros2 topic echo /stepit_server/execution --qos-durability transient_local --qos-reliability reliable
```

A tree that throws ends like any other: the tree is halted, its goal is aborted with the exception's message, its name leaves `~/objective`, and its run on `~/execution` ends as failed.

## Plugging in a Robot

A robot provides two things, in packages of its own, usually built as a
workspace on top of this one:

- **Behaviors**: C++ nodes, exported as a BehaviorTree.CPP plugin with
  `BT_PLUGIN_EXPORT` and installed into a folder of the package's share
  directory, e.g. `share/my_behaviors/bt_plugins`.
- **Objectives**: BehaviorTree XML files, installed into a folder of the
  package's share directory, e.g. `share/my_objectives/objectives`. The main
  tree of each file, named by `main_tree_to_execute` on its `<root>`, is an
  objective, which a client asks for by its `ID`. Any other tree is a subtree,
  which only runs inside another tree, included with a SubTree node: the server
  rejects a goal for a subtree.

A parameter file of the robot lists these folders, as `package_name/subfolder`:

```yaml
/**:
  ros__parameters:
    plugins:
      - my_behaviors/bt_plugins
    behavior_trees:
      - my_objectives/objectives
```

and is passed to the launch file, which loads it after the server's own
[`stepit_server.yaml`](src/stepit_server/config/stepit_server.yaml):

```bash
ros2 launch stepit_server commander.launch.py params_file:=/path/to/robot.yaml
```

The server reads the XML files again before each goal whenever one was added,
changed or removed, also following the links of an installed folder back to its
source when the package was built with `--symlink-install`, so a new or edited
objective runs on the next goal, with no build and no restart. A new behavior
needs a build, and a restart of the server, which loads the plugins once.

Editors such as the [StepIt Editor](https://github.com/kineticsystem/stepit-editor)
describe the payload of an objective from a `<TreeNodesModel>` of its file,
declaring it as the ports of a `<SubTree>` with the objective's ID: one
`input_port` per entry, whose description ends with an example of its value,
after `e.g.`. The server ignores it.

## Build and Run

Check out the repository including its submodules:

```bash
git clone --recurse-submodules git@github.com:kineticsystem/stepit-commander.git
```

If the `--recurse-submodules` switch was missed, the submodules can be cloned
afterwards with:

```bash
git submodule update --init --recursive
```

Everything is built and run inside a Docker container. From the root of the
repo, create the image and the container (see [docker/README.md](docker/README.md)):

```bash
./docker/dock.sh stepit-commander build
./docker/dock.sh stepit-commander start
```

Inside the container, install the dependencies and build:

```bash
./bin/update.sh    # rosdep install
./bin/build.sh     # colcon build
```

Then start the server, with the parameter file of the robot, after sourcing the
workspace that holds its behaviors and objectives:

```bash
source install/setup.bash
source /path/to/robot_ws/install/setup.bash
ros2 launch stepit_server commander.launch.py params_file:=/path/to/robot.yaml
```

The commander also starts rosbridge, so that web applications such as the
StepIt Editor can run objectives. It listens on port 9090: change it with
`rosbridge_port:=<port>`, or leave rosbridge out with `rosbridge:=false`.

## Tests

```bash
./bin/test.sh
```

The tests of the server need no robot. [`TODO.md`](TODO.md) records the
decisions deferred about the server, and what we knew when deferring them.

## Continuous Integration

Three GitHub Actions workflows run on every push and pull request, as in StepIt
Driver:

| Workflow | What it checks |
|---|---|
| [`industrial_ci.yml`](.github/workflows/industrial_ci.yml) | Builds and tests the packages with [Industrial CI](https://github.com/ros-industrial/industrial_ci), against the main and the testing ROS repositories. |
| [`ci-format.yml`](.github/workflows/ci-format.yml) | The pre-commit hooks that need no ROS: clang-format, black, codespell, and the checks of whitespace and files. |
| [`ci-ros-lint.yml`](.github/workflows/ci-ros-lint.yml) | The ament linters of every package: copyright, lint_cmake and cpplint. |

Industrial CI does not check out submodules: it imports BehaviorTree.ROS2 from
[`stepit.repos`](stepit.repos), at the commit the submodule records. **Update
both together** when moving the submodule.

The workflows run locally with [Nektos `act`](https://github.com/nektos/act),
which reads the variables of [`.env`](.env), from a clean checkout: Industrial
CI mounts the working tree, `build` and `install` included. See
[How to run GitHub Actions locally](https://github.com/kineticsystem/stepit-motors#how-to-run-github-actions-locally)
in the README of StepIt Motors.
