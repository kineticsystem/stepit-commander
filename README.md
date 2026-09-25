# StepIt Commander

A single ROS 2 action server that commands the [StepIt](https://github.com/kineticsystem/stepit)
robot by executing *objectives*, written as [BehaviorTree.CPP](https://www.behaviortree.dev)
trees.

A client never talks to a controller directly. It sends the **name** of an
objective and a **payload** holding its parameters to one action:

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: OffsetJointsBy,
    payload: '{joints: [joint1, joint3], offset: -6.28}'}"
```

The action server itself comes from [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)
(`BT::TreeExecutionServer`): it loads every objective and every behavior found in
the folders listed in its parameters, so a new objective is added by dropping an
XML file into a package, without touching the server.

## Table of Contents <!-- omit in toc -->

- [Packages](#packages)
- [The command](#the-command)
- [Objectives](#objectives)
- [Build and run](#build-and-run)
- [Tests](#tests)
- [Adding a new objective](#adding-a-new-objective)

## Packages

Each package has one concern, and one only.

| Package | Role |
|---|---|
| `stepit_objectives` | The objectives and the subtrees they are built from: BehaviorTree XML files, no code. `objectives/stepit_behaviors.xml` describes the behaviors for editors such as the StepIt Editor. |
| `stepit_behaviors` | The behaviors the objectives are built from. The only place that knows the topics, actions and services of the robot. |
| `stepit_server` | The single action server, its parameters and its launch file. It knows nothing about the robot. |
| `stepit_tests` | Tests: the logic of the behaviors, the payload of a command, and the objectives run end to end against a fake robot. |

`BehaviorTree.ROS2` is not released as a Debian package, so it is checked out as
a git submodule under [`modules`](modules), next to `src`. Colcon builds every
package it finds under the workspace root, so its packages are built together
with ours.

## The command

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

## Objectives

Each objective is documented in its own file under [`docs`](docs), named after
the objective, i.e. after the `target_tree` of the command:

| Objective | What it does |
|---|---|
| [`OffsetJointsBy`](docs/OffsetJointsBy.md) | Moves joints **by** a signed offset, relative to where they are. |
| [`MoveJointsTo`](docs/MoveJointsTo.md) | Moves joints **to** absolute positions. |
| [`ActivateController`](docs/ActivateController.md) | Stops the controller driving the robot and activates another one. |
| [`SpinTest`](docs/SpinTest.md) | Hardware test: joint *k* turns *k* times clockwise at 90% of the motors' limits, then all return home. |

## Build and run

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

Start the StepIt robot in its own container, as described in its README, then
start the commander:

```bash
source install/setup.bash
ros2 launch stepit_server commander.launch.py
```

From another shell in the container, rotate `joint1` and `joint3` by one turn
clockwise:

```bash
source install/setup.bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: OffsetJointsBy,
    payload: '{joints: [joint1, joint3], offset: -6.28}'}"
```

The commander also starts rosbridge, so that web applications such as the
[StepIt Editor](https://github.com/kineticsystem/stepit-editor) can run
objectives. It listens on port 9090: change it with `rosbridge_port:=<port>`,
or leave rosbridge out with `rosbridge:=false`.

## Tests

```bash
./bin/test.sh
```

or, for this project alone:

```bash
colcon test --packages-select stepit_tests --event-handlers console_direct+
```

The objective tests, e.g. `test_offset_joints_by_objective`, run the real
objective XML and the real behaviors against a fake robot that publishes
`/joint_states` and serves `FollowJointTrajectory`, so no hardware and no
controller are needed.

> [!WARNING]
> The fake robot uses the names of the real one. With the StepIt robot running
> on the same network and ROS domain, the tests read its joint states, switch
> its controllers and **move it**. Run them on a domain of their own:
>
> ```bash
> ROS_DOMAIN_ID=77 ./bin/test.sh
> ```

## Adding a new objective

1. Write the XML in `src/stepit_objectives/objectives`. Nothing else to do:
   the folder is already loaded by the server. A step that more than one
   objective needs goes in a tree of its own, in the same folder, called with
   `<SubTree ID="..."/>`. Such a subtree takes its parameters from ports
   (`{controllers}`), so each caller can pass its own; only the objective a
   client asks for reads the payload (`{@controllers}`). See how
   `ActivateController` forwards its payload to `EnsureControllers`.
2. If it needs a new behavior, add it to `src/stepit_behaviors` and register
   it in `stepit_behaviors::registerNodes`. It is picked up automatically,
   because the whole package is loaded as one plugin. Then regenerate the node
   models that editors such as the StepIt Editor read (`test_nodes_model` fails
   until you do):

   ```bash
   ros2 run stepit_behaviors write_nodes_model src/stepit_objectives/objectives/stepit_behaviors.xml
   ```
3. Add a test to `src/stepit_tests`.
4. Document its parameters in `docs/<ObjectiveName>.md` and add it to the
   [Objectives](#objectives) table.

The three general-purpose objectives shipped here, `OffsetJointsBy`, `MoveJointsTo` and
`ActivateController`, are built from six behaviors and show every shape a
behavior can take: a ROS action client (`FollowJointTrajectory`), service
clients (`GetActiveControllers`, `SwitchController`), a subscriber
(`GetJointPositions`) and pure logic (`OffsetVector`, `TrapezoidalTrajectory`).

Building a trajectory and following it are separate behaviors, and
`FollowJointTrajectory` sends whatever trajectory it is given to the
controller. Two nodes build one:

- `CubicTrajectory`: a single waypoint, reached at rest after a given
  duration. The controller joins it with a cubic, which reaches its peak
  acceleration only at the start and the end, and its top speed only halfway.
  No objective uses it; it is there for a move that must take a given time.
- `TrapezoidalTrajectory`: as fast as the limits allow, 2.7 turns/s and
  1.8 turns/s² by default, 90% of those of the StepIt motors: at 100% the
  motors trail their commands and arrive late. Each joint accelerates at
  the limit, cruises at top speed and brakes at the limit; all joints start and
  stop together. It needs the positions the joints start from, e.g. from
  `GetJointPositions`. Every objective that moves the robot uses it.
