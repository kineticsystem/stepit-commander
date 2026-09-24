# TODO

Open questions and concerns, written down as they came up. The code builds, the
tests pass and the objectives were run on the robot: these are decisions we
deferred, and measurements we made while deferring them. Item 3 is the one to
read before trusting the result of a goal.

## Commanding motion

### 1. A speed instead of a duration

`FollowJointTrajectory` builds a trajectory with a single waypoint: the target
positions, zero velocity on arrival, and `time_from_start = duration` (5 s by
default). We say *where* to end up and *when* to be there, never how fast, so
the speed falls out of the arithmetic:

| Command | Average speed |
|---|---|
| `{offset: 6.28, duration: 4.0}` | ~1.6 rad/s |
| `{offset: 0.10, duration: 4.0}` | ~0.025 rad/s |

The speed is not even constant: the controller eases in and out to land at zero
velocity, so the mid-motion peak is about twice the average (measured on the
robot: ~2.3 rad/s during a move whose average was 1.05 rad/s).

Two consequences:

- To move at a known speed, the client has to compute
  `duration = |offset| / speed` for every command.
- Nothing rejects the impossible. The motors are configured with
  `max_velocity` 31.4159 rad/s and `acceleration` 3.14159 rad/s²
  (`stepit.ros2_control.xacro`), so `{offset: 31.4, duration: 0.5}` asks for
  ~63 rad/s: the controller commands a trajectory the hardware cannot follow and
  the joint simply lags behind it.

Possible answer: accept a `velocity` in the payload as an alternative to
`duration`, with `duration = |offset| / velocity` and exactly one of the two
allowed. It is a port on `FollowJointTrajectory` plus a few lines and tests, no
structural change. Optionally also refuse a command that exceeds `max_velocity`.

A trapezoid is available for the same duration, if the shape of the motion ever
matters. The controller never generates a velocity profile: it interpolates
between the waypoints it is given (positions only gives linear, positions and
velocities cubic, adding accelerations quintic). The single waypoint we send
therefore becomes a cubic, of peak velocity `1.5 d/T` and peak acceleration
`6 d/T²`. Three waypoints -- end of acceleration, start of deceleration, target
-- describe a trapezoid instead, of cruise velocity `d/(T - ta)` and
acceleration `d/(r (1 - r) T²)`, where `ta = r T` is the time spent
accelerating. Measured on the robot, 1 rad in 2 s with `r = 0.25`:

| | peak velocity | peak acceleration | time at cruise |
|---|---|---|---|
| one waypoint (cubic) | 0.770 rad/s | 1.50 rad/s² | 0.50 s |
| three waypoints (trapezoid) | 0.714 rad/s | 1.33 rad/s² | 0.83 s |

Same arrival time, 11% less peak velocity and acceleration, so a duration the
cubic cannot honour may still be feasible as a trapezoid. It also makes our
setpoints agree with the trapezoid the MCU runs internally, instead of the MCU
chasing a bell curve. The cost is building the waypoints in
`FollowJointTrajectory`.

### 2. Why the trajectory controller and not the position controller

Question raised: is a subset of joints impossible with the position controller,
or is something misconfigured in StepIt?

It is a property of the controller type, and it is now verified on the robot:

- `position_controller` is a `position_controllers/JointGroupPositionController`,
  configured with `joint1 … joint5` (`robot_description/config/controllers.yaml`).
  Controllers of that family take a `std_msgs/Float64MultiArray` on `~/commands`
  holding one value per configured joint, in order. The message has no field
  naming joints, so there is no way to express "only joint1": every command
  writes all five.
- `joint_trajectory_controller` names the joints inside the message and is
  configured with `allow_partial_joints_goal: true`, which is exactly why a
  subset works.

Publishing all five values, with only `joint1` changed and the other four set to
the positions just read from `/joint_states`, moves that joint alone: the other
four stayed at 0.0 without a twitch. The moved joint peaked at 1.047 rad/s over
0.35 rad, against the `sqrt(a d) = 1.048 rad/s` of the MCU's own trapezoid, i.e.
the fastest the hardware will go over that distance. So the workaround works,
and it is also the way to reach a position as fast as the robot can.

What it costs:

- **No completion feedback at all.** There is no action and no goal, so an
  objective ends when the command is published, not when the robot arrives.
  Recovering it means a behavior that waits on `/joint_states` for the target,
  with a position tolerance, a settled velocity and a timeout, which is
  reimplementing what the trajectory controller's goal tolerance already does.
- **No synchronisation.** Each joint runs its own profile at full acceleration,
  so a large and a small motion commanded together arrive at different times.
- **Cancelling stops nothing.** The controller goes on holding the last setpoint,
  so halting the tree does not halt the robot. The waiting behavior would have to
  publish the current position to stop it.

**Decision: keep the trajectory controller for now.** It is the only option that
gives completion feedback, joints that arrive together, and a cancel that
actually decelerates the robot, and none of the alternatives improved on all
three. The position controller stays the answer if a maximum-speed move is ever
wanted; those objectives would be named `SnapJointsTo` and `SnapJointsBy`,
beside the timed `MoveJointsTo` and `OffsetJointsBy`.

The other option, never tried, is to load several `JointGroupPositionController`
instances, each with its own `joints` list (e.g. one per joint). They claim
different command interfaces, so several can be active at once, and
`EnsureControllers` can activate the one matching the subset.

### 3. A goal succeeds without checking where the robot is

`controllers.yaml` configures no `constraints`, and in the trajectory controller
a tolerance of 0.0 means *disabled*, not *exact*. The per-joint `goal` tolerance
is therefore never applied, and the only check left is
`stopped_velocity_tolerance` (0.01 rad/s by default), which the controller folds
into the goal tolerance as its velocity component. The success condition of a
goal is thus "the last point of the trajectory is in the past and the joint is
not moving", with the position never compared.

For the durations we send, this is right by accident: at time `T` the robot is
genuinely there and stopped. It breaks as soon as a trajectory ends before the
motion does. Commanding a single waypoint with `time_from_start = 0`, measured on
the robot:

| Goal tolerances | Result | When |
|---|---|---|
| none, i.e. today's configuration | SUCCESS | 0.051 s, at 0.198 of 0.0, accelerating |
| `goal_tolerance.position = 0.01` | SUCCESS | 0.501 s, the true arrival |
| and `goal_time_tolerance = 0.1` | ABORT, tolerance violated | 0.201 s, stranded at 0.172 |

`FollowJointTrajectory` carries `goal_tolerance` and `goal_time_tolerance` in the
goal itself, so this is a field on the message we already send, not a change to
StepIt's configuration. A `goal_time_tolerance` of zero means "wait
indefinitely", which is what the second row does; a non-zero one aborts and
leaves the joint wherever it stopped, as the third shows.

## Safety and control flow

### 4. Cancelling is a controlled stop, not an emergency stop

Cancelling the `ExecuteTree` goal propagates correctly: the tree halts, the
behavior cancels the `FollowJointTrajectory` goal, and the controller holds the
position captured at that moment. But the robot decelerates at the configured
acceleration, so the overshoot grows with speed: measured 0.04 rad when creeping,
~1.7 rad when cancelled mid-motion at ~2.3 rad/s. A true emergency stop would be
a different mechanism, e.g. deactivating the controller or halting the hardware.

### 5. Switching controllers while a trajectory is running

`EnsureControllers` deactivates whatever is driving the robot. Deactivating the
trajectory controller *during* a motion has never been tested: it may or may not
stop the motors cleanly. If it matters, the objective should cancel the motion
first.

### 6. The objectives switch controllers unconditionally

`OffsetJointsBy` and `MoveJointsTo` both call `EnsureControllers`, so they stop a
controller that was activated on purpose. That is the self-healing behavior we
chose, but the alternative is to *check* the active controller and fail instead
of switching. One line of XML either way.

### 7. No timeout around the trajectory

If the trajectory controller accepts a goal and never finishes, the objective
runs until the client cancels. Wrapping `FollowJointTrajectory` in the built-in
`<Timeout msec="...">` decorator would bound it. XML only, no C++.

## Payload conventions

### 8. Scalars and lists are not symmetric

`{controllers: velocity_controller}` and `{controllers: [velocity_controller]}`
are both accepted, because those ports go through `stepit_behaviors::getNames`.
`joints` and `positions` must always be lists: `{joints: joint1, offset: -1.0}`
fails. Making them symmetric means routing `FollowJointTrajectory`'s ports
through `getNames` and adding a numeric twin of it. Small, but it is new API
surface, so it was left alone.

### 9. Radians are assumed everywhere

`offset` and `positions` are documented as radians. Nothing in the behaviors is
bound to rotary joints, but a prismatic joint would carry metres in the same
field, with no way to tell them apart. Only a documentation problem today.

## Tooling and operations

### 10. The hooks need the container

The three ament linters come from the ROS workspace, so `git commit` on the host
fails unless they are skipped:

```bash
SKIP=ament_copyright,ament_lint_cmake,ament_cpplint git commit ...
```

Committing from inside the container is the intended path: it has `pre-commit`,
`clang-format` and ROS. `pre-commit install` has not been run in either place.

### 11. No CI

StepIt has three GitHub Actions workflows (industrial_ci, format, ros-lint).
This repository has none, so nothing checks a pull request. The hooks and
`./bin/test.sh` already define what CI would have to run.

### 12. Two servers collide on the Groot2 port

Running a second `stepit_server` makes every goal fail with
`Behavior Tree exception: Address already in use`, because both try to publish
Groot2 on port 1667. Worth knowing before debugging the tree itself.

### 13. The submodule tracks a branch

`modules/BehaviorTree.ROS2` is pinned to a commit, as git always does, but
`.gitmodules` names the branch `humble`, so `git submodule update --remote`
would move it. There is no released Debian package to depend on instead.

### 14. The Dockerfile carries the container passwords

`developer:developer` and `root:docker` are in `docker/Dockerfile`, inherited
from the StepIt template. Intentional for a development container, but visible
to anyone who can read the repository.

## Worth considering

### 15. Node patterns from Nav2

There is no library of ready-made behaviors for `ros2_control` robots, but
`nav2_behavior_tree` has domain-agnostic control and decorator nodes worth
copying (Apache-2.0): `recovery_node` (run an action, run a recovery, retry) and
`rate_controller` (tick a branch at N Hz) are the two that would earn their place
here. They are built on Nav2's own base classes, so they would have to be ported
to `BehaviorTree.ROS2`, not linked against.
