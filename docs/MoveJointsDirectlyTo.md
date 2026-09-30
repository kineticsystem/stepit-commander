# MoveJointsDirectlyTo

[`move_joints_directly_to.xml`](../src/stepit_objectives/objectives/move_joints_directly_to.xml)
moves the joints **to** the given positions through the position controller:
the microcontroller plans each move itself, on its own trapezoid, as fast as the
motors allow. It is the direct counterpart of [`MoveJointsTo`](MoveJointsTo.md),
which plans a trajectory and sends it to the trajectory controller.

| Parameter | Required | Meaning |
|---|---|---|
| `joints` | yes | The joints to move, e.g. `joint1` or `[joint1, joint2]`. |
| `positions` | yes | The absolute target of each joint, in radians: one for every joint, or one per joint. |

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: MoveJointsDirectlyTo,
    payload: '{joints: [joint1, joint2], positions: [0.0, 1.57]}'}"
```

```
Sequence
├── SubTree EnsureControllers  (activates position_controller)
└── CommandJointPositions      (publishes on /position_controller/commands, waits on /joint_states)
```

The objective succeeds once every joint is within 0.01 rad of its target and
has stopped, and fails if they are not there after 60 s.

Cancelling it, or the timeout, deactivates the position controller. The StepIt
hardware then sends a velocity of 0 to every joint the controller released, and
the microcontroller brakes each one to rest on its own profile, where it
naturally stops. Sending the joints where they are would not do: a joint moving
at speed cannot stop there, it brakes past it and comes back. After a cancel no
controller drives the robot, until the next objective activates the one it
needs, as every motion objective does first.

Why a second way to move: through the trajectory controller, two planners run
one after the other. The controller sends a new setpoint every cycle, 30 times a
second, and the microcontroller plans a trapezoid to each one as if it were the
last, so the motors trail the plan and settle only after it ends, some 0.6 s
after a long move. Here there is a single planner, the microcontroller's, which
is the fastest the motors can go.

What it costs:

- **The joints are not synchronised.** Each one runs its own profile at the
  limits of the motors, so a long and a short move started together end at
  different times. `MoveJointsTo` makes them arrive together.
- **The limits are those of the motors.** The position controller carries no
  speed, so there is no `max_velocity` nor `max_acceleration`.
- **Every joint of the controller is commanded.** The position controller takes
  one position per joint, `joint1` to `joint5`, in the order of
  `robot_description/config/controllers.yaml`. `CommandJointPositions` sends the
  joints left out where they already are, read from `/joint_states`, so they do
  not move.
