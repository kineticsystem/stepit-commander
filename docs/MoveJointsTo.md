# MoveJointsTo

[`move_joints_to.xml`](../src/stepit_objectives/objectives/move_joints_to.xml) is
the absolute counterpart of [`OffsetJointsBy`](OffsetJointsBy.md): it moves the joints **to** the given
positions, whatever position they are in when the objective starts, as fast as
the motors allow.

| Parameter | Required | Meaning |
|---|---|---|
| `joints` | yes | The joints to move, e.g. `[joint1, joint2]`. |
| `positions` | yes | The absolute target of each joint, in radians. One per joint. |
| `max_velocity` | no | Top speed, in rad/s: one for every joint, or one per joint. Defaults to 16.96 (2.7 turns/s), 90% of the limit of the StepIt motors. |
| `max_acceleration` | no | Acceleration, in rad/s²: one for every joint, or one per joint. Defaults to 11.31 (1.8 turns/s²), 90% of the limit of the StepIt motors. |

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: MoveJointsTo,
    payload: '{joints: [joint1, joint2], positions: [0.0, 1.57]}'}"
```

```
Sequence
├── SubTree EnsureControllers  (activates joint_trajectory_controller)
├── GetJointPositions          (reads /joint_states)             -> current_positions
├── TrapezoidalTrajectory      (pure logic: no ROS)              -> trajectory
└── FollowJointTrajectory      (calls the trajectory controller)
```

The motion is a trapezoid: each joint accelerates at the limit, cruises at top
speed and brakes at the limit. A move too short to reach the top speed is a
triangle: accelerate, then brake. All joints start and stop together, each
moving in proportion to its distance, so the joint with the longest way sets
the pace.

There is no `duration`: the time follows from the distance and the limits. To
move more gently, lower the limits, e.g. `max_velocity: 3.14` for half a turn
per second.

It needs no C++ of its own. The positions are already the targets, so no offset
is applied, but the shape of the trapezoid depends on the distance, so the
objective reads where the joints are first. Running it twice leaves the robot
where it was the first time.
