# OffsetJointsBy

[`offset_joints_by.xml`](../src/stepit_objectives/objectives/offset_joints_by.xml)
offsets one or more joints, at the same time, **relative** to the position they
have when the objective starts, as fast as the motors allow. It is the relative
counterpart of [`MoveJointsTo`](MoveJointsTo.md): the two names say how they
differ, *by* an amount against *to* a position.

| Parameter | Required | Meaning |
|---|---|---|
| `joints` | yes | The joints to move, e.g. `[joint1, joint3]`. |
| `offset` | yes | The signed displacement, in radians: one number moves every joint by the same amount, a list gives one per joint, in the order of `joints`, e.g. `[-6.28, 3.14]`. |
| `max_velocity` | no | Top speed, in rad/s: one for every joint, or one per joint. Defaults to 16.96 (2.7 turns/s), 90% of the limit of the StepIt motors. |
| `max_acceleration` | no | Acceleration, in rad/s²: one for every joint, or one per joint. Defaults to 11.31 (1.8 turns/s²), 90% of the limit of the StepIt motors. |

The tree reads the current position of the joints from `/joint_states`, turns
the offset into absolute joint targets, builds a trapezoidal trajectory to them,
and sends it to the `FollowJointTrajectory` action of the
`joint_trajectory_controller`:

```
Sequence
├── SubTree EnsureControllers  (activates joint_trajectory_controller)
├── GetJointPositions          (reads /joint_states)             -> current_positions
├── OffsetVector               (pure logic: no ROS)              -> target_positions
├── TrapezoidalTrajectory      (pure logic: no ROS)              -> trajectory
└── FollowJointTrajectory      (calls the trajectory controller)
```

Each joint accelerates at the limit, cruises at top speed and brakes at the
limit; a move too short to reach the top speed is a triangle. All joints start
and stop together, each moving in proportion to its distance. There is no
`duration`: the time follows from the distance and the limits, and lowering the
limits moves more gently, e.g. `max_velocity: 3.14` for half a turn per second.

The objective starts by making sure the trajectory controller is the one
driving the robot: it cannot send a trajectory otherwise. That first step is the
`EnsureControllers` subtree, shared with [`ActivateController`](ActivateController.md).

**Sign convention.** The offset is signed, and its sign is the one of the joint
positions themselves: a **negative** offset decreases the joint position, which
on the StepIt motors means turning **clockwise**, as in the StepIt README, where
`joint1` is rotated 6.28 rad clockwise by commanding the position `-6.28`. There
is no separate direction parameter: `offset: -6.28` is one turn clockwise,
`offset: 1.57` a quarter turn counterclockwise.
