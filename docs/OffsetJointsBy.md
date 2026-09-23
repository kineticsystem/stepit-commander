# OffsetJointsBy

[`offset_joints_by.xml`](../src/commander_objectives/objectives/offset_joints_by.xml)
offsets one or more joints, at the same time, **relative** to the position they
have when the objective starts. It is the relative counterpart of
[`MoveJointsTo`](MoveJointsTo.md): the two names say how they differ, *by* an amount against *to* a
position.

| Parameter | Required | Meaning |
|---|---|---|
| `joints` | yes | The joints to move, e.g. `[joint1, joint3]`. |
| `offset` | yes | The signed displacement of each joint, in radians. |
| `duration` | no | Time to complete the motion, in seconds. Defaults to 5. |

The tree reads the current position of the joints from `/joint_states`, turns
the offset into absolute joint targets, and sends them as a single waypoint to
the `FollowJointTrajectory` action of the `joint_trajectory_controller`:

```
Sequence
├── SubTree EnsureControllers  (activates joint_trajectory_controller)
├── GetJointPositions          (reads /joint_states)             -> current_positions
├── OffsetJointPositions       (pure logic: no ROS)              -> target_positions
└── FollowJointTrajectory      (calls the trajectory controller)
```

The objective starts by making sure the trajectory controller is the one
driving the robot: it cannot send a trajectory otherwise. That first step is the
`EnsureControllers` subtree, shared with [`ActivateController`](ActivateController.md).

**Sign convention.** The offset is signed, and its sign is the one of the joint
positions themselves: a **negative** offset decreases the joint position, which
on the StepIt motors means turning **clockwise**, as in the StepIt README, where
`joint1` is rotated 6.28 rad clockwise by commanding the position `-6.28`. There
is no separate direction parameter: `offset: -6.28` is one turn clockwise,
`offset: 1.57` a quarter turn counterclockwise.
