# OffsetJointsDirectlyBy

[`offset_joints_directly_by.xml`](../src/stepit_objectives/objectives/offset_joints_directly_by.xml)
moves the joints **by** a signed offset, relative to where they are when the
objective starts, through the position controller: the microcontroller plans
each move itself, on its own trapezoid, as fast as the motors allow. It is the
direct counterpart of [`OffsetJointsBy`](OffsetJointsBy.md), which plans a
trajectory and sends it to the trajectory controller.

| Parameter | Required | Meaning |
|---|---|---|
| `joints` | yes | The joints to move, e.g. `joint1` or `[joint1, joint2]`. |
| `offset` | yes | How far to move them, in radians: one for every joint, or one per joint. Negative turns a StepIt motor clockwise. |

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: OffsetJointsDirectlyBy,
    payload: '{joints: [joint1, joint2], offset: -6.28}'}"
```

```
Sequence
├── SubTree EnsureControllers  (activates position_controller)
├── GetJointPositions          (reads /joint_states)             -> current_positions
├── OffsetVector               (pure logic: no ROS)              -> target_positions
└── CommandJointPositions      (publishes on /position_controller/commands, waits on /joint_states)
```

It succeeds once every joint is within 0.01 rad of its target and has stopped,
and fails if they are not there after 60 s. Cancelling it deactivates the
position controller, and the hardware brakes every joint to rest: see
[`MoveJointsDirectlyTo`](MoveJointsDirectlyTo.md).

It trades what [`MoveJointsDirectlyTo`](MoveJointsDirectlyTo.md) trades: the
joints are not synchronised, the limits are those of the motors, and the joints
left out are sent where they already are. That page also explains why it is
faster than `OffsetJointsBy`.
