# MoveJointsTo

[`move_joints_to.xml`](../src/stepit_objectives/objectives/move_joints_to.xml) is
the absolute counterpart of [`OffsetJointsBy`](OffsetJointsBy.md): it moves the joints **to** the given
positions, whatever position they are in when the objective starts.

| Parameter | Required | Meaning |
|---|---|---|
| `joints` | yes | The joints to move, e.g. `[joint1, joint2]`. |
| `positions` | yes | The absolute target of each joint, in radians. One per joint. |
| `duration` | no | Time to complete the motion, in seconds. Defaults to 5. |

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: MoveJointsTo,
    payload: '{joints: [joint1, joint2], positions: [0.0, 1.57], duration: 3.0}'}"
```

```
Sequence
├── SubTree EnsureControllers  (activates joint_trajectory_controller)
└── FollowJointTrajectory      (calls the trajectory controller)
```

It needs no C++ of its own. The positions are already the targets, so neither
the current state of the robot nor an offset to apply to it come into
it: `GetJointPositions` and `OffsetJointPositions` are simply not in the tree,
and the payload goes straight to the controller. Running it twice leaves the
robot where it was the first time.
