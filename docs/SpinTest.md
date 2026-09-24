# SpinTest

[`spin_test.xml`](../src/stepit_objectives/objectives/spin_test.xml) is a
hardware test: joint *k* turns *k* full turns clockwise, joint1 once up to
joint5 five times, as fast as the motors allow, and then every joint returns to
where it started.

It takes no parameters: the joints, the turns and the durations are fixed in
the XML.

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: SpinTest, payload: ''}"
```

```
SpinTest
├── SubTree EnsureControllers             (joint_trajectory_controller only)
├── Stage 1: GetJointPositions → {home}   (retried until a joint state arrives)
│            OffsetJointPositions −2π
│            FollowJointTrajectory        (joints 1..5, 3.5 s)
├── Stage 2..5: GetJointPositions → {current}
│               OffsetJointPositions −2π
│               FollowJointTrajectory     (joints k..5, 3.5 s)
└── FollowJointTrajectory {home}          (all joints back, 8 s)
```

Three details are worth knowing:

- **Why stages.** `OffsetJointPositions` applies one offset to all its joints, so
  different turn counts cannot be expressed in a single move. Each stage turns
  joints *k* to 5 once more, which adds up to *k* turns for joint *k*.
- **Why 3.5 s per turn.** The motors are limited to 3.14 rad/s², far before
  they could reach their 31.4 rad/s velocity limit over one turn. The single
  waypoint `FollowJointTrajectory` sends becomes a cubic of peak acceleration
  `6 d/T²`, so one turn (2π) needs at least 3.46 s; 3.5 s peaks at ~2.6 rad/s.
  The return is 8 s because joint5 has five turns (31.4 rad) to undo.
- **Why only the first read is retried.** The subscription to `/joint_states` is
  created with the tree, i.e. with every goal, and the topic is volatile, so the
  first read usually finds no message yet. It is retried every 50 ms, up to 2 s,
  pausing only after a failure. Every later read shares the same subscription,
  which by then always holds a message, so it needs no retry.

Measured on the robot, starting from 0.0: the joints reached exactly 1 to 5
turns, peaked at 2.6 to 5.8 rad/s, and ended stopped at 0.0000 rad; the goal
took 26.5 s.
