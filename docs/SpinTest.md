# SpinTest

[`spin_test.xml`](../src/stepit_objectives/objectives/spin_test.xml) is a
hardware test: joint *k* turns *k* full turns clockwise, joint1 once up to
joint5 five times, as fast as the motors allow within 90% of their limits, and
then every joint returns to where it started.

It takes no parameters: the joints and the turns are fixed in the XML, and both
moves run at 90% of the limits of the motors.

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: SpinTest, payload: ''}"
```

```
SpinTest
├── SubTree EnsureControllers   (joint_trajectory_controller only)
├── GetJointPositions           -> {home}
├── OffsetVector                (joint k by −2π·k)   -> {target}
├── TrapezoidalTrajectory       {home} to {target}   -> {trajectory}
├── FollowJointTrajectory       {trajectory}
├── TrapezoidalTrajectory       {target} to {home}   -> {trajectory}
└── FollowJointTrajectory       {trajectory}
```

Three details are worth knowing:

- **One move.** `OffsetVector` takes one offset per joint, so joint *k* is
  offset by *k* turns, −2π·*k*, and all the turns are a single move. The joints
  start and stop together.
- **At 90% of the limits.** Both moves are trapezoids at 16.96 rad/s
  (2.7 turns/s) and 11.31 rad/s² (1.8 turns/s²), 90% of the limits of stepit.
  Joint 5, with the longest way, accelerates for 1.5 s, cruises at top speed and
  brakes for 1.5 s: 3.4 s for its five turns (31.4 rad). The other joints follow
  the same profile scaled down, so joint *k* moves at *k*/5 of that. Motors 1
  and 2, which lose steps first, stay at a fifth and two fifths of it.
- **Why not 100%.** The microcontroller follows the commanded positions with
  its own ramp, limited to the same values, and trails them. At 100%, motor 5
  cannot catch up: in the first run it was still 0.035 rad short half a second
  after the end, and the controller aborted the goal. No step was lost.

The whole test takes about 7 s, plus the time to activate the controller. At
90% it has not run on the robot yet.
