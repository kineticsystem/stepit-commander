# Stack

[`stack.xml`](../src/stepit_objectives/objectives/stack.xml) steps joint1 and
joint2 through a grid, e.g. to take a photo at every position of a photo stack:
joint1 goes from where it is to 5 turns further in 10 steps, and at each of its
11 positions joint2 does the same, starting again from where it was at the
start. That makes 11 × 11 = 121 positions. When the grid is done, every joint
goes back to where it was at the start. Joints 3, 4 and 5 stay in place.

It takes no parameters: the joints, the 5 turns and the 10 steps are fixed in
the XML.

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: Stack, payload: ''}"
```

```
Stack
├── SubTree EnsureControllers   (joint_trajectory_controller only)
├── GetJointPositions           -> {home}
├── OffsetVector                joint1 by +10π      -> {joint1_end}
├── Steps                       {home} to {joint1_end}, 11 values   -> {row}
│   └── Sequence
│       ├── GetJointPositions, TrapezoidalTrajectory, FollowJointTrajectory   to {row}
│       ├── OffsetVector        joint2 by +10π      -> {row_end}
│       └── Steps               {row} to {row_end}, 11 values       -> {target}
│           └── Sequence
│               ├── GetJointPositions, TrapezoidalTrajectory, FollowJointTrajectory   to {target}
│               └── (the photo, to add)
└── GetJointPositions, TrapezoidalTrajectory, FollowJointTrajectory   back to {home}
```

Five details are worth knowing:

- **All five joints in every move.** `Steps` steps the positions of all five
  joints at once, and only joint1, or joint2, changes from one value to the
  next. Every trajectory therefore commands joints 3, 4 and 5 too, at the
  positions read at the start, and the controller holds them there.
- **10 steps are 11 positions.** The first position of each loop is where the
  joint already is, so the first move of each loop does not move, and the photo
  there is the first of the stack.
- **Joint2 goes back to its start** with each step of joint1: the move to the
  next row sends joint2 back 5 turns while joint1 steps half a turn.
- **Home only after a complete grid.** The way back is the last step of the
  sequence, so a grid that is cancelled, or whose move fails, leaves the joints
  where they stopped.
- **Counter-clockwise.** 5 turns is +10π rad; clockwise turns are negative, as
  in [SpinTest](SpinTest.md).

Each step is half a turn, π rad, too short to reach the top speed: a triangle
at the acceleration limit of `TrapezoidalTrajectory`, about 1.05 s. Each return
of joint2 is a trapezoid over 5 turns, about 3.4 s, and so is the way home,
where joint1 and joint2 each come back 5 turns together. The whole objective
takes about 2.5 minutes. It has not run on the robot yet.
