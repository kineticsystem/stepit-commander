# ActivateController

[`activate_controller.xml`](../src/commander_objectives/objectives/activate_controller.xml)
stops whichever controller is currently driving the robot and activates the
requested one instead.

| Parameter | Required | Meaning |
|---|---|---|
| `controllers` | yes | The controllers to activate, e.g. `[velocity_controller]`. A single name may be written as a scalar. |

```bash
ros2 action send_goal /commander/execute_objective \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: ActivateController, payload: '{controllers: velocity_controller}'}"
```

```
ActivateController                        (the objective: reads the payload)
└── SubTree EnsureControllers             (the reusable part)
    ├── GetActiveControllers              (calls /controller_manager/list_controllers)
    └── SwitchController                  (calls /controller_manager/switch_controller)
```

The objective itself is only an adapter: it forwards `{@controllers}` from the
payload into the `EnsureControllers` subtree, which holds the actual work. Any
objective that needs a given controller calls the same subtree with a fixed
name, as [`OffsetJointsBy`](OffsetJointsBy.md) does:

```xml
<SubTree ID="EnsureControllers" controllers="joint_trajectory_controller"/>
```

The tree first asks the controller manager which controllers are running, and
stops only those that **own a command interface**: a broadcaster such as the
`joint_state_broadcaster` reads the state of the robot without driving it, and
must keep running, or every other objective would go blind.

Two details are worth knowing:

- The switch is *not* delegated to the `FORCE_AUTO` strictness of the controller
  manager. On StepIt each joint exports both a `position` and a `velocity`
  command interface, so `joint_trajectory_controller` and `velocity_controller`
  do not conflict: asking the controller manager to resolve the switch by itself
  leaves **both** of them active. Measured on the robot, hence the explicit list
  and stop.
- The strictness used is `best_effort`, so activating the controller that is
  already running is not an error. An unknown controller still is, and because
  the controller manager applies a switch atomically, the running controller
  survives a command that could not be honoured.
