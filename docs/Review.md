# StepIt Commander Review

## Table of Contents <!-- omit in toc -->

- [Scope and Method](#scope-and-method)
- [Verdict](#verdict)
- [The Packages and Their Boundaries](#the-packages-and-their-boundaries)
- [Interfaces and Abstractions](#interfaces-and-abstractions)
- [Separation of Concerns](#separation-of-concerns)
- [SOLID](#solid)
- [The Threads and What They Share](#the-threads-and-what-they-share)
  - [The Gap Between Two Trees](#the-gap-between-two-trees)
  - [One Callback Group for the Whole Node](#one-callback-group-for-the-whole-node)
- [The Stop, Path by Path](#the-stop-path-by-path)
- [Coupling and Duplication](#coupling-and-duplication)
- [Testability and Tests](#testability-and-tests)
- [Implementation Issues](#implementation-issues)
- [Suspicions the Probes Disproved](#suspicions-the-probes-disproved)
- [What the Documents Say and the Code Does Not](#what-the-documents-say-and-the-code-does-not)
- [Recommendations](#recommendations)

Reviewed on 2026-11-15, at commit `ac7a385`, the commit StepIt Macro pins in its `stepit.repos`. It judges the design that [docs/ARCHITECTURE.md](ARCHITECTURE.md) describes, and does not repeat it.

## Scope and Method

This review covers the whole module: [`src/stepit_server`](../src/stepit_server), 10 C++ files and 1299 lines with their licence headers, its launch file and its parameter file; and [`src/stepit_server_tests`](../src/stepit_server_tests), 4 test programs and 1246 lines. BehaviorTree.ROS2, our fork at commit `b0ae01d`, is a neighbour that another review covers: we followed the calls into [`tree_execution_server.cpp`](https://github.com/kineticsystem/BehaviorTree.ROS2/blob/b0ae01d/behaviortree_ros2/src/tree_execution_server.cpp), `bt_utils.cpp` and `bt_action_node.hpp` wherever the behaviour of the commander depends on them, and a finding that lives there says so.

The neighbours read are those the `CLAUDE.md` of StepIt Macro names: the sections `launch.commander` and `stepit_server` of its `rig.yaml`, `rig.launch.py`, `parameters.cpp` and `ros_action_node.hpp` of its plugin `stepit_behaviors`, and the clients: `gamepad_teleop.cpp`, `power_off.cpp`, `ui/src/commander/commander.ts`, and `src/client/execution.ts` of the StepIt Editor. The deliberate choices listed in [Design Decisions and Trade-offs](ARCHITECTURE.md#design-decisions-and-trade-offs) are judged as design, not reported as bugs. Each implementation issue says whether it is a safety issue for the rig, where every motion and every stop goes through this server.

Each suspected bug was reproduced before it was written down. The probes were one throwaway gtest file of 14 cases, beside the module's tests in a copy of the two packages, built in a scratch folder on top of the rig's installed modules, inside the `stepit-macro` image, on ROS domain 85 with discovery limited to the container. They ran the real `CommanderServer`, through a subclass that only registers two test nodes, one slow to construct and one whose `halt()` throws, and records the global blackboard after `onTreeCreated`. Seven more probes ran the real program `stepit_server` from a shell: signals, folders that do not exist, and one goal. Two builds had instruments: one with `-fsanitize=thread`, run under `setarch -R`, and the plain one run with `MALLOC_PERTURB_=165`, which fills freed memory with a byte. BehaviorTree.CPP is the Debian package and is not instrumented, so ThreadSanitizer only sees what one of our frames leads to. Everything was deleted afterwards. A finding marked **verified** failed its probe; the others are **from reading**.

The whole suite: `colcon test` of `stepit_server_tests` passes 49 gtest cases in 4 programs, with no failure and no skip, in about 8 s, of which 7.8 s are `test_preemption`.

## Verdict

The design holds its one idea, and holds it well: nothing in `stepit_server` names a robot, the only edge from a robot is one header with one interface, and the three helpers, `parsePayload`, `TreeLoader` and `ExecutionStatus`, are small, free of ROS and tested alone. The server itself is 211 lines. Preemption through the hooks, the latched topics and the reload before each goal are good ideas, cheaply made.

The weakness is everything that happens **when two things happen at once, or when something goes wrong**. The server was written and tested for one goal at a time, arriving while a tree ticks. Three things are shared between the executor and the thread of the goal with no lock, on the assumption that no goal arrives while a tree is being created; a goal that does arrive then is not a preemption, changes the payload of the tree being created even when it is rejected, can free the XML that tree is being built from, and leaves the node deaf to a cancel until the run ends (findings 1 to 3, verified). One goal with a malformed payload ends the process (finding 4, verified), and so does a signal while a tree runs, without halting the tree (finding 5, verified).

For the rig these are safety issues, in the sense of the project: **the stop is a goal or a cancel, and there are moments when neither stops the robot**. None is likely in a session: the gap is the time a tree takes to be created, and the other two need a bad client or a shutdown mid-run. All are cheap to close, and four of the fixes need no change to the fork. Until then the stop that does not go through the commander, the gamepad's zero velocity and the power switch, is the one to trust.

The tests cover the helpers thoroughly and the server thinly: 8 cases run it, all with one goal at a time, and none cancels, rejects, or sends a payload.

## The Packages and Their Boundaries

| Rule | Verdict | Evidence |
|---|---|---|
| `stepit_server` names no robot topic, action, service, behavior or objective | Kept | A search for `joint`, `camera`, `freezer`, `teleop`, `motor`, `velocity` and `controller` in the package finds 2 lines, both example payloads: the comment of [`payload.hpp`](../src/stepit_server/include/stepit_server/payload.hpp) and the error message of `parsePayload` (`{joints: [joint1], direction: clockwise, rotation: 6.28}`). Text, not knowledge. |
| The server's parameter file lists no folder | Kept | [`stepit_server.yaml`](../src/stepit_server/config/stepit_server.yaml) holds five values and a commented example. |
| The helpers do not know the server, nor ROS | Kept | `payload.hpp`, `tree_loader.hpp` and `execution_status.hpp` include BehaviorTree.CPP and the standard library; only `stepit_server.hpp` includes `rclcpp`. No cycle: `stepit_server.hpp` includes the three, none includes it. |
| A plugin takes one header from the server | Kept | `progress.hpp` includes `<optional>` alone, and is exported as the target `stepit_server::progress`. |
| `stepit_server` carries no test | Kept | All 4 programs are in `stepit_server_tests`. |
| BehaviorTree.ROS2 at one commit everywhere | Kept | The module's `stepit.repos` and StepIt Macro's both pin `b0ae01d`. |

`yaml-cpp` is linked `PRIVATE` and stays out of the public headers, which is right; it is also why a `YAML::Exception` that escapes `parsePayload` has a type its caller cannot name, see finding 4.

## Interfaces and Abstractions

| Interface | Where | Judgement |
|---|---|---|
| The five hooks of `TreeExecutionServer` | [`stepit_server.hpp`](../src/stepit_server/include/stepit_server/stepit_server.hpp) | The right seam for a server of 211 lines, and the source of most findings: the hooks say nothing of the thread they run on, `onGoalReceived` can only answer yes or no, `onTreeExecutionCompleted` gets one `bool` for a cancel and for a preemption and no text for an exception, and nothing is called between the acceptance of a goal and the creation of its tree. |
| `parsePayload` and `writeToBlackboard` | [`payload.hpp`](../src/stepit_server/include/stepit_server/payload.hpp) | A pure function and a value type, `Payload`. Its contract, "throws `PayloadError`", is not kept for every input, see finding 4. |
| `TreeLoader` | [`tree_loader.hpp`](../src/stepit_server/include/stepit_server/tree_loader.hpp) | Takes the factory and the folders as arguments, so it is tested with a plain `BehaviorTreeFactory` and a temporary folder. It assumes it is the only one that registers trees, and `TreeExecutionServer` registers them too, see finding 7. |
| `ExecutionStatus` and `SnapshotPacer` | [`execution_status.hpp`](../src/stepit_server/include/stepit_server/execution_status.hpp) | Both take `now` as a defaulted argument: the period is tested without sleeping. The best seam of the module. `Ending` lets a caller build endings the server never produces, and the test does, see finding 10. |
| `ProgressReporter` | [`progress.hpp`](../src/stepit_server/include/stepit_server/progress.hpp) | One method, found with a `dynamic_cast`, called on the ticking thread only. Clean. |
| The JSON of the feedback and of `~/execution` | the comment of `execution_status.hpp` | A contract in a comment, with two hand-written readers, see [Coupling and Duplication](#coupling-and-duplication). |
| The answer to a goal | `rclcpp_action` | A rejection carries no reason: the three reasons of `onGoalReceived` reach only the server's log. The architecture document says so; a client such as StepIt UI can only show "rejected". |

## Separation of Concerns

- **`CommanderServer` holds the state of the run as 12 loose members.** `payload_`, `written_keys_`, `logger_`, `execution_status_`, `snapshot_pacer_`, `tick_status_`, `runs_`, `running_`, `preempt_requested_`, `preempted_` and `preempting_objective_` with its mutex belong to one run or to the hand-over between two, and are set in `onTreeCreated` and cleared, some of them, in `onTreeExecutionCompleted`. Nothing in the class says which thread owns which. One `struct Run`, created in `onTreeCreated` and destroyed at the end, would hold most of them and leave the shared ones in sight.
- **The validation of a goal and the request to preempt are in one function**, `onGoalReceived`, which also reloads the files and stores the payload for another thread. It does four things, and the order between them is what findings 1 and 2 turn on.
- **`TreeLoader` reads XML with a regular expression** (`mainTree`) beside BehaviorTree.CPP's own parser, and finds a file "with trees" by searching its text for `<BehaviorTree` (`hasBehaviorTree`), which a comment also matches. The architecture document names the first as a price; finding 9 is what it costs.
- **The helpers keep their layers.** Nothing of ROS reaches `parsePayload`, `TreeLoader` or `ExecutionStatus`, and nothing of the JSON reaches `CommanderServer`.

## SOLID

| Principle | Verdict | Where |
|---|---|---|
| Single responsibility | Partly violated | `CommanderServer::onGoalReceived` and the 12 members above. The helpers each do one thing. |
| Open/closed | No violation found | A robot extends the server with folders and parameters; a new type of payload value is one alternative of `PayloadValue` and one branch of `parsePayload`. |
| Liskov substitution | Partly violated | `CommanderServer` narrows what its base promises: the base accepts every goal and queues it, the derived class makes the hook of one goal act on another goal's tree. The base's `onTreeExecutionCompleted(status, was_cancelled)` is called with `was_cancelled` true for a preemption, and the derived class passes it on as `cancelled`, see finding 10. |
| Interface segregation | No violation found | `ProgressReporter` has one method; the hooks are five. |
| Dependency inversion | Partly violated | The server depends on the concrete `TreeExecutionServer` by inheritance, which the architecture document chooses and prices. The cost for the tests is in [Testability and Tests](#testability-and-tests): a server that ran a goal cannot be destroyed. |

## The Threads and What They Share

The [Threads](ARCHITECTURE.md#threads) table of the architecture document is right about who runs what, and says in one clause what this section is about: `payload_`, the `TreeLoader` and the factory "rely on a goal arriving while no tree is being created". Nothing enforces that, and a client cannot know when a tree is being created.

### The Gap Between Two Trees

`handle_accepted` starts the thread of a goal and returns. From then until `onTreeCreated`, the thread runs `executeRegistration` if a parameter changed, then `createTree`, which constructs every node of the tree. During that time `running_` is false, as it also is between `onTreeExecutionCompleted` of the previous tree and this point. A second goal handled then, on the executor:

1. writes `payload_` (`stepit_server.cpp`, line 60), before it is known whether the goal is accepted;
2. calls `reloadTrees()`, which, if a file changed, clears every tree of the factory while `createTree` reads them on the other thread;
3. finds `running_` false and raises no preemption (line 86);
4. is accepted, and `handle_accepted` joins the thread of the first goal: it waits for the whole first objective.

And whatever request it had raised, `onTreeCreated` of the first tree clears `preempt_requested_` (line 154). All four were reproduced, with a tree whose first node takes 500 ms to construct and then sleeps 3 s, and a second goal sent 150 ms after the first:

| Probe | Result |
|---|---|
| The second goal is `Quick`, an `AlwaysSuccess` | The first tree runs its 3 s and succeeds. `Quick` ends 3384 ms after it was sent. Sent 1000 ms after the first instead, when the tree ticks, it preempts it at once. |
| A cancel of every goal, 700 ms later | Answered after 2671 ms, when the first tree had ended by itself. It cancelled `Quick`, the goal that was waiting, and not the tree that was running. |
| The first goal has the payload `{x: 1}`, the second is for a tree that does not exist, with `{x: 2, y: 3}` | The second goal is rejected. The first tree finds `x=2` and `y=3` in the global blackboard. |
| `quick.xml` is saved between the two goals | ThreadSanitizer reports `reloadTrees()` (line 114) freeing memory that `BT::XMLParser::PImpl::recursivelyCreateSubtree` is reading on the thread of the goal. With freed memory filled (`MALLOC_PERTURB_=165`) the process ends with a segmentation fault 3 times out of 3; the two other probes above, with the same timing and no saved file, pass with it. Without it, 5 runs out of 5 pass: the tree is built from freed memory that still holds its text. |

How wide the gap is on the rig was not measured: it is the time `createTree` takes for the rig's largest objective with the rig's plugin, whose nodes create subscriptions and clients in their constructors.

### One Callback Group for the Whole Node

`rclcpp_action::create_server` is called with no callback group, so the action server lives in the node's default group, which is mutually exclusive, with the registration timer, `get_loaded_trees` and the node's parameter services. `handle_accepted` runs in that group and blocks in `join()`. The `MultiThreadedExecutor` has other threads, but none may enter the group: **while a goal waits for the previous one, the node answers nothing**, a cancel included.

With `preempt` true this lasts one tick and the halt of the tree, except in the gap above. With `preempt` false it lasts the whole running objective, by design of the option. Reproduced with `preempt` false, a tree of 3 s, a second goal 300 ms after its start, and 200 ms later a cancel of the running goal, a call of `get_loaded_trees` and a `get_parameters`: all three were answered after 2520 ms, when the tree had ended, and the running goal ended `SUCCEEDED`, not cancelled. With nothing queued, `get_loaded_trees` answers in under 1 ms.

The rig's plugin is not caught by this: `command_joint_positions.cpp`, `get_joint_positions.cpp`, `expect_picture.cpp`, `pause_live_view.cpp` and `restore_picture_folder.cpp` each create a callback group of their own, and BehaviorTree.ROS2's wrappers spin an executor of their own. A behavior that subscribed in the default group would stop receiving while a goal is queued.

## The Stop, Path by Path

On the rig, stopping the robot is sending another goal or cancelling. What each path does, as measured:

| Path | What happens | Evidence |
|---|---|---|
| A goal while a tree ticks | The tree is halted at its next tick, its goal aborted with `Preempted by objective '<name>'`. | The suite, and the probe at 1000 ms. |
| A cancel while a tree ticks | The tree is halted before its next tick, its goal ends `CANCELED`. | Verified; the suite has no such case. |
| A goal while a tree is being created | Not a stop: it waits for the whole objective, and no cancel is answered meanwhile. | Finding 1, verified. |
| A cancel while a tree is being created, with no goal queued | Accepted; the loop sees it before the first tick. | From reading `execute()`. |
| A goal or a cancel while a node blocks in its tick | Waits for the tick to return. | From reading; the architecture document says so. |
| A halt that throws | The process survives; the tree is halted a second time, and a cancelled goal ends `ABORTED`. | Verified, see [Suspicions the Probes Disproved](#suspicions-the-probes-disproved). |
| The process gets `SIGINT` or `SIGTERM` while a tree runs | The tree is **not** halted. The process ends by `abort()`. | Finding 5, verified. |
| The process dies, e.g. finding 4 | Nothing is halted; whatever the behaviors started goes on. | From reading. |

The gamepad of StepIt Macro does not depend on the commander alone: `GamepadTeleop::stop()` first publishes a zero velocity, then sends its goal, and ignores a second press while its first goal is pending (`switch_started_`), so it cannot put two of its own goals in the gap. StepIt UI stops with the `cancel_goal` service, which is the call that goes unanswered while a goal is queued.

## Coupling and Duplication

| What | Where | Weight |
|---|---|---|
| The fields of the feedback and of the snapshot | `ExecutionStatus::feedback()` and `snapshot()`; the comment of `execution_status.hpp`; the README; `parseFeedback` and `parseSnapshot` of the editor's `execution.ts` | Medium. Three descriptions and one hand-written reader, kept in step by hand across two repositories. The editor derives `canceled` from `cancelled === true`, which is how a preempted run comes to be shown as stopped, finding 10. |
| The wording of BehaviorTree.CPP's exception | `failedNodeUid` of the editor's `execution.ts` reads the `_uid` out of `Exception in node '...::20'` | Low. It is the only way a client learns which node threw, since neither the feedback nor the snapshot says it. |
| The name of the node, in the names of the topics | `name="stepit_server"` in `commander.launch.py`; `/stepit_server/objective` in `power_off.cpp`, `power_off.yaml` and `commander.ts`; the execution topic in the editor's `settings.ts` | Low. A robot that renames the node silently loses the clients. |
| The name of the action | `action_name` in `stepit_server.yaml`; `kCommanderAction` in `gamepad_teleop.cpp`; `ACTION` in `commander.ts`; `runAction` in the editor's `settings.ts` | Low. |
| Two ways to register the trees | `TreeLoader::reloadIfChanged` and `TreeExecutionServer::executeRegistration`, which reads the installed folders only | High. The second undoes the first, finding 7. |
| Two readers of the XML | `TreeLoader::mainTree`, a regular expression, and BehaviorTree.CPP's parser | Medium, finding 9. |
| The rule "only the main tree is an objective" | `TreeLoader::isObjective`, and the editor's lists and Run button | Low. The module's `CLAUDE.md` names both. |

## Testability and Tests

What is tested is in the [Tests](ARCHITECTURE.md#tests) table of the architecture document, and its list of what is not tested is honest. To that list this review adds what it found by running it:

| Not tested | Why it matters |
|---|---|
| A goal while a tree is being created | Findings 1, 2 and 3. |
| A cancel, anywhere | It is half of the rig's stop. The last snapshot of a cancelled run says `status: RUNNING`. |
| A rejected goal through the real server, a payload reaching a tree, the previous payload being unset | `onGoalReceived` and `onTreeCreated` are covered only for a goal with no payload. The unset works: verified. |
| A payload that is not a map of scalar keys | Finding 4. `test_payload` has 3 refused payloads, none with a key that is not a scalar. |
| The end of the process | Finding 5. |
| A folder of `plugins` or `behavior_trees` that does not exist | Finding 6. `test_tree_loader` has `AMissingFolderLoadsNothing`, for `TreeLoader`, which is the half that works. |
| A parameter set while the server runs | Finding 7. |
| The endings the server really publishes | `TheLastSnapshotTellsHowTheRunEnded` builds `Ending{FAILURE, false, "Preempted by objective 'Other'"}`, an ending the server never sends: a preempted run has `cancelled` true. `EveryRunIsPublishedWhole` checks the `status` and the `message` of a preempted run, not `cancelled`. |

The seams that are missing:

- **A server cannot be destroyed.** `~TreeExecutionServer` does not join, so `test_preemption` leaks every server, gives each a node name, an action and a Groot2 port of its own, and waits 200 ms in `TearDown`. A probe of a crash needs a process of its own. It is also why the real program cannot exit cleanly, finding 5.
- **Nothing makes a tree slow to create or a halt fail.** The probes needed a subclass that overrides `registerNodesIntoFactory`; `PreemptionTest` constructs `CommanderServer` directly. A test fixture that owns such a subclass, with two or three test nodes, gives every case of the table above a place.
- **The tests wait on time.** `test_preemption` has 11 `sleep_for` calls, of 20 to 300 ms, most of them to let a tree start; the latched `~/objective` already tells when it has.

## Implementation Issues

Ordered by severity. Each says whether it is a safety issue for the rig.

| # | Issue | Proof | Safety issue for the rig |
|---|---|---|---|
| 1 | **A goal that arrives while a tree is being created does not preempt it, and no cancel is answered until that tree ends.** `onGoalReceived` reads `running_`, false until `onTreeCreated`; `handle_accepted` then joins in the node's only callback group. See [The Gap Between Two Trees](#the-gap-between-two-trees). | Verified: 3384 ms instead of at once; the cancel answered after 2671 ms, and it cancelled the wrong goal. | **Yes.** A stop sent just after an objective starts, e.g. the gamepad's just after a stack is started from StepIt UI, waits for the whole objective, and StepIt UI's Stop is not answered either. |
| 2 | **A goal in the gap, even a rejected one, replaces the payload of the tree being created.** `payload_` is written at line 60 before the goal is validated, with no lock, and read by `onTreeCreated` at line 139. | Verified: the tree sent with `{x: 1}` runs with `x=2, y=3` of a rejected goal. ThreadSanitizer did not report it: the two accesses are 350 ms apart with locks taken between them. | **Yes.** An objective moves with the numbers of another goal. |
| 3 | **A reload in the gap frees the XML documents `createTree` is reading.** `reloadTrees()` calls `clearRegisteredBehaviorTrees()` on the executor with no lock against the thread of the goal. | Verified with ThreadSanitizer, frame `stepit_server.cpp:114`, and by a segmentation fault 3 times out of 3 with `MALLOC_PERTURB_=165`; latent in a plain build, 5 runs out of 5 pass. | **Yes, when it crashes**: the process dies with a tree starting, see 5. It needs a file saved between two goals a few milliseconds apart. |
| 4 | **One goal ends the process.** A payload whose key is not a scalar, e.g. `{[a]: 1}`, makes `entry.first.as<std::string>()` (`payload.cpp`, line 136) throw `YAML::TypedBadConversion`, outside the `try` of `parsePayload`. `onGoalReceived` catches only `PayloadError`, and the executor's thread has no handler. | Verified against the real program: `terminate called after throwing an instance of 'YAML::TypedBadConversion<...>'`, exit by `abort()`. `{{a: 1}: 2}` and `? [a, b]` do the same. | **Yes.** Any client of the action can do it, and rosbridge on port 9090 gives the action to anything that reaches the port, with no authentication: it is also a security issue, a denial of service. The running tree is not halted, and no stop goes through the commander until it is restarted. |
| 5 | **A signal while a tree runs does not halt the tree, and the process aborts.** The loop of `execute()` leaves on `!rclcpp::ok()` with the status `RUNNING`, skips `haltTree()`, and calls `goal_handle->abort()` on a context that is shut down, which throws on the thread of the goal. With no tree running but one goal run since the start, `main()` destroys the server with its thread never joined: `std::terminate`. The fork owns both; `main.cpp` has no guard. | Verified against the real program. With a tree running: `Tree finished with status: RUNNING`, no `RUNNING -> IDLE` of the node, then `terminate ... Asked to publish result for goal that does not exist`. After a goal that ended: `terminate called without an active exception`. With no goal ever run: exit code 0. | **Yes.** At `SIGINT` the behaviors get no `halt()`: the motors get their stop only if the controllers' own shutdown gives it. Every stop of the rig after one goal ends the commander by `abort()`. |
| 6 | **A folder that does not exist ends the process at the start.** `RegisterPlugins` and `LoadBehaviorTrees` of the fork open a `directory_iterator` with no error code, on the registration timer. An entry with no `/`, e.g. `plugins: [my_behaviors]`, throws `std::length_error` from `GetDirectoryPath`, which compares `find_first_of` with `size()` instead of `npos`. `CommanderServer::reloadTrees` calls the same `GetDirectoryPath` at every goal. | Verified against the real program: `filesystem error: directory iterator cannot open directory`, and `basic_string::_M_create`. A package that does not exist is logged and skipped, as documented. | No: it fails at the start, with a message that names the folder. From reading: if a parameter changed and the folder went missing since, the second registration throws on the thread of the goal, before the `try` of `execute()`. |
| 7 | **A parameter set from outside drops the objectives that exist only in the source folder**, and the server's own parameters invite it. `preempt` and `execution_period` are not read-only: `ros2 param set /stepit_server preempt false` answers successful, changes nothing, since `preempt_` is read once in the constructor, and makes the next `execute()` register the trees again from the installed folders. | Verified: an objective only in the source folder runs; after `preempt` is set, its goal is accepted then aborted with `Can't find a tree with name`, the next one is rejected, until any XML file is saved. `tick_frequency` is read-only and refused. A parameter declared on the node does the same, as the rig's `CLAUDE.md` says. | No. A person who believes `preempt` is now false is misled about what a second goal does. |
| 8 | **With `preempt` false, a queued goal makes the running one impossible to cancel.** See [One Callback Group for the Whole Node](#one-callback-group-for-the-whole-node). | Verified: the cancel is answered after 2520 ms, when the tree has ended. | Not as the rig is configured, with `preempt` true. **Yes** for a robot that sets it to false: the README offers the option and does not say it. |
| 9 | **A comment of 30 kB in an objective file ends the process at the next goal.** `TreeLoader::mainTree` removes comments with `std::regex` `<!--[\s\S]*?-->`, which libstdc++ matches by recursion, one level per character. | Verified, on a thread as in the server: 20,000 bytes pass, 30,000 end with a segmentation fault. 2 MB of short comments pass. | Yes when it happens, as 4. It needs a large block commented out in one file; the rig's 16 files total 56 KiB. |
| 10 | **The last snapshot does not say how the run ended.** A preempted run has `cancelled: true`; a cancelled run has `status: RUNNING`; a run that threw has an empty `message`, and a node whose `halt()` threw stays `RUNNING` in it. The editor's `parseSnapshot` then shows a preempted run as stopped. | Verified: the four snapshots were printed. | No. A second tablet shows a wrong reason. |
| 11 | **The typing of the payload has surprises.** `!!str 5` is the number 5, since only the tag `!` counts as quoted; `inf`, `-inf`, `infinity` and `nan` are numbers, while YAML's own `.inf` and `.nan` are strings; `017` is 17; `{~: 1}` sets a key named `null`; of `{a: 1, a: 2}` the first wins and of two documents the first, silently; a scalar of 5 MB is accepted, and 3,000,000 nested brackets take 2.2 s on the executor to refuse. | Verified, 49 inputs. | For the plugin to answer: `stepit_behaviors` has no `isfinite` check, so `offset: nan` reaches a behavior as a `double`. The server passes it. |
| 12 | **What a behavior writes to the global blackboard stays for every later run.** Only the payload's keys are unset. | Verified: `@leak := 42` in one objective is found by the 6 runs after it. A later payload with the same key replaces it, with another type too. | No, as long as no objective reads an `@` entry it did not get in its payload. |
| 13 | **Every goal reads every XML file**, to search for `<BehaviorTree`, before it compares the stamps, on the executor and before the preemption is raised. | Measured, Debug build as the rig's, on the review computer: 8.7 ms for the rig's 16 files, 306 ms for 512 files of 1.8 MiB. | No at 16 files: a stop is 9 ms later, more on the Raspberry Pi. It grows with the folder. |
| 14 | **`~/objective` is empty while a goal is accepted and its tree not yet created**, and while a goal waits for another. `power_off` of StepIt Macro refuses to switch off only when the topic names an objective. | From reading. | Low: the computer can be switched off in the gap of finding 1. |
| 15 | **Every status change of every node is printed**, by the `BT::StdCoutLogger` that `onTreeCreated` always creates, with no parameter to turn it off. | From reading; the probes' output was full of it. | No. |

## Suspicions the Probes Disproved

- **A halt that throws `UnknownGoalHandleError` ends the process.** It does not, at this commit of the fork. Every `haltTree()` of `execute()` is inside its `try`, the lambda `stop_action` included. A node whose `halt()` always throws was preempted, then cancelled: the process lived, the preempted goal ended `ABORTED` with `Preempted by objective 'Quick'`, and the cancelled one `ABORTED` with `Behavior Tree exception:Goal handle is not known to this client.` instead of `CANCELED`. `halt()` was called twice per run. So `stepit_behaviors::RosActionNode` is no longer needed to keep the commander alive; it is still what makes a cancelled goal end as cancelled, calls `onHalt()`, and keeps the snapshot right, finding 10.
- **Anchors, aliases, merge keys, deep nesting or huge scalars break the parser.** They do not: an alias gives the value it points to, `<<` and an undefined alias are a `PayloadError`, 3,000,000 nested brackets are a `PayloadError` from yaml-cpp's own depth guard. Only the non-scalar key escapes, finding 4.
- **A payload whose type differs from an entry already in the global blackboard throws in `onTreeCreated`, after `written_keys_` was cleared**, leaving half a payload behind for ever. Not reproduced: a number over a string entry and a string over a number entry were both written.
- **A package that does not exist ends the process.** It is logged and skipped, as the architecture document says. A subfolder that does not exist is the one that ends it, finding 6.
- **A parameter the server refuses still causes the second registration.** It does not: the refused `tick_frequency` left the objectives alone.

## What the Documents Say and the Code Does Not

| Document | It says | The code |
|---|---|---|
| `CLAUDE.md` of StepIt Macro, and the comment of `ros_action_node.hpp` | Halting a node whose goal has just ended makes BehaviorTree.ROS2 throw `UnknownGoalHandleError`, "which would end the commander's process". | Not at `b0ae01d`: see above. The reason left for the wrapper is the outcome of the goal, not the life of the process. |
| `rig.yaml` of StepIt Macro, section `stepit_server` | "The parameters of the rig's behaviors, which the plugin declares on the commander's node". | `parameters.cpp` declares none and reads the overrides, as the same repository's `CLAUDE.md` requires. |
| [ARCHITECTURE.md](ARCHITECTURE.md#the-payload-and-the-blackboard) | A payload that is not valid YAML or not a map, a value that is a map, and a list that holds anything but scalars throw `PayloadError`. | A key that is not a scalar throws `YAML::TypedBadConversion`, finding 4. |
| [ARCHITECTURE.md](ARCHITECTURE.md#at-the-start) | A package that is not found, a plugin or a file that fails to load is skipped; "the server starts anyway". | True of those three. A subfolder that does not exist, or an entry with no `/`, ends it, finding 6. |
| [ARCHITECTURE.md](ARCHITECTURE.md#threads) | "The program never destroys it before it exits." | `main()` destroys it when `spin()` returns, and after one goal that is `std::terminate`, finding 5. |
| [ARCHITECTURE.md](ARCHITECTURE.md#how-a-run-ends) | Five ends of a run. | A sixth: the shutdown, with `onTreeExecutionCompleted(RUNNING, false)` and no halt, finding 5. |
| [ARCHITECTURE.md](ARCHITECTURE.md#preemption) | A goal in the gap "raises no request". | True, and understated: it also waits for the whole objective, changes its payload, and blocks every cancel, findings 1 and 2. |
| README, One Objective at a Time | The running tree is halted "at most 10 ms later". | Not in the gap, and not while a node blocks in its tick. |
| `.claude/CLAUDE.md` of the module | "There is no `docs/ARCHITECTURE.md` yet." | There is one now, uncommitted. |

## Recommendations

Ordered by value against effort. The first five need no change to the fork.

1. **Refuse, never throw, in `onGoalReceived`.** Move the conversion of the key inside a `try` of `parsePayload` that turns every `YAML::Exception` into a `PayloadError`, and catch `std::exception` around the whole hook. Add the three payloads of finding 4 to `test_payload`. Fixes 4. No trade-off.
2. **Parse the payload on the thread of the goal.** `TreeExecutionServer` stores the goal's own payload before it calls `onTreeCreated`: parse `goalPayload()` there, and let `onGoalReceived` only check that it parses. `payload_` is then no longer shared. Fixes 2. The payload is parsed twice.
3. **Count the goals, not the trees.** Give each accepted goal a number in `onGoalReceived`, and have the loop end any run whose number is lower than the last accepted one, instead of a flag that `onTreeCreated` clears and a `running_` that is false while a tree is created. A goal in the gap then preempts the tree at its first tick. Fixes the first half of 1, and 14 if `~/objective` is published at acceptance. The second half, the unanswered cancel, lasts only as long as the creation of the tree.
4. **Do not reload while a tree is being created.** With the count of recommendation 3, `onGoalReceived` knows that a goal is accepted and its tree not yet created: wait for `onTreeCreated` on a condition variable before `reloadTrees()`. Fixes 3. A goal in the gap is answered a few milliseconds later.
5. **Read each file once, and only when its stamp changed.** Compare time and size first; read a file, for `<BehaviorTree` and for its main tree, only when it is new or changed, and take `main_tree_to_execute` with tinyxml2, which BehaviorTree.CPP already brings, instead of `std::regex`. Fixes 9 and 13. A new dependency to declare.
6. **Make `preempt` and `execution_period` read-only**, with a `ParameterDescriptor`. Fixes the misleading half of 7. In the fork, register again only when `plugins` or `behavior_trees` changed, which they cannot, being read-only: the second registration, and the rule it imposes on every plugin, then go. Fixes 7.
7. **In the fork: end a run at shutdown as at a cancel, and join.** Halt the tree when the loop leaves on `!rclcpp::ok()`, do not publish a result on a dead context, and join the thread in the destructor. Fixes 5, and lets the tests destroy their servers. A second commit to carry until upstream takes it.
8. **In the fork: do not join in `handle_accepted`.** Start the new thread at once and let it join the previous one before it creates its tree, or give the action server a callback group of its own. Fixes 8 and the rest of 1.
9. **In the fork: open the folders with an error code, and compare with `npos`.** Fixes 6.
10. **Say how a run ended, in one new field.** Add `outcome` to the last snapshot, `succeeded`, `failed`, `preempted`, `cancelled` or `threw`, from what `CommanderServer` already knows (`preempted_`, `was_cancelled`, the status), keep `cancelled` as it is for the clients that read it, and make `TheLastSnapshotTellsHowTheRunEnded` build the endings the server sends. Then the editor reads `outcome`. Fixes 10, but for the text of the exception, which the hook does not receive.
11. **Decide the edges of the payload.** Refuse a number that is not finite, a key that is null or repeated, and a payload over a size; honour `!!str`. Fixes 11. An objective that wants a string named `nan` must quote it.
12. **Empty the global blackboard when a run ends**, in `onTreeExecutionCompleted`, or document that `@` entries are for ever. Fixes 12. A robot that passes values from one objective to the next through it loses them; StepIt Macro keeps such values in `stack_state` by its own rule.
13. **A fixture for the real server with test nodes**, as the probes had: a subclass that registers a node slow to construct and one whose halt throws, waiting on `~/objective` instead of sleeping. Give it the cases of [Testability and Tests](#testability-and-tests): a goal in the gap, a cancel, the three rejections, a payload reaching the tree and being unset, a parameter set.
14. **Correct the documents** of the table above, in this repository and in StepIt Macro.
