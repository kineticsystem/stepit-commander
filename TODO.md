# TODO

Open questions and concerns about the server, written down as they came up:
decisions we deferred, and what we knew when deferring them.

## Tooling and operations

### 1. The hooks need the container

The three ament linters come from the ROS workspace, so `git commit` on the host
fails unless they are skipped:

```bash
SKIP=ament_copyright,ament_lint_cmake,ament_cpplint git commit ...
```

Committing from inside the container is the intended path: it has `pre-commit`,
`clang-format` and ROS. `pre-commit install` has not been run in either place.

### 2. No CI

StepIt Driver has three GitHub Actions workflows (industrial_ci, format,
ros-lint). This repository has none, so nothing checks a pull request. The hooks and
`./bin/test.sh` already define what CI would have to run.

### 3. Two servers collide on port 1667

Running a second `stepit_server` makes every goal fail with
`Behavior Tree exception: Address already in use`: BehaviorTree.ROS2 publishes
the state of the running tree on port 1667, and both servers try to. Worth
knowing before debugging the tree itself.

### 4. The submodule tracks a branch

`modules/BehaviorTree.ROS2` is pinned to a commit, as git always does, but
`.gitmodules` names the branch `humble`, so `git submodule update --remote`
would move it. There is no released Debian package to depend on instead.

### 5. The Dockerfile carries the container passwords

`developer:developer` and `root:docker` are in `docker/Dockerfile`, inherited
from the StepIt template. Intentional for a development container, but visible
to anyone who can read the repository.
