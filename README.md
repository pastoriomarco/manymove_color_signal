# manymove_color_signal

Simple ROS 2 package that:

- Publishes `signal_column_msgs/SignalColor` messages to `/signal_column` (demo node)
- Provides a BehaviorTree-based UR client executable that mirrors Manymove's `bt_client_ur`,
  plus a new BT node `PublishSignalColorAction` used to emit a green signal at startup.

Build:

- `colcon build --packages-select signal_column_msgs manymove_color_signal`
- `. install/setup.bash`

Run publisher:

- `ros2 run manymove_color_signal color_signal_demo`

Inspect topic:

- `ros2 topic echo /signal_column signal_column_msgs/msg/SignalColor`

Run UR BT client variant (expects the standard Manymove bringup stack running):

- Start your UR bringup (see `manymove_bringup/launch/ur_movegroup_fake_cpp_trees.launch.py`).
  You can use that file as a reference to prepare the robot, MoveIt, planners, and object manager.
- Then start this client only:
  - `ros2 launch manymove_color_signal bt_client_ur_color_signal.launch.py robot_model:=ur3e tcp_frame:=tool0 is_robot_real:=false`

Notes
- The new BT node `PublishSignalColorAction` is registered locally in this package, so
  `manymove_cpp_trees` does not depend on `signal_column_msgs`.
- The executable `bt_client_ur_color_signal` reuses the full Manymove C++ tree library and logic
  via `manymove_cpp_trees`, registering the extra node and injecting it into the startup sequence.

## Docker Overlay

Run the helper script to build (or refresh) the base ManyMove image plus a color-signal overlay,
then drop into an interactive container:

- `./src/manymove_color_signal/docker/run_manymove_color_signal_container.sh jazzy`

Useful flags:

- `--pull-latest` updates the upstream ManyMove repo before rebuilding the base image.
- `--force-rebuild` rebuilds both base and overlay images even if nothing appears to have changed.
- `--build-only` skips launching the container after the images are up to date.

Pass any additional `docker run` options after `--`, for example:

- `./src/manymove_color_signal/docker/run_manymove_color_signal_container.sh jazzy -- --volume "$(pwd)":/host_ws`

Need a one-shot bootstrap that fetches the repos and kicks everything off?

```bash
export MANYMOVE_ROS_WS=~/workspaces/dev_ws && \
export MANYMOVE_NO_GPU=1 && \
bash ${MANYMOVE_ROS_WS}/src/manymove_color_signal/docker/bootstrap_color_signal_workspace.sh jazzy --workspace ${MANYMOVE_ROS_WS}
```

- `./src/manymove_color_signal/docker/bootstrap_color_signal_workspace.sh jazzy`

The bootstrapper creates the workspace (`~/workspaces/dev_ws` by default), clones `manymove`,
`manymove_color_signal`, and `signal_column_msgs`, then hands off to the container helper. You can
override the workspace path (`--workspace /tmp/manymove_ws`), switch branches or repo URLs, reuse the
runner flags like `--pull-latest`, and pass extra docker options after `--` just as with the direct runner.
Prefer the positional `humble|jazzy` argument to match the other run scripts, but `--ros-distro` is still
accepted for backwards compatibility (both cannot conflict).

### Throttle build workers

Set `MANYMOVE_COLCON_WORKERS` (positive integer) before calling either the bootstrapper or container runner to cap
how many packages `colcon` builds in parallel inside the Docker images and during any subsequent
`/opt/manymove/setup_workspace.sh` runs:

```bash
export MANYMOVE_COLCON_WORKERS=1
./src/manymove_color_signal/docker/run_manymove_color_signal_container.sh jazzy
```

Leave the variable unset (or empty) to revert to the default concurrency.
