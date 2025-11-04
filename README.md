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
