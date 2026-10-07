# velocity_controller

ROS 2 Humble `ros2_control` plugin wrapping the supplied Cartesian controller.
The controller classes and helpers use the global namespace; the original
control/queueing logic is retained.
The missing kinematics dependency is supplied locally using Orocos KDL and
Eigen, including URDF velocity limits, base-frame geometric Jacobians,
variable-damping pseudoinversion, and quaternion utilities. This is a new
implementation of that dependency, not the unavailable original helper.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/inverse-demo/velocity_controller src/inverse-demo/inverse_msgs --ignore-src -r -y
colcon build --packages-select inverse_msgs velocity_controller
source install/setup.bash
```

## Load

Start the robot's existing controller manager and `robot_state_publisher` first.
The hardware must export each configured joint's `velocity` command interface
and `position` and `velocity` state interfaces. The supplied configuration is
for a six-joint UR arm, with `base_link` and `tool0` as chain endpoints.
Stop any other controller claiming these joints' velocity interfaces before
activating this controller.

```bash
ros2 launch velocity_controller controller.launch.py
```

Launch arguments:

- `controller`: defaults to `cartesian_velocity_controller`; also supports
  `cartesian_position_controller` and `cartesian_position_velocity_controller`.
- `controller_manager`: defaults to `/controller_manager`.
- `params_file`: defaults to the installed `config/controllers.yaml`.

The spawner loads and activates one controller in an existing manager. To
configure the manager itself (including its 500 Hz update rate and simulation
clock), pass `config/controllers.yaml` to the robot's controller-manager launch.
For hardware, use a configuration matching the robot's clock and joint names.
`robot_description_source` defaults to `robot_state_publisher`; its
`robot_description` parameter is queried during configuration.

Plugin identifier: `velocity_controller/CartesianPositionController`.

## Commands

Topic: `/<controller>/commands`

Type: `inverse_msgs/msg/CartesianTrajectoryPoint`

```text
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration
builtin_interfaces/Duration time_from_start
```

Pose and twist are expressed in the configured base frame, at the configured
tip origin. Acceleration is unused. Use a unit quaternion for orientation.
The input is the custom trajectory-point message, not a plain `Twist`.

The original behavior is preserved: velocity and position/velocity modes need
two consecutive samples with increasing `time_from_start`; publish continuously
faster than the configured command timeout (0.1 s by default). Repeating an
unchanged timestamp does not start normal playback. Position mode uses the
latest pose. See the controller header for the original interpolation details.

Diagnostics are published under `~`: `interpolated_pose`, `interpolated_twist`,
and `jacobian_determinant`.

## inverse_bringup integration

Robot-specific settings are also registered in
`inverse_bringup/config/ur_controllers.yaml`, preserving its 125 Hz manager rate
and clock settings. This configuration uses a 10% URDF joint-velocity limit,
speed scaling, and the prefixed `base_link` / `tool0` chain. Unlike the existing
motion controller, it does not command the `target_link` TCP.

```bash
ros2 launch inverse_bringup inverse_ur.launch.py use_fake_hardware:=true ctrl:=cartesian_velocity_controller
```

The existing default controller is unchanged. When selected through bringup,
the velocity controller is loaded and configured **inactive**. After checking
the hardware interfaces, frame convention, and command publisher, activate it
explicitly (deactivating any active arm-motion controllers first):

```bash
ros2 control set_controller_state cartesian_velocity_controller active
```

The command topic remains `/cartesian_velocity_controller/commands`, using
`inverse_msgs/msg/CartesianTrajectoryPoint` with advancing trajectory times.
Fake-hardware checks do not establish hardware safety; the original controller
limitations remain unchanged. The standalone package launch described above
still activates its selected controller directly.
