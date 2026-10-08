# inverse_spacemouse

Reads a 3Dconnexion SpaceMouse directly over USB (Linux `/dev/hidraw*`, standard
library only, no extra Python packages) and publishes, at 200 Hz:

- `spacemouse/command` (`inverse_msgs/CartesianTrajectoryPoint`): only `velocity` is
  filled (base_link frame, scaled by `max_linear` / `max_angular`); `time_from_start` is
  the current ROS time; `pose` is the identity. **Use only with `command_mode: velocity`**
  (`cartesian_velocity_controller`); in position modes the controller would track the
  identity pose.
- `spacemouse/joy` (`sensor_msgs/Joy`): axes in [-1, 1] plus buttons.

The device is auto-detected. Until it is plugged in, and after it is unplugged, the node
publishes zero velocity and retries every second.

## Setup on the machine the SpaceMouse is plugged into (once)

```bash
cd ~/ros2_ws && colcon build --packages-select inverse_msgs inverse_spacemouse
source install/setup.bash
# read access for non-root users, then replug the SpaceMouse
sudo cp $(ros2 pkg prefix inverse_spacemouse)/share/inverse_spacemouse/udev/99-spacemouse.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

## Run

```bash
ros2 launch inverse_spacemouse spacemouse.launch.py \
  command_topic:=/cartesian_velocity_controller/commands
```

Without `command_topic` it publishes on `/spacemouse/command`, which is useful for checking
axis directions with `ros2 topic echo` before connecting the controller. Push forward
should give `+linear.x`, left `+linear.y`, up `+linear.z`; flip entries in `axis_sign`
(`config/spacemouse.yaml`) if not. Pass a different config with `params_file:=...`.

## Troubleshooting

- `no SpaceMouse found`: check `cat /sys/class/hidraw/*/device/uevent | grep HID_NAME`
  lists it, or set `device: /dev/hidrawN` in the config.
- `no permission to read /dev/hidrawN`: the udev rule is missing; install it and replug.
