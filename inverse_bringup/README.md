# Inverse bringup package

## Registering transforms

1. Start the robot
   ```bash
   ros2 launch inverse_bringup inverse_ur.launch.py
   ```
2. Optional: close the gripper:
   ```bash
   ros2 service call /pick inverse_msgs/srv/EnqueueTrigger "{}"
   ```
3. From the UR teach pendant, block communication with computer so that the robot can be moved in free-drive;
4. Move the robot to the desired configuration;
5. Run the command
   ```bash
   ros2 run tf2_ros tf2_echo "base_link" "target_link"
   ```
6. In the `publish_poses` function of `launch/inverse_ur.launch.py`, append the data from the `tf2_echo` call:
   ```python
   frames = {
        # ...
        "<frame_name>": (
            [0.0, 0.0, 0.0] ,  # Fill with translation data
            [1.0, 0.0, 0.0, 0.0] ,  # Fill with quaternion data
        ),
        # ...
   }
   ```

