# Inverse Motion Planner

Implementation of a motion planner to be used within the context of the INVERSE framework.

## Parameters

- `base_link`: base link w.r.t. which poses are broadcasted;
- `ee_link`: end-effector link whose motion shall be controlled;
- `frame_topic_name`: name of topic in which the the reference shall be broadcasted onto;

## Services

- `/set_broadcast_state` (type `std_srvs/srv/SetBool`): set wether the reference computed by the planner shall be broadcasted to the low-level controller or not;
- `/safe_stop` (type `std_srvs/srv/Trigger`): triggers the immediate stopping of the robot;
- `/reach_position` (type `magician_msgs/srv/ReachPosition`): service that commands the robot to reach a given position;
- `/execute_ptp_motion` (type `magician_msgs/srv/PointToPointMotion`): service that performs a point-to-point motion within two arbitrary points;
- `/pick` (type `magician_msgs/srv/EnqueueTrigger`): service that enqueues a motion to close the gripper;
- `/place` (type `magician_msgs/srv/EnqueueTrigger`): service that enqueues a motion to open the gripper;


## Utility scripts

### Open/close gripper

1. Ensure the planner is enabled:
   ```bash
   ros2 service call /set_broadcast_state std_srvs/srv/SetBool "data: true"
   ```
1. Call the corresponding service (pick in this case):
   ```bash
   ros2 service call /pick inverse_msgs/srv/EnqueueTrigger "{}"
   ```
1. (optional) Disable the planner broadcasting state:
   ```bash
   ros2 service call /set_broadcast_state std_srvs/srv/SetBool "data: false"
   ```

### Move to a TF frame

To avoid fully specifying the `/reach_position` service call from the CLI, this package expose the `reach_frame` utility script.
For example:

- To go to `homing` reference frame with default speed of 0.05m/s speed
  ```bash
  ros2 run inverse_motion_planner reach_frame homing
  ```
- Go to `homing` with a specified speed (in m/s)
  ```bash
  ros2 run inverse_motion_planner reach_frame bus_bar --velocity 0.08
  ```

**Notes**:

- The script internally starts a ROS 2 node that listens for TF trees. If the reference frame is not found, try to increase the discovery time by specifying a `--timeout DURATION_S` option (defaults to 2s).
- The script is *smart*: for example, if `front_connector` does not exists as a standalone frame, but `via(front_connector)` and `obs(front_connector)` both exists, then the planner will enqueue 2 separate motions:
  ```
  current config -> via(desired_frame) -> obs(desired_frame)
  ```

## Queued gripper primitives

`/motion_planner/pick` (close) and `/motion_planner/place` (open) use
`inverse_msgs/srv/EnqueueTrigger`: an empty request and a response containing
`bool success`, `string message`, and `uint64[] motion_ids`. Success means the
primitive was **enqueued**, not that the grasp/release has completed. An
unavailable action server rejects the request without allocating a motion ID.

```bash
ros2 service call /motion_planner/pick inverse_msgs/srv/EnqueueTrigger '{}'
ros2 service call /motion_planner/place inverse_msgs/srv/EnqueueTrigger '{}'
```

The Python wrappers also support `node.pick().enqueue()` and
`node.place().enqueue()`. These are gripper-only operations; enqueue approach,
transport, and retreat motions separately. Reference broadcasting must be enabled
for the queue to execute, as with other primitives.

Each primitive starts its action only when it becomes active. It holds the arm's
incoming reference pose with zero reference twist while waiting. Successful
completion publishes the primitive ID on `/motion_end` and permits the next
queued motion to start (`/motion_start` reports its start).

Parameters (defaults match the orchestrator's open/close commands):

| Parameter | Default |
| --- | --- |
| `gripper_action_name` | `/robotiq_gripper_controller/gripper_cmd` |
| `gripper_open_position` | `0.47` |
| `gripper_closed_position` | `0.7` |
| `gripper_max_effort` | `100.0` |
| `gripper_timeout_sec` | `10.0` |
| `gripper_pick_allow_stalled` | `true` |

Only an action result with `SUCCEEDED` and `reached_goal` completes a place.
A pick also accepts a `SUCCEEDED` result with `stalled` when
`gripper_pick_allow_stalled` is enabled, allowing closure against an object.
This does not independently verify object presence. Rejection, abort,
cancellation, disappearance of the action server, or timeout holds the pose and
blocks the queue, logging an error without publishing successful completion.
Use `safe_stop` to clear the queue before recovery. Stopping/replacing an active
primitive requests action cancellation, including goals accepted after the stop.
Cancellation is best-effort, not a hardware emergency stop.
