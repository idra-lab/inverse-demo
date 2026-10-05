# Magician Motion Planner

Implementation of a motion planner to be used within the context of the MAGICIAN framework.

docs here https://github.com/magician-project/magician_documentation/blob/main/tests/motion-planner.md

## Parameters

- `base_link`: base link w.r.t. which poses are broadcasted;
- `ee_link`: end-effector link whose motion shall be controlled;
- `ft_link`: link where the force-torque sensor data are displayed into; necessary only if using the _fake_ admittance control;
- `ft_topic`: ROS2 topic name with type `geometry_msgs/msg/WrenchStamped` where the force torque data are retrieved;
- `proportional_gain` and `integral_gain`: while using admittance control, the displacement $\delta z(t)$ along the $z$ axis of the end-effector is (roughly) given by the law 
  $$ \delta z(t) = k_p \big(f_{des}(t) - f_{meas}(t)\big) + k_i \int_0^t k_p \big(f_{des}(\tau) - f_{meas}(\tau)\big) \, dt $$
  In this equation $k_p$ is the `proportional_gain`, while $k_i$ is the `integral_gain`;
- `integral_bound` and `integral_velocity_bound`: actually, the integrator of the above equation has two bounds: one on the maximum displacement $\delta z_i$ that the integrator can generate (`integral_bound` in _m_), and one on its maximum rate of change $\dot{\delta z_i}$ (`integral_velocity_bound` in _m/s_);


## Services

- `/motion_planner/set_broadcast_state` (type `std_srvs/srv/SetBool`): set wether the reference computed by the planner shall be broadcasted to the low-level controller or not;
- `motion_planner/safe_stop` (type `std_srvs/srv/Trigger`): triggers the immediate stopping of the robot;
- `/motion_planner/reach_position` (type `magician_msgs/srv/ReachPosition`): service that commands the robot to reach a given position;
- `/motion_planner/execute_ptp_motion` (type `magician_msgs/srv/PointToPointMotion`): service that performs a point-to-point motion within two arbitrary points;
- `/motion_planner/execute_mesh_ptp_motion` (type `magician_msgs/srv/MeshPointToPointMotion`): service that performs a point-to-point motion within two arbitrary points which are projected on a specified mesh;
- `/motion_planner/hold_position` (type `magician_msgs/srv/HoldPosition`): a simple motion primitives that stays in a point for a given amount of seconds;
- `/motion_planner/ptp_time_estimate` (type `magician_msgs/srv/PointToPointTime`): provides the time-transition matrix for reaching some poses;

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
