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
