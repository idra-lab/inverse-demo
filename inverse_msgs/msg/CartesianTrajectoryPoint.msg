# Cartesian quantities expressed in the controller's configured base_link frame.
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration
# Publisher-assigned trajectory time; must advance between consecutive samples
# in velocity and position_velocity modes. Acceleration is currently unused.
builtin_interfaces/Duration time_from_start
