cmake_minimum_required(VERSION 3.10)
project(inverse_msgs)

# Build custom msgs
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
# "msg/YourMsg.msg"
# DEPENDENCIES std_msgs
# )
ament_export_dependencies(rosidl_default_runtime)
ament_package()
