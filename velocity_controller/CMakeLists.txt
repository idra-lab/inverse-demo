cmake_minimum_required(VERSION 3.16)
project(velocity_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(inverse_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cartesian_position_controller.cpp
  src/robot_kinematics.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  controller_interface geometry_msgs hardware_interface inverse_msgs
  kdl_parser orocos_kdl pluginlib rcl_interfaces rclcpp rclcpp_lifecycle
  realtime_tools std_msgs urdf
)

pluginlib_export_plugin_description_file(controller_interface velocity_controller_plugins.xml)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sine_velocity_test.py DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface Eigen3 geometry_msgs hardware_interface inverse_msgs
  kdl_parser orocos_kdl pluginlib rcl_interfaces rclcpp rclcpp_lifecycle
  realtime_tools std_msgs urdf
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME})
  ament_add_gtest(test_controller_init test/test_controller_init.cpp)
  target_link_libraries(test_controller_init ${PROJECT_NAME})
endif()
ament_package()
