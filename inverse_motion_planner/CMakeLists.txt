cmake_minimum_required(VERSION 3.10)

project(inverse_motion_planner)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

macro(find_and_log_required package)
    find_package(${package} REQUIRED)
    message(STATUS "Package ${package} found at ${${package}_DIR}")
endmacro()

macro(find_and_log package)
    find_package(${package})
    if(${package}_FOUND)
        message(STATUS "Package ${package} found at ${${package}_DIR}")
    else()
        message(STATUS "Package ${package} not found!")
    endif()
endmacro()

find_and_log_required(ament_cmake)
find_and_log_required(ament_cmake_python)
find_and_log_required(Eigen3)
find_and_log_required(fmt)
find_and_log_required(mdvcpplib)
find_and_log_required(nlohmann_json)
find_and_log_required(rclcpp)
find_and_log_required(std_srvs)
find_and_log_required(geometry_msgs)
find_and_log_required(inverse_msgs)
find_and_log_required(tf2_ros)

add_library(
    motion_planner_commons
    src/motion_planner.cpp
    src/components/motion.cpp
    src/components/motion_queue.cpp
    src/components/path_profiler.cpp
    src/components/interpolation.cpp
    src/components/skill_database.cpp
    src/motions/dmp_motion_interface.cpp
    src/motions/discrete_dmp_motion.cpp
    src/motions/hold_position.cpp
    src/motions/rhytmic_dmp_motion.cpp
    src/motions/raw_trajectory.cpp
    src/motions/peg_in_hole.cpp
    src/ros2/ros2_motion_parameters.cpp
    src/ros2/ros2_robot_system.cpp
)
target_include_directories(
    motion_planner_commons
    PUBLIC
    include
)
target_link_libraries(
    motion_planner_commons
    PUBLIC
    Eigen3::Eigen
    mdvcpplib::mdvcpplib
    nlohmann_json::nlohmann_json
)
ament_target_dependencies(
    motion_planner_commons
    PUBLIC
    rclcpp
    geometry_msgs
    tf2_ros
    inverse_msgs
    std_srvs
)

add_executable(
    motion_planner
    src/ros2_motion_planner.cpp
)
target_link_libraries(
    motion_planner
    motion_planner_commons
)

add_executable(
    skill_learner
    src/skill_learner.cpp
)
target_link_libraries(
    skill_learner
    motion_planner_commons
)

install(
    TARGETS motion_planner skill_learner
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
