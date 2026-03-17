cmake_minimum_required(VERSION 3.10)
project(smpl_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)


# Eigen
find_package(Eigen3 REQUIRED)
# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(smpl_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# ZED SMPL Tracking
find_package(ZED REQUIRED)
find_package(CUDA REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
SET(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(OpenCV 4.11)

set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})


# Executable
add_executable(zed_smpl_tracking
  src/zed_smpl_tracking/zed_smpl_tracking.cpp
  src/zed_smpl_tracking/ClientPublisher.cpp
)


# ZED SMPL Tracking
target_include_directories(zed_smpl_tracking PUBLIC ${CUDA_INCLUDE_DIRS})
target_include_directories(zed_smpl_tracking PUBLIC ${ZED_INCLUDE_DIRS})
target_include_directories(zed_smpl_tracking PUBLIC ${GLEW_INCLUDE_DIRS})
target_include_directories(zed_smpl_tracking PUBLIC ${GLUT_INCLUDE_DIR})
target_include_directories(zed_smpl_tracking PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(zed_smpl_tracking PUBLIC ${OpenCV_INCLUDE_DIRS})
link_directories(zed_smpl_tracking ${ZED_LIBRARY_DIR})
link_directories(zed_smpl_tracking ${CUDA_LIBRARY_DIRS})
link_directories(zed_smpl_tracking ${GLEW_LIBRARY_DIRS})
link_directories(zed_smpl_tracking ${GLUT_LIBRARY_DIRS})
link_directories(zed_smpl_tracking ${OpenGL_LIBRARY_DIRS})
link_directories(zed_smpl_tracking ${OpenCV_LIBRARY_DIRS})
target_link_libraries(zed_smpl_tracking ${ZED_LIBS} ${UTILS_LIB} ${SPECIAL_OS_LIBS} ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${GLEW_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(zed_smpl_tracking
  rclcpp
  smpl_msgs
  sensor_msgs
  tf2_ros
  cv_bridge
)

# Install
install(TARGETS zed_smpl_tracking 
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
