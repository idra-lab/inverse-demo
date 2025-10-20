cmake_minimum_required(VERSION 3.8)
project(inverse_resources)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)


ament_python_install_package(${PROJECT_NAME})
ament_package()
