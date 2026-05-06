
cmake_minimum_required(VERSION 3.8)
project(rdsim_localization)


if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_localization REQUIRED)










add_executable(initialpose_to_setpose src/initialpose_to_setpose.cpp)

ament_target_dependencies(initialpose_to_setpose
  rclcpp
  geometry_msgs
  robot_localization
)

install(
  TARGETS initialpose_to_setpose
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)




  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
