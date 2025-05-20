cmake_minimum_required(VERSION 3.12)
project(lance)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT MSVC)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "srv/SetRobotMode.srv")

rosidl_get_typesupport_target(
    lance_custom_types
    ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(robot_status src/robot_status.cpp)
target_link_libraries(robot_status "${lance_custom_types}")
ament_target_dependencies(robot_status rclcpp std_msgs)

ament_export_dependencies(rclcpp std_msgs rosidl_default_runtime)
ament_package()

install(
    TARGETS robot_status
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)
