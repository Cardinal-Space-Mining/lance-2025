cmake_minimum_required(VERSION 3.12)
project(lance)

# --- Dependencies -------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(phoenix_ros_driver REQUIRED)

# --- Presets ------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(NOT MSVC)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# --- Custom Types -------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetRobotMode.srv")

rosidl_get_typesupport_target(lance_custom_types
    ${PROJECT_NAME} rosidl_typesupport_cpp)

# --- Executables --------------------------------------------------------------
add_executable(teleop_control
    src/teleop_control/main.cpp
    src/teleop_control/controller.cpp
    src/teleop_control/collection_state.cpp)
ament_target_dependencies(teleop_control
    rclcpp
    sensor_msgs
    std_msgs
    phoenix_ros_driver)

add_executable(robot_status src/robot_status.cpp)
target_link_libraries(robot_status "${lance_custom_types}")
ament_target_dependencies(robot_status
    rclcpp
    std_msgs)

# --- Export -------------------------------------------------------------------
ament_export_dependencies(
    rclcpp
    rosidl_default_runtime
    std_msgs
    sensor_msgs
    phoenix_ros_driver)
ament_package()

install(TARGETS
    teleop_control
    robot_status
    DESTINATION lib/${PROJECT_NAME} )
install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME} )
