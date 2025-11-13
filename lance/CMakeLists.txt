cmake_minimum_required(VERSION 3.12)
project(lance)

# --- Dependencies -------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(phoenix_ros_driver REQUIRED)
find_package(cardinal_perception REQUIRED)
find_package(csm_metrics REQUIRED)

find_package(Eigen3 REQUIRED)

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
add_executable(robot_control
    src/control_node.cpp
    src/robot/robot_params.cpp
    src/robot/collection_state.cpp
    src/robot/robot_controller.cpp
    src/robot/controllers/auto_controller.cpp
    src/robot/controllers/mining_controller.cpp
    src/robot/controllers/teleop_controller.cpp
    src/robot/controllers/offload_controller.cpp
    src/robot/controllers/traversal_controller.cpp
    src/robot/controllers/auto_mining_controller.cpp
    src/robot/controllers/auto_offload_controller.cpp
    src/robot/controllers/localization_controller.cpp)
target_link_libraries(robot_control
    csm_metrics::profiling
    Eigen3::Eigen)
ament_target_dependencies(robot_control
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    phoenix_ros_driver
    cardinal_perception
    csm_metrics)

add_executable(robot_status src/watchdog_node.cpp)
target_link_libraries(robot_status "${lance_custom_types}")
ament_target_dependencies(robot_status
    rclcpp
    std_msgs)

# --- Export -------------------------------------------------------------------
ament_export_dependencies(
    rosidl_default_runtime
    rclcpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    phoenix_ros_driver
    cardinal_perception
    csm_metrics)
ament_package()

install(TARGETS
    robot_control
    robot_status
    DESTINATION lib/${PROJECT_NAME} )
install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME} )
