cmake_minimum_required(VERSION 3.15)
project(csm_metrics)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rosidl_generator_py REQUIRED)

find_package(ament_cmake)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)


# --- Messages -----------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/FloatStamped.msg"
    "msg/LabelStamped.msg"
    "msg/ProcessStats.msg"
    "msg/TraceNotification.msg"
    "msg/TraceNotifications.msg"
    DEPENDENCIES builtin_interfaces )

rosidl_get_typesupport_target(${PROJECT_NAME}_custom_types
    ${PROJECT_NAME} rosidl_typesupport_cpp )

# --- Stats --------------------------------------------------------------------
add_library(stats STATIC
    "src/cpp/stats.cpp" )
target_include_directories(stats PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>" )
ament_target_dependencies(stats)

file(WRITE ${CMAKE_BINARY_DIR}/test_sensors.cpp "#include <sensors/sensors.h>")
try_compile(HAS_SENSORS ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_sensors.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_sensors.cpp)
if(${HAS_SENSORS})
    target_compile_definitions(stats
        PUBLIC HAS_SENSORS)
    target_link_libraries(stats sensors)
    message("Sensors support found.")
else()
    message("No sensors support.")
endif()

file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
    target_compile_definitions(stats
        PUBLIC HAS_CPUID)
    message("CPUID support detected!")
else()
    message("No CPUID support detected.")
endif()

# --- Profiling ----------------------------------------------------------------
add_library(profiling STATIC
    "src/cpp/profiling.cpp" )
target_link_libraries(profiling
    ${${PROJECT_NAME}_custom_types} )
target_include_directories(profiling PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>" )
ament_target_dependencies(profiling
    rclcpp )


# --- Install ------------------------------------------------------------------
install(TARGETS stats
    EXPORT "export_stats"
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include )
install(TARGETS profiling
    EXPORT "export_profiling"
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include )

install(DIRECTORY include/
    DESTINATION include )

install(PROGRAMS "src/profiling_manager.py"
    DESTINATION lib/${PROJECT_NAME} )

ament_export_targets(
    "export_stats"
    "export_profiling" )
ament_export_dependencies(
    rclpy
    rclcpp
    rosidl_default_runtime
    builtin_interfaces )
ament_package()
