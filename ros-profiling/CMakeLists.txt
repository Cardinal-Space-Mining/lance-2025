cmake_minimum_required(VERSION 3.15)
project(ros_profiling)

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
# find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ProcessMetrics.msg"
    "msg/ThreadStatus.msg"
    "msg/TraceNotification.msg"
    "msg/TraceNotifications.msg"
    DEPENDENCIES builtin_interfaces )

# rosidl_get_typesupport_target(${PROJECT_NAME}_custom_types
#     ${PROJECT_NAME} rosidl_typesupport_cpp )

set(LIB_NAME ${PROJECT_NAME}_cpp)

add_library(${LIB_NAME} INTERFACE)
# target_link_libraries(${LIB_NAME}
#     ${${PROJECT_NAME}_custom_types} )
target_include_directories(${LIB_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>" )
ament_target_dependencies(${LIB_NAME})

install(TARGETS ${LIB_NAME}
    EXPORT "export_${LIB_NAME}"
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include )
install(DIRECTORY include/
    DESTINATION include )

install(PROGRAMS "src/profiling_manager.py"
    DESTINATION lib/${PROJECT_NAME} )

ament_export_targets("export_${LIB_NAME}")
# ament_export_include_directories(include)   # if exporting linkable libraries, use the commented out configs instead
ament_export_dependencies(
    rclpy
    rosidl_default_runtime
    builtin_interfaces )
ament_package()
