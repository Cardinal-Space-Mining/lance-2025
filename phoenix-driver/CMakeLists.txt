cmake_minimum_required(VERSION 3.8)
project(phoenix_ros_driver)

# --- Dependencies -------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(phoenix6 REQUIRED)

# --- Presets ------------------------------------------------------------------
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(NOT MSVC)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")


# --- Generate Message Types ---------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    msg/TalonCtrl.msg
    msg/TalonInfo.msg
    msg/TalonFaults.msg
    DEPENDENCIES std_msgs)

rosidl_get_typesupport_target(phoenix_ros_driver_custom_types
    ${PROJECT_NAME} rosidl_typesupport_cpp)

# --- Phoenix 5 Driver ---------------------------------------------------------
set(PHX5_EXAMPLE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/Phoenix5-Linux-Example)
if(${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf")    # arm32
    set(PHX5_LIB_DIR ${PHX5_EXAMPLE_DIR}/lib/arm32)
elseif(${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")  # aarch64
    set(PHX5_LIB_DIR ${PHX5_EXAMPLE_DIR}/lib/arm64)
else()
    set(PHX5_LIB_DIR ${PHX5_EXAMPLE_DIR}/lib/x86-64)
endif()

add_executable(phx5_driver src/phx5_driver.cpp)
target_include_directories(phx5_driver PUBLIC
    $<BUILD_INTERFACE:${PHX5_EXAMPLE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(phx5_driver
    ${phoenix_ros_driver_custom_types}
    ${PHX5_LIB_DIR}/libCTRE_Phoenix.so
    ${PHX5_LIB_DIR}/libCTRE_PhoenixCCI.so
    ${PHX5_LIB_DIR}/libCTRE_PhoenixTools.so)
ament_target_dependencies(phx5_driver
    rclcpp
    std_msgs)
target_compile_features(phx5_driver PUBLIC c_std_99 cxx_std_20)     # not necessary but silences a lot of warnings

# --- Phoenix 6 Driver ---------------------------------------------------------
add_executable(phx6_driver src/phx6_driver.cpp)
target_link_libraries(phx6_driver
    ${phoenix_ros_driver_custom_types})
ament_target_dependencies(phx6_driver
    rclcpp
    std_msgs
    phoenix6)
target_compile_features(phx6_driver PUBLIC c_std_99 cxx_std_20)     # not necessary but silences a lot of warnings

# --- Motor Sim Node -----------------------------------------------------------
add_executable(motor_sim src/motor_sim.cpp)
target_link_libraries(motor_sim
    ${phoenix_ros_driver_custom_types})
ament_target_dependencies(motor_sim
    rclcpp
    nav_msgs
    std_msgs
    sensor_msgs
    geometry_msgs)

# --- Export -------------------------------------------------------------------
ament_export_dependencies(
    rclcpp
    nav_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    rosidl_default_runtime)
ament_package()

install(TARGETS
    phx5_driver
    phx6_driver
    motor_sim
    DESTINATION lib/${PROJECT_NAME} )
install(DIRECTORY
    launch
    scripts
    DESTINATION share/${PROJECT_NAME} )

install(CODE
    "execute_process(COMMAND cp \
    ${PHX5_LIB_DIR}/libCTRE_Phoenix.so \
    ${PHX5_LIB_DIR}/libCTRE_PhoenixCCI.so \
    ${PHX5_LIB_DIR}/libCTRE_PhoenixTools.so \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/ )" )
install(CODE
    "execute_process(COMMAND patchelf --set-rpath \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME} \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libCTRE_Phoenix.so )" )
install(CODE
    "execute_process(COMMAND patchelf --set-rpath \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME} \
    ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/phx5_driver )" )
