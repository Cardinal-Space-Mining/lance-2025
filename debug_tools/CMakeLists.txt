cmake_minimum_required(VERSION 3.8)
project(debug_tools)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(include)
include_directories(SYSTEM)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AccuracyAnalysis.msg"
  DEPENDENCIES std_msgs geometry_msgs)

rosidl_get_typesupport_target(debug_tools_custom_types
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(scan_transformer "src/scan_transformer.cpp")
target_include_directories(scan_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(scan_transformer
  ${PCL_LIBRARIES}
  Eigen3::Eigen)
ament_target_dependencies(scan_transformer
  "rclcpp"
  "pcl_ros"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_sensor_msgs"
  "tf2_geometry_msgs")

install(TARGETS scan_transformer
  DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------

add_executable(imu_transformer "src/imu_transformer.cpp")
target_include_directories(imu_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(imu_transformer
  Eigen3::Eigen)
ament_target_dependencies(imu_transformer
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_sensor_msgs"
  "tf2_geometry_msgs")

install(TARGETS imu_transformer
  DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------

add_executable(imu_visualizer "src/imu_visualizer.cpp")
target_include_directories(imu_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(imu_visualizer
  Eigen3::Eigen)
ament_target_dependencies(imu_visualizer
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_sensor_msgs"
  "tf2_geometry_msgs")

install(TARGETS imu_visualizer
  DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------

add_executable(accuracy_analyzer "src/accuracy_analyzer.cpp")
target_include_directories(accuracy_analyzer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(accuracy_analyzer
  Eigen3::Eigen
  "${debug_tools_custom_types}")
ament_target_dependencies(accuracy_analyzer
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_sensor_msgs"
  "tf2_geometry_msgs")

install(TARGETS accuracy_analyzer
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros tf2_ros tf2_sensor_msgs tf2_geometry_msgs rosidl_default_runtime)
ament_package()
