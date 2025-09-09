cmake_minimum_required(VERSION 3.8)
project(launch_utils)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY
    src
    config
    launch
    DESTINATION share/${PROJECT_NAME} )