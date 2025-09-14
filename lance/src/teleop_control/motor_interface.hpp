#pragma once

#include "phoenix_ros_driver/msg/talon_ctrl.hpp"
#include "phoenix_ros_driver/msg/talon_info.hpp"


using phoenix_ros_driver::msg::TalonCtrl;
using phoenix_ros_driver::msg::TalonInfo;

/** Contains TalonInfo for each motor */
struct RobotMotorStatus
{
    TalonInfo track_right;
    TalonInfo track_left;
    TalonInfo trencher;
    TalonInfo hopper_belt;
    TalonInfo hopper_actuator;
};

/** Contains TalonCtrl for each motor */
struct RobotMotorCommands
{
    TalonCtrl track_right;
    TalonCtrl track_left;
    TalonCtrl trencher;
    TalonCtrl hopper_belt;
    TalonCtrl hopper_actuator;
};
