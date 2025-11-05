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

    inline void set_hopper_act_percent(double percent)
    {
        this->hopper_actuator.set__mode(TalonCtrl::PERCENT_OUTPUT)
            .set__value(percent);
    }
    inline void set_hopper_belt_velocity(double rps)
    {
        this->hopper_belt.set__mode(TalonCtrl::VELOCITY).set__value(rps);
    }
    inline void set_trencher_velocity(double rps)
    {
        this->trencher.set__mode(TalonCtrl::VELOCITY).set__value(rps);
    }
    inline void set_tracks_velocity(double left_rps, double right_rps)
    {
        this->track_left.set__mode(TalonCtrl::VELOCITY).set__value(left_rps);
        this->track_right.set__mode(TalonCtrl::VELOCITY).set__value(right_rps);
    }
};
