#pragma once

#include "phoenix_ros_driver/msg/talon_ctrl.hpp"
#include "phoenix_ros_driver/msg/talon_info.hpp"


using TalonCtrlMsg = phoenix_ros_driver::msg::TalonCtrl;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;

/** Contains TalonInfo for each motor */
struct RobotMotorStatus
{
    TalonInfoMsg track_right;
    TalonInfoMsg track_left;
    TalonInfoMsg trencher;
    TalonInfoMsg hopper_belt;
    TalonInfoMsg hopper_actuator;
};

/** Contains TalonCtrl for each motor */
struct RobotMotorCommands
{
    TalonCtrlMsg track_right;
    TalonCtrlMsg track_left;
    TalonCtrlMsg trencher;
    TalonCtrlMsg hopper_belt;
    TalonCtrlMsg hopper_actuator;

    inline void setHopperActPercent(double percent)
    {
        this->hopper_actuator.set__mode(TalonCtrlMsg::PERCENT_OUTPUT)
            .set__value(percent);
    }
    inline void setHopperBeltVelocity(double rps)
    {
        this->hopper_belt.set__mode(TalonCtrlMsg::VELOCITY).set__value(rps);
    }
    inline void setTrencherVelocity(double rps)
    {
        this->trencher.set__mode(TalonCtrlMsg::VELOCITY).set__value(rps);
    }
    inline void setTracksVelocity(double left_rps, double right_rps)
    {
        this->track_left.set__mode(TalonCtrlMsg::VELOCITY).set__value(left_rps);
        this->track_right.set__mode(TalonCtrlMsg::VELOCITY)
            .set__value(right_rps);
    }

    inline void disableAll()
    {
        this->track_left.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
        this->track_right.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
        this->trencher.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
        this->hopper_belt.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
        this->hopper_actuator.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
    }
};
