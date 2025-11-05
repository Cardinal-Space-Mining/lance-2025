#pragma once

#include "sensor_msgs/msg/joy.hpp"

#include "../motor_interface.hpp"


class MiningController
{
    using JoyMsg = sensor_msgs::msg::Joy;

public:
    MiningController();
    ~MiningController() = default;

public:
    void initialize();
    bool isFinished();
    void setCancelled();

    void iterate(
        const JoyMsg& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        INITIALIZATION,
        LOWERING,
        TRAVERSING,
        RAISING,
        FINISHED
    };

protected:
    Stage stage{Stage::FINISHED};
};
