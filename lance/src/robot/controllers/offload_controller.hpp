#pragma once

#include "sensor_msgs/msg/joy.hpp"

#include "../motor_interface.hpp"
#include "../../util/pub_map.hpp"


class OffloadController
{
    using JoyMsg = sensor_msgs::msg::Joy;

public:
    OffloadController();
    ~OffloadController() = default;

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
        BACKUP,
        RAISING,
        OFFLOADING,
        LOWERING,
        FINISHED
    };

protected:
    const util::GenericPubMap& pub_map;

    Stage stage{Stage::FINISHED};

};


// --- Implementation ----------------------------------------------------------

OffloadController::OffloadController()
{

}

void OffloadController::initialize()
{
    this->stage = Stage::INITIALIZATION;
}

bool OffloadController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void OffloadController::setCancelled()
{

}

void OffloadController::iterate(
    const JoyMsg& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch(this->stage)
    {
        case Stage::INITIALIZATION:
        {

        }
        case Stage::BACKUP:
        {

        }
        case Stage::RAISING:
        {

        }
        case Stage::OFFLOADING:
        {

        }
        case Stage::LOWERING:
        {

        }
        case Stage::FINISHED:
        {

        }
    }
}
