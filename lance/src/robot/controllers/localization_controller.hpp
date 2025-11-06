#pragma once

#include "../motor_interface.hpp"
#include "../../util/pub_map.hpp"


class LocalizationController
{
public:
    LocalizationController();
    ~LocalizationController() = default;

public:
    void initialize();
    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        INITIALIZATION,
        SEARCHING,
        TARGETTING,
        FINISHED
    };

protected:
    const util::GenericPubMap& pub_map;

    Stage stage{Stage::FINISHED};
};


// --- Implementation ----------------------------------------------------------

LocalizationController::LocalizationController()
{

}

void LocalizationController::initialize()
{
    this->stage = Stage::INITIALIZATION;
}

bool LocalizationController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void LocalizationController::setCancelled()
{

}

void LocalizationController::iterate(
    const JoyMsg& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch(this->stage)
    {
        case Stage::INITIALIZATION:
        {

        }
        case Stage::SEARCHING:
        {

        }
        case Stage::TARGETTING:
        {

        }
        case Stage::FINISHED:
        {

        }
    }
}
