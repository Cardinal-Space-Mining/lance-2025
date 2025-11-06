#pragma once

#include "../motor_interface.hpp"
#include "../../util/pub_map.hpp"


class TraversalController
{
public:
    TraversalController();
    ~TraversalController() = default;

public:
    void initialize();
    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class State
    {
        INITIALIZATION,
        TRAVERSING,
        FINISHED
    };

protected:
    const util::GenericPubMap& pub_map;

    State state{State::FINISHED};
};


// --- Implementation ----------------------------------------------------------

TraversalController::TraversalController()
{

}

void TraversalController::initialize()
{
    this->state = State::INITIALIZATION;
}

bool TraversalController::isFinished()
{
    return this->state == State::FINISHED;
}

void TraversalController::setCancelled()
{

}

void TraversalController::iterate(
    const JoyMsg& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch(this->state)
    {
        case State::INITIALIZATION:
        {

        }
        case State::TRAVERSING:
        {

        }
        case State::FINISHED:
        {

        }
    }
}
