#include "mining_controller.hpp"


MiningController::MiningController()
{

}

void MiningController::initialize()
{
    this->stage = Stage::INITIALIZATION;
}

bool MiningController::isFinished()
{

}

void MiningController::setCancelled()
{

}

void MiningController::iterate(
    const JoyMsg& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch(this->stage)
    {
        case Stage::INITIALIZATION:
        {

        }
    }
}
