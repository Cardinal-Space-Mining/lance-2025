#pragma once

#include "sensor_msgs/msg/joy.hpp"

#include "../hid_constants.hpp"
#include "../motor_interface.hpp"
#include "../../util/pub_map.hpp"

#include "mining_controller.hpp"
#include "offload_controller.hpp"


class TeleopController
{
    using JoyMsg = sensor_msgs::msg::Joy;

public:
    TeleopController();
    ~TeleopController() = default;

public:
    void initialize();
    void setCancelled();

    void iterate(
        const JoyMsg& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Operation
    {
        MANUAL,
        ASSISTED_MINING,
        ASSISTED_OFFLOAD,
        PRESET_MINING,
        PRESET_OFFLOAD
    };

protected:
    const util::GenericPubMap& pub_map;

    Operation op_mode{Operation::MANUAL};

    MiningController mining_controller;
    OffloadController offload_controller;
};



// --- Implementation ----------------------------------------------------------

TeleopController::TeleopController() : mining_controller{}, offload_controller{}
{
}

void TeleopController::initialize() { this->op_mode = Operation::MANUAL; }

void TeleopController::setCancelled()
{
    switch (this->op_mode)
    {
        case Operation::ASSISTED_MINING:
        case Operation::PRESET_MINING:
        {
            this->mining_controller.setCancelled();
            break;
        }
        case Operation::ASSISTED_OFFLOAD:
        case Operation::PRESET_OFFLOAD:
        {
            this->offload_controller.setCancelled();
            break;
        }
    }
}

void TeleopController::iterate(
    const JoyMsg& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    // iterate controllers... if inputs result in finish state, continue
    // to iterate manual mode below (motor commands meaningless anyway)
    bool command_finished = false;
    switch (this->op_mode)
    {
        // same controllers, init determines exec mode
        case Operation::ASSISTED_MINING:
        case Operation::PRESET_MINING:
        {
            this->mining_controller.iterate(joy, motor_status, commands);
            command_finished = this->mining_controller.isFinished();
            break;
        }
        // same controllers, init determines exec mode
        case Operation::ASSISTED_OFFLOAD:
        case Operation::PRESET_OFFLOAD:
        {
            this->offload_controller.iterate(joy, motor_status, commands);
            command_finished = this->offload_controller.isFinished();
            break;
        }
    }
    if (command_finished)
    {
        this->op_mode = Operation::MANUAL;
        // commands.disableAll(); <-- can add back to be extra safe
    }

    // controllers either iterated and didn't finish (op_mode isn't MANUAL),
    // or a transition to MANUAL occurred, in which case we can override
    // any motor commands since they are worthless
    if (this->op_mode == Operation::MANUAL)
    {
        // inputs --> commands & inputs --> state transisions

        // iterate controllers ONLY IF an op_mode transition occurred
        // (otherwise op_mode will still be MANUAL)
        switch (this->op_mode)
        {
            case Operation::ASSISTED_MINING:
            case Operation::PRESET_MINING:
            {
                this->mining_controller.iterate(joy, motor_status, commands);
                break;
            }
            case Operation::ASSISTED_OFFLOAD:
            case Operation::PRESET_OFFLOAD:
            {
                this->offload_controller.iterate(joy, motor_status, commands);
                break;
            }
        }
    }
}
