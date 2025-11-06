#pragma once

#include <cstdint>

#include "sensor_msgs/msg/joy.hpp"

#include "../util/pub_map.hpp"

#include "motor_interface.hpp"
#include "collection_state.hpp"

#include "controllers/auto_controller.hpp"
#include "controllers/teleop_controller.hpp"


class RobotController
{
    using JoyMsg = sensor_msgs::msg::Joy;

public:
    enum class ControlMode
    {
        DISABLED = 0,
        TELEOP = 1,
        AUTO = 2
    };

public:
    RobotController();
    ~RobotController() = default;

public:
    void iterate(
        int32_t watchdog,
        const JoyMsg& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    const util::GenericPubMap& pub_map;

    ControlMode control_mode{ControlMode::DISABLED};

    AutoController auto_controller;
    TeleopController teleop_controller;
};



// --- Implementation ----------------------------------------------------------

inline constexpr RobotController::ControlMode getMode(int32_t watchdog)
{
    return watchdog > 0
               ? RobotController::ControlMode::TELEOP
               : (watchdog < 0 ? RobotController::ControlMode::AUTO
                               : RobotController::ControlMode::DISABLED);
}
inline constexpr int encodeTransition(
    RobotController::ControlMode from,
    RobotController::ControlMode to)
{
    return (static_cast<int>(from) << 2) | static_cast<int>(to);
}

template<RobotController::ControlMode FromV, RobotController::ControlMode ToV>
inline constexpr int transition_v = encodeTransition(FromV, ToV);



RobotController::RobotController() : auto_controller{}, teleop_controller{} {}

void RobotController::iterate(
    int32_t watchdog,
    const JoyMsg& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    const ControlMode prev_mode = this->control_mode;
    this->control_mode = getMode(watchdog);

    // process transition actions
    switch (encodeTransition(prev_mode, this->control_mode))
    {
        case transition_v<ControlMode::DISABLED, ControlMode::TELEOP>:
        {
            this->teleop_controller.initialize();
            break;
        }
        case transition_v<ControlMode::DISABLED, ControlMode::AUTO>:
        {
            this->auto_controller.initialize();
            break;
        }
        case transition_v<ControlMode::TELEOP, ControlMode::DISABLED>:
        {
            this->teleop_controller.setCancelled();
            break;
        }
        case transition_v<ControlMode::TELEOP, ControlMode::AUTO>:
        {
            this->teleop_controller.setCancelled();
            this->auto_controller.initialize();
            break;
        }
        case transition_v<ControlMode::AUTO, ControlMode::DISABLED>:
        {
            this->auto_controller.setCancelled();
            break;
        }
        case transition_v<ControlMode::AUTO, ControlMode::TELEOP>:
        {
            this->auto_controller.setCancelled();
            this->teleop_controller.initialize();
            break;
        }
    }

    // process current state actions
    switch (this->control_mode)
    {
        case ControlMode::TELEOP:
        {
            this->teleop_controller.iterate(joy, motor_status, commands);
            break;
        }
        case ControlMode::AUTO:
        {
            this->auto_controller.iterate(joy, motor_status, commands);
        }
    }
}
