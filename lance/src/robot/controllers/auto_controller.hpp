#pragma once

#include "sensor_msgs/msg/joy.hpp"

#include "../motor_interface.hpp"
#include "../../util/pub_map.hpp"

#include "mining_controller.hpp"
#include "offload_controller.hpp"
#include "traversal_controller.hpp"
#include "localization_controller.hpp"


class AutoController
{
    using JoyMsg = sensor_msgs::msg::Joy;

public:
    AutoController();
    ~AutoController() = default;

public:
    void initialize();
    void setCancelled();

    void iterate(
        const JoyMsg& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        LOCALIZATION,
        MINING_INIT,
        MINING_EXEC,
        TRAVERSAL,
        OFFLOAD_INIT,
        OFFLOAD_EXEC,
        RETRAVERSAL,
        UNKNOWN
    };

protected:
    const util::GenericPubMap& pub_map;

    Stage stage{Stage::LOCALIZATION};

    MiningController mining_controller;
    OffloadController offload_controller;
    TraversalController traversal_controller;
    LocalizationController localization_controller;
};



// --- Implementation ----------------------------------------------------------

AutoController::AutoController() :
    mining_controller{},
    offload_controller{},
    traversal_controller{},
    localization_controller{}
{
}

void AutoController::initialize()
{
    // if we previously transitioned from LOCALIZATION, and we are initializing
    // again, then we don't know what the robot state is!
    if (this->stage != Stage::LOCALIZATION)
    {
        this->stage = Stage::UNKNOWN;
    }
}

void AutoController::setCancelled()
{
    switch (this->stage)
    {
        case Stage::LOCALIZATION:
        {
            this->localization_controller.setCancelled();
            break;
        }
        case Stage::MINING_INIT:
        case Stage::MINING_EXEC:
        {
            this->mining_controller.setCancelled();
            break;
        }
        case Stage::TRAVERSAL:
        case Stage::RETRAVERSAL:
        {
            this->traversal_controller.setCancelled();
            break;
        }
        case Stage::OFFLOAD_INIT:
        case Stage::OFFLOAD_EXEC:
        {
            this->offload_controller.setCancelled();
            break;
        }
    }
}

void AutoController::iterate(
    const JoyMsg& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->stage)
    {
        case Stage::LOCALIZATION:
        {
            this->localization_controller.iterate(motor_status, commands);
            if (this->localization_controller.isFinished())
            {
                this->stage = Stage::MINING_INIT;
                this->mining_controller.initialize();
            }
            else
            {
                break;
            }
            [[fallthrough]];
        }
        case Stage::MINING_INIT:
        MINING_INIT_L:
            // TODO
        case Stage::MINING_EXEC:
        {
            this->mining_controller.iterate(joy, motor_status, commands);
            if (this->mining_controller.isFinished())
            {
                this->stage = Stage::TRAVERSAL;
                // TODO: pass traversal destination here -->
                this->traversal_controller.initialize();
            }
            else
            {
                break;
            }
            [[fallthrough]];
        }
        case Stage::TRAVERSAL:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (this->traversal_controller.isFinished())
            {
                this->stage = Stage::OFFLOAD_INIT;
                this->offload_controller.initialize();
            }
            else
            {
                break;
            }
            [[fallthrough]];
        }
        case Stage::OFFLOAD_INIT:
            // TODO
        case Stage::OFFLOAD_EXEC:
        {
            this->offload_controller.iterate(joy, motor_status, commands);
            if (this->offload_controller.isFinished())
            {
                this->stage = Stage::RETRAVERSAL;
                // TODO: pass traversal destination here -->
                this->traversal_controller.initialize();
            }
            else
            {
                break;
            }
            [[fallthrough]];
        }
        case Stage::RETRAVERSAL:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (this->traversal_controller.isFinished())
            {
                this->stage = Stage::MINING_INIT;
                this->mining_controller.initialize();
                // chatgpt says I should change this to a while loop that wraps the entire switch-case
                goto MINING_INIT_L;
            }
            break;
        }
    }
}
