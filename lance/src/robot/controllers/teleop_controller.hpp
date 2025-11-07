/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "../hid_bindings.hpp"
#include "../motor_interface.hpp"
#include "../collection_state.hpp"
#include "../../util/pub_map.hpp"
#include "../../util/joy_utils.hpp"

#include "mining_controller.hpp"
#include "offload_controller.hpp"


class TeleopController
{
    using RclNode = rclcpp::Node;
    using JoyState = util::JoyState;
    using GenericPubMap = util::GenericPubMap;

public:
    TeleopController(RclNode&, const GenericPubMap&, const HopperState&);
    ~TeleopController() = default;

public:
    void initialize();
    void setCancelled();

    void iterate(
        const JoyState& joy,
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
    bool handleGlobalInputs(const JoyState& joy);
    void handleTeleopInputs(const JoyState& joy, RobotMotorCommands& commands);

protected:
    const GenericPubMap& pub_map;

    Operation op_mode{Operation::MANUAL};

    MiningController mining_controller;
    OffloadController offload_controller;
};



// --- Implementation ----------------------------------------------------------

TeleopController::TeleopController(
    RclNode& node,
    const GenericPubMap& pub_map,
    const HopperState& hopper_state) :
    pub_map{pub_map},
    mining_controller{node, pub_map, hopper_state},
    offload_controller{node, pub_map, hopper_state}
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
        default:
        {
        }
    }
}

void TeleopController::iterate(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    // handle "config" setters and "disable all" button
    if(!this->handleGlobalInputs(joy))
    {
        commands.disableAll();
        return;
    }

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
        default:
        {
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
        // handle manual control
        this->handleTeleopInputs(joy, commands);

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
            default:
            {
            }
        }
    }
}

bool TeleopController::handleGlobalInputs(const JoyState& joy)
{
    using namespace Bindings;

    if(TeleopLowSpeedButton::wasPressed(joy))
    {
        // set low speed
    }
    if(TeleopMediumSpeedButton::wasPressed(joy))
    {
        // set medium speed
    }
    if(TeleopHighSpeedButton::wasPressed(joy))
    {
        // set high speed
    }

    if(AssistedHopperEnableButton::wasPressed(joy))
    {
        // set state
    }
    if(AssistedHopperDisableButton::wasPressed(joy))
    {
        // set state
    }

    if(DisableAllActionsButton::rawValue(joy))
    {
        this->mining_controller.setCancelled();
        this->offload_controller.setCancelled();
        return false;
    }

    return true;
}
void TeleopController::handleTeleopInputs(
    const JoyState& joy,
    RobotMotorCommands& commands)
{
    using namespace Bindings;

    if(AssistedMiningToggleButton::wasPressed(joy))
    {
        this->mining_controller.initialize();
        this->op_mode = Operation::ASSISTED_MINING;
        return;
    }
    if(AssistedOffloadToggleButton::wasPressed(joy))
    {
        this->offload_controller.initialize();
        this->op_mode = Operation::ASSISTED_OFFLOAD;
        return;
    }

    // TODO: preset mining, offload

    // tracks
    {
        const double stick_x = TeleopDriveXAxis::rawValue(joy);
        const double stick_y = TeleopDriveYAxis::rawValue(joy);
        // deadzone? ^

        // arcade to differential, speed scalar + max rps scaling

        commands.setTracksVelocity(0., 0.);
    }
    // trencher
    {
        double trencher_scalar = TeleopTrencherSpeedAxis::triggerValue(joy);
        if(TeleopTrencherInvertButton::rawValue(joy))
        {
            trencher_scalar *= -1.;
        }

        // scale by max rps

        commands.setTrencherVelocity(0.);
    }
    // hopper
    {
        double hopper_belt_scalar = TeleopHopperSpeedAxis::triggerValue(joy);
        if(TeleopHopperInvertButton::rawValue(joy))
        {
            hopper_belt_scalar *= -1.;
        }

        // scale by max rps

        commands.setHopperBeltVelocity(0.);

        double hopper_act_scalar = TeleopHopperActuateAxis::rawValue(joy);

        // deadband

        commands.setHopperActPercent(0.);
    }

}
