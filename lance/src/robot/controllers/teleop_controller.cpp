/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
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

#include "teleop_controller.hpp"

#include "../hid_bindings.hpp"


TeleopController::TeleopController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    const HopperState& hopper_state) :
    pub_map{pub_map},
    params{params},
    driving_rps_scalar{
        params.driving_medium_scalar * params.tracks_max_velocity_rps},
    mining_controller{node, pub_map, params, hopper_state},
    offload_controller{node, pub_map, params, hopper_state}
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
    if (!this->handleGlobalInputs(joy))
    {
        commands.disableAll();
        return;
    }

    // iterate controllers... if inputs result in finish state, continue
    // to iterate manual mode below (motor commands meaningless anyway)
    bool command_finished = false;
    switch (this->op_mode)
    {
        case Operation::ASSISTED_MINING:
        {
            this->mining_controller.iterate(joy, motor_status, commands);
            command_finished = this->mining_controller.isFinished();
            break;
        }
        case Operation::ASSISTED_OFFLOAD:
        {
            this->offload_controller.iterate(joy, motor_status, commands);
            command_finished = this->offload_controller.isFinished();
            break;
        }
        case Operation::PRESET_MINING:
        {
            this->mining_controller.iterate(motor_status, commands);
            command_finished = this->mining_controller.isFinished();
            break;
        }
        case Operation::PRESET_OFFLOAD:
        {
            this->offload_controller.iterate(motor_status, commands);
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
            {
                this->mining_controller.iterate(joy, motor_status, commands);
                break;
            }
            case Operation::ASSISTED_OFFLOAD:
            {
                this->offload_controller.iterate(joy, motor_status, commands);
                break;
            }
            case Operation::PRESET_MINING:
            {
                this->mining_controller.iterate(motor_status, commands);
                break;
            }
            case Operation::PRESET_OFFLOAD:
            {
                this->offload_controller.iterate(motor_status, commands);
                break;
            }
            default:
            {
            }
        }
    }

    this->publishState();
}

bool TeleopController::handleGlobalInputs(const JoyState& joy)
{
    using namespace Bindings;

    if (TeleopLowSpeedButton::wasPressed(joy))
    {
        this->driving_rps_scalar = this->params.driving_low_scalar *
                                   this->params.tracks_max_velocity_rps;
    }
    if (TeleopMediumSpeedButton::wasPressed(joy))
    {
        this->driving_rps_scalar = this->params.driving_medium_scalar *
                                   this->params.tracks_max_velocity_rps;
    }
    if (TeleopHighSpeedButton::wasPressed(joy))
    {
        this->driving_rps_scalar = this->params.driving_high_scalar *
                                   this->params.tracks_max_velocity_rps;
    }

    if (DisableAllActionsButton::rawValue(joy))
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

    if (AssistedMiningToggleButton::wasPressed(joy))
    {
        this->mining_controller.initialize();
        this->op_mode = Operation::ASSISTED_MINING;
        return;
    }
    if (AssistedOffloadToggleButton::wasPressed(joy))
    {
        this->offload_controller.initialize();
        this->op_mode = Operation::ASSISTED_OFFLOAD;
        return;
    }

    if (PresetMiningInitButton::wasPressed(joy))
    {
        this->mining_controller.initialize(
            this->params.preset_mining_traversal_dist_meters);
        this->op_mode = Operation::PRESET_MINING;
        return;
    }
    if (PresetOffloadInitButton::wasPressed(joy))
    {
        this->offload_controller.initialize(
            this->params.preset_offload_backup_dist_meters);
        this->op_mode = Operation::PRESET_OFFLOAD;
        return;
    }

    // tracks
    {
        const float x = TeleopDriveXAxis::rawValue(joy);
        const float y = TeleopDriveYAxis::rawValue(joy);
        if ((x * x + y * y) < (this->params.driving_magnitude_deadzone *
                               this->params.driving_magnitude_deadzone))
        {
            commands.setTracksVelocity(0., 0.);
        }
        else
        {
            const float l = y - x;  // y + (-x)
            const float r = y + x;  // y - (-x)
            const float s =
                (this->driving_rps_scalar /
                 std::max({1.f, std::abs(l), std::abs(r)}));
            commands.setTracksVelocity((l * s), (r * s));
        }
    }
    // trencher
    {
        float trencher_percent = TeleopTrencherSpeedAxis::triggerValue(joy);
        if (TeleopTrencherInvertButton::rawValue(joy))
        {
            trencher_percent *= -1.f;
        }

        commands.setTrencherVelocity(
            trencher_percent * this->params.trencher_max_velocity_rps);
    }
    // hopper
    {
        float hopper_belt_percent = TeleopHopperSpeedAxis::triggerValue(joy);
        if (TeleopHopperInvertButton::rawValue(joy))
        {
            hopper_belt_percent *= -1.;
        }
        commands.setHopperBeltVelocity(
            hopper_belt_percent * this->params.hopper_belt_max_velocity_rps);

        float hopper_act_scalar = TeleopHopperActuateAxis::rawValue(joy);
        if (std::abs(hopper_act_scalar) < this->params.default_stick_deadzone)
        {
            hopper_act_scalar = 0.f;
        }
        commands.setHopperActPercent(hopper_act_scalar);
    }
}

void TeleopController::publishState()
{
    static constexpr char const* OP_STRINGS[] = {
        "Teleop Manual",
        "Teleop Assisted Mining",
        "Teleop Assisted Offload",
        "Teleop Preset Mining",
        "Teleop Preset Offload"};

    this->pub_map.publish<std_msgs::msg::String, std::string>(
        "/lance/op_status",
        OP_STRINGS[static_cast<size_t>(this->op_mode)]);
}
