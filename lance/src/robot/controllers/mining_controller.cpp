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

#include "mining_controller.hpp"


MiningController::MiningController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    const HopperState& hopper_state) :
    pub_map{pub_map},
    params{params},
    hopper_state{hopper_state}
{
}

void MiningController::initialize(float traversal_dist_m)
{
    this->stage = Stage::INITIALIZATION;
    this->traversal_state.init(traversal_dist_m);
}

bool MiningController::isFinished() { return this->stage == Stage::FINISHED; }

void MiningController::setCancelled() { this->stage = Stage::FINISHED; }

void MiningController::setRemaining(float traversal_dist_m)
{
    this->traversal_state.setRemaining(traversal_dist_m);
}

void MiningController::setUseHopperModel(bool enabled)
{
    this->using_hopper_model = enabled;
}

void MiningController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->iterate(nullptr, motor_status, commands);
}

void MiningController::iterate(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->iterate(&joy, motor_status, commands);
}


void MiningController::TraversalState::init(float remaining_dist)
{
    if (remaining_dist <= 0.f)
    {
        this->setRemaining(std::numeric_limits<float>::infinity());
    }
    else
    {
        this->setRemaining(remaining_dist);
    }
    this->prev_odom = std::numeric_limits<float>::infinity();
}
void MiningController::TraversalState::setRemaining(float remaining_dist)
{
    this->remaining_dist = remaining_dist;
}
void MiningController::TraversalState::updateOdom(float odom)
{
    if (!std::isinf(this->remaining_dist) && !std::isinf(this->prev_odom))
    {
        this->remaining_dist -= (odom - this->prev_odom);
    }
    this->prev_odom = odom;
}
bool MiningController::TraversalState::hasRemaining()
{
    return this->remaining_dist > 0.f;
}


void MiningController::BeltDutyCycleState::setMoved()
{
    this->belt_moving = true;
}
void MiningController::BeltDutyCycleState::setStopped()
{
    if (this->belt_moving)
    {
        this->prev_belt_stop_time = std::chrono::system_clock::now();
        this->belt_moving = false;
    }
}
bool MiningController::BeltDutyCycleState::canMove(float thresh_s)
{
    return std::chrono::duration<float>(
               std::chrono::system_clock::now() - this->prev_belt_stop_time)
               .count() >= thresh_s;
}


void MiningController::iterate(
    const JoyState* joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    using namespace Bindings;

    if (this->stage != Stage::INITIALIZATION && joy &&
        AssistedMiningToggleButton::wasPressed(*joy))
    {
        this->stage = Stage::RAISING;
    }

    this->traversal_state.updateOdom(
        static_cast<float>(track_motor_rps_to_ground_mps(
            0.5 * (motor_status.track_left.position +
                   motor_status.track_right.position))));

    commands.disableAll();

    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            this->stage = Stage::LOWERING;
            [[fallthrough]];
        }
        case Stage::LOWERING:
        {
            const double hopper_act_val =
                motor_status.getHopperActNormalizedValue();
            if (hopper_act_val > this->params.hopper_actuator_mining_target)
            {
                commands.setTrencherVelocity(
                    this->params.trencher_mining_velocity_rps);
                if (hopper_act_val >
                    this->params.hopper_actuator_traversal_target)
                {
                    commands.setHopperActPercent(
                        -this->params.hopper_actuator_max_speed);
                }
                else
                {
                    commands.setHopperActPercent(
                        -this->params.hopper_actuator_plunge_speed);
                }
                break;
            }

            this->stage = Stage::TRAVERSING;
            [[fallthrough]];
        }
        case Stage::TRAVERSING:
        {
            const bool finished_state =
                ((this->hopper_state.isBeltCapacity() &&
                  this->using_hopper_model) ||
                 !this->traversal_state.hasRemaining());

            if (!finished_state)
            {
                // default setpoints
                float trencher_target =
                    this->params.trencher_mining_velocity_rps;
                float hopper_act_target =
                    this->params.hopper_actuator_mining_target;
                float tracks_target = this->params.tracks_mining_velocity_rps;
                float hopper_belt_target = 0.f;

                // 1. Set belt via hopper model target
                if (this->using_hopper_model)
                {
                    if (motor_status.hopper_belt.position <
                            this->hopper_state.miningTargetMotorPosition() &&
                        this->belt_duty_cycle.canMove(
                            this->params
                                .hopper_belt_mining_duty_cycle_base_seconds))
                    {
                        hopper_belt_target =
                            this->params.hopper_belt_mining_velocity_rps;
                        this->belt_duty_cycle.setMoved();
                    }
                    else
                    {
                        this->belt_duty_cycle.setStopped();
                    }
                }

                // 2. Commands/target updates from user input
                if (joy)
                {
                    // adjust trencher
                    trencher_target =
                        this->params.trencher_mining_velocity_rps *
                        (1.f - TeleopTrencherSpeedAxis::triggerValue(*joy));

                    // adjust trencher depth
                    {
                        const float raw =
                            TeleopHopperActuateAxis::rawValue(*joy);
                        if (std::abs(raw) >=
                            this->params.default_stick_deadzone)
                        {
                            if (raw > 0.f)
                            {
                                hopper_act_target +=
                                    raw *
                                    (this->params
                                         .hopper_actuator_transport_target -
                                     this->params
                                         .hopper_actuator_mining_target);
                            }
                            else if (raw < 0.f)
                            {
                                hopper_act_target +=
                                    raw *
                                    (this->params
                                         .hopper_actuator_mining_target -
                                     this->params.hopper_actuator_mining_min);
                            }
                        }
                    }
                    // adjust tracks
                    {
                        const float raw = TeleopDriveYAxis::rawValue(*joy);
                        if (std::abs(raw) >=
                            this->params.driving_magnitude_deadzone)
                        {
                            if (raw > 0.f)
                            {
                                tracks_target +=
                                    raw *
                                    this->params
                                        .tracks_mining_adjustment_range_rps;
                            }
                            else if (raw < 0.f)
                            {
                                tracks_target +=
                                    raw *
                                    this->params.tracks_mining_velocity_rps;
                            }
                        }
                    }
                    // manual hopper belt - don't override automatic setpts
                    if (!this->using_hopper_model)
                    {
                        hopper_belt_target =
                            TeleopHopperSpeedAxis::triggerValue(*joy) *
                            this->params.hopper_belt_mining_velocity_rps;
                    }
                }

                // 3. Apply targets
                {
                    commands.setTrencherVelocity(trencher_target);
                    commands.setTracksVelocity(tracks_target, tracks_target);
                    commands.setHopperBeltVelocity(hopper_belt_target);

                    const double hopper_val =
                        motor_status.getHopperActNormalizedValue();
                    if (std::abs(hopper_act_target - hopper_val) <
                        this->params.hopper_actuator_targetting_thresh)
                    {
                        commands.disableHopperAct();
                    }
                    else if (hopper_val < hopper_act_target)
                    {
                        commands.setHopperActPercent(
                            this->params.hopper_actuator_plunge_speed);
                    }
                    else if (hopper_val > hopper_act_target)
                    {
                        commands.setHopperActPercent(
                            -this->params.hopper_actuator_plunge_speed);
                    }
                }

                break;
            }

            this->stage = Stage::RAISING;
            [[fallthrough]];
        }
        case Stage::RAISING:
        {
            if (motor_status.getHopperActNormalizedValue() <
                this->params.hopper_actuator_transport_target)
            {
                commands.setTrencherVelocity(
                    this->params.trencher_mining_velocity_rps);
                commands.setHopperActPercent(
                    this->params.hopper_actuator_extract_speed);
                break;
            }

            this->stage = Stage::FINISHED;
            [[fallthrough]];
        }
        case Stage::FINISHED:
        {
            //
        }
    }
}
