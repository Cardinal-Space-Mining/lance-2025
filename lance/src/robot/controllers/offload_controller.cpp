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

#include "offload_controller.hpp"


OffloadController::OffloadController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    const HopperState& hopper_state) :
    pub_map{pub_map},
    params{params},
    hopper_state{hopper_state}
{
}

void OffloadController::initialize(float traversal_dist_m)
{
    this->stage = Stage::INITIALIZATION;
    this->traversal_state.init(traversal_dist_m);
}

bool OffloadController::isFinished() { return this->stage == Stage::FINISHED; }

void OffloadController::setCancelled() { this->stage = Stage::FINISHED; }

void OffloadController::setUseHopperModel(bool enabled)
{
    this->using_hopper_model = enabled;
}

void OffloadController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->iterate(nullptr, motor_status, commands);
}

void OffloadController::iterate(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->iterate(&joy, motor_status, commands);
}


void OffloadController::TraversalState::init(float remaining_dist)
{
    this->remaining_dist = remaining_dist;
    this->prev_odom = std::numeric_limits<float>::infinity();
}
void OffloadController::TraversalState::updateOdom(float odom)
{
    if (this->remaining_dist > 0.f && !std::isinf(this->prev_odom))
    {
        this->remaining_dist -= (this->prev_odom - odom);
    }
    this->prev_odom = odom;
}
bool OffloadController::TraversalState::hasRemaining()
{
    return this->remaining_dist > 0.f;
}


void OffloadController::iterate(
    const JoyState* joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    using namespace Bindings;

    if (this->stage != Stage::INITIALIZATION && joy &&
        AssistedOffloadToggleButton::wasPressed(*joy))
    {
        this->stage = Stage::LOWERING;
    }

    this->traversal_state.updateOdom(
        static_cast<float>(track_motor_rps_to_ground_mps(
            0.5 * (motor_status.track_left.position +
                   motor_status.track_right.position))));

    commands.disableAll();

    if (joy)
    {
        float tracks_vel = TeleopDriveYAxis::deadzoneValue(
                               *joy,
                               this->params.default_stick_deadzone) *
                           this->params.tracks_offload_velocity_rps;
        commands.setTracksVelocity(tracks_vel, tracks_vel);
    }

    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            this->calculated_target_belt_pos =
                this->hopper_state.calcOffloadTargetMotorPosition(
                    motor_status.hopper_belt.position);
            this->stage = Stage::BACKUP;
            [[fallthrough]];
        }
        case Stage::BACKUP:
        {
            if (!joy && this->traversal_state.hasRemaining())
            {
                commands.setTracksVelocity(
                    -this->params.tracks_offload_velocity_rps,
                    -this->params.tracks_offload_velocity_rps);
                break;
            }

            this->stage = Stage::RAISING;
            [[fallthrough]];
        }
        case Stage::RAISING:
        {
            if (motor_status.getHopperActNormalizedValue() <
                this->params.hopper_actuator_offload_target)
            {
                commands.setHopperActPercent(
                    this->params.hopper_actuator_max_speed);
                break;
            }

            this->stage = Stage::OFFLOADING;
            [[fallthrough]];
        }
        case Stage::OFFLOADING:
        {
            const double target_belt_pos =
                this->using_hopper_model
                    ? this->hopper_state.offloadTargetMotorPosition()
                    : this->calculated_target_belt_pos;

            if (motor_status.hopper_belt.position < target_belt_pos)
            {
                commands.setHopperBeltVelocity(
                    this->params.hopper_belt_max_velocity_rps);
                break;
            }

            this->stage = Stage::LOWERING;
            [[fallthrough]];
        }
        case Stage::LOWERING:
        {
            if (motor_status.getHopperActNormalizedValue() >
                this->params.hopper_actuator_traversal_target)
            {
                commands.setHopperActPercent(
                    -this->params.hopper_actuator_max_speed);
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
