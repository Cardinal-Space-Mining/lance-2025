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

#include <limits>

#include <rclcpp/rclcpp.hpp>

#include "../robot_math.hpp"
#include "../hid_bindings.hpp"
#include "../robot_params.hpp"
#include "../motor_interface.hpp"
#include "../collection_state.hpp"
#include "../../util/pub_map.hpp"
#include "../../util/joy_utils.hpp"


class MiningController
{
    using RclNode = rclcpp::Node;
    using JoyState = util::JoyState;
    using GenericPubMap = util::GenericPubMap;

public:
    MiningController(
        RclNode&,
        const GenericPubMap&,
        const RobotParams&,
        const HopperState&);
    ~MiningController() = default;

public:
    void initialize(float traversal_dist_m = 0.f);
    bool isFinished();
    void setCancelled();

    void setRemaining(float traversal_dist_m);
    void iterate(
        const JoyState& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        INITIALIZATION,
        LOWERING,
        TRAVERSING,
        RAISING,
        FINISHED
    };

protected:
    const GenericPubMap& pub_map;
    const RobotParams& params;
    const HopperState& hopper_state;

    Stage stage{Stage::FINISHED};
    float remaining_distance_m{0.f};
    float prev_tracks_odom{0.f};
};


// --- Implementation ----------------------------------------------------------

MiningController::MiningController(
    RclNode& node,
    const GenericPubMap& pub_map,
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
    if (traversal_dist_m <= 0.f)
    {
        this->remaining_distance_m = std::numeric_limits<float>::infinity();
    }
}

bool MiningController::isFinished() { return this->stage == Stage::FINISHED; }

void MiningController::setCancelled() { this->stage = Stage::FINISHED; }

void MiningController::iterate(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    using namespace Bindings;

    if (this->stage != Stage::INITIALIZATION &&
        AssistedMiningToggleButton::wasPressed(joy))
    {
        this->stage = Stage::RAISING;
    }

    const float tracks_odom = static_cast<float>(track_motor_rps_to_ground_mps(
        0.5 * (motor_status.track_left.position +
               motor_status.track_right.position)));

    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            this->prev_tracks_odom = tracks_odom;
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
            else
            {
                commands.disableHopperAct();
                [[fallthrough]];
            }
        }
        case Stage::TRAVERSING:
        {
        }
        case Stage::RAISING:
        {
        }
        case Stage::FINISHED:
        {
        }
    }

    this->prev_tracks_odom = tracks_odom;
}
