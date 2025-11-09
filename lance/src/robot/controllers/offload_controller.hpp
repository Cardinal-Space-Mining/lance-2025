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

#include "../robot_math.hpp"
#include "../hid_bindings.hpp"
#include "../robot_params.hpp"
#include "../motor_interface.hpp"
#include "../collection_state.hpp"
#include "../../util/pub_map.hpp"
#include "../../util/joy_utils.hpp"


class OffloadController
{
    using RclNode = rclcpp::Node;
    using JoyState = util::JoyState;
    using GenericPubMap = util::GenericPubMap;

public:
    OffloadController(
        RclNode&,
        const GenericPubMap&,
        const RobotParams&,
        const HopperState&);
    ~OffloadController() = default;

public:
    void initialize(float traversal_dist_m = 0.f);
    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);
    void iterate(
        const JoyState& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        INITIALIZATION,
        BACKUP,
        RAISING,
        OFFLOADING,
        LOWERING,
        FINISHED
    };

    struct TraversalState
    {
        void init(float remaining_dist = 0.f);
        void updateOdom(float odom);
        bool hasRemaining();

    private:
        float remaining_dist{0.f};
        float prev_odom{0.f};
    };

protected:
    void iterate(
        const JoyState* joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    const GenericPubMap& pub_map;
    const RobotParams& params;
    const HopperState& hopper_state;

    Stage stage{Stage::FINISHED};
    TraversalState traversal_state{};
};


// --- Implementation ----------------------------------------------------------

OffloadController::OffloadController(
    RclNode& node,
    const GenericPubMap& pub_map,
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

void OffloadController::setCancelled()
{
    this->stage = Stage::FINISHED;
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

    if(this->stage != Stage::INITIALIZATION && joy &&
        AssistedOffloadToggleButton::wasPressed(*joy))
    {
        this->stage = Stage::LOWERING;
    }

    this->traversal_state.updateOdom(
        static_cast<float>(track_motor_rps_to_ground_mps(
            0.5 * (motor_status.track_left.position +
                   motor_status.track_right.position))));

    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
        }
        case Stage::BACKUP:
        {
        }
        case Stage::RAISING:
        {
        }
        case Stage::OFFLOADING:
        {
        }
        case Stage::LOWERING:
        {
        }
        case Stage::FINISHED:
        {
        }
    }
}
