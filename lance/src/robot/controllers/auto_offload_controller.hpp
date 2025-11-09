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

#include "../robot_params.hpp"
#include "../motor_interface.hpp"
#include "../collection_state.hpp"

#include "offload_controller.hpp"
#include "traversal_controller.hpp"


class AutoOffloadController
{
    using RclNode = rclcpp::Node;
    using JoyState = util::JoyState;
    using GenericPubMap = util::GenericPubMap;

public:
    AutoOffloadController(
        RclNode&,
        const GenericPubMap&,
        const RobotParams&,
        const HopperState&,
        TraversalController&);
    ~AutoOffloadController() = default;

public:
    void initialize();
    bool isFinished();
    void setCancelled();

    void iterate(
        const JoyState& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        INITIALIZATION,
        PLANNING,
        TRAVERSING,
        OFFLOADING,
        FINISHED
    };

protected:
    const GenericPubMap& pub_map;
    const RobotParams& params;
    const HopperState& hopper_state;

    Stage stage{Stage::FINISHED};

    TraversalController& traversal_controller;
    OffloadController offload_controller;
};


// --- Implementation ----------------------------------------------------------

AutoOffloadController::AutoOffloadController(
    RclNode& node,
    const GenericPubMap& pub_map,
    const RobotParams& params,
    const HopperState& hopper_state,
    TraversalController& trav_controller) :
    pub_map{pub_map},
    params{params},
    hopper_state{hopper_state},
    traversal_controller{trav_controller},
    offload_controller{node, pub_map, params, hopper_state}
{
}

void AutoOffloadController::initialize()
{
    this->stage = Stage::INITIALIZATION;
}

bool AutoOffloadController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void AutoOffloadController::setCancelled() { this->stage = Stage::FINISHED; }

void AutoOffloadController::iterate(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            this->stage = Stage::PLANNING;
            [[fallthrough]];
        }
        case Stage::PLANNING:
        {
            if (false)  // *if not finished planning*
            {
                // planning algo here

                break;  // break if more work is required
            }

            // init with planned destination
            this->traversal_controller.initialize();
            this->stage = Stage::TRAVERSING;
            [[fallthrough]];
        }
        case Stage::TRAVERSING:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            // initialize with query result
            this->offload_controller.initialize();
            this->stage = Stage::OFFLOADING;
            [[fallthrough]];
        }
        case Stage::OFFLOADING:
        {
            this->offload_controller.iterate(joy, motor_status, commands);
            if (!this->offload_controller.isFinished())
            {
                break;
            }

            this->stage = Stage::FINISHED;
            [[fallthrough]];
        }
        case Stage::FINISHED:
        {
        }
    }
}
