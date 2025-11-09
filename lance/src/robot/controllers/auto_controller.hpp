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
#include "../../util/pub_map.hpp"
#include "../../util/joy_utils.hpp"

#include "auto_mining_controller.hpp"
#include "auto_offload_controller.hpp"
#include "traversal_controller.hpp"
#include "localization_controller.hpp"


class AutoController
{
    using RclNode = rclcpp::Node;
    using JoyState = util::JoyState;
    using GenericPubMap = util::GenericPubMap;

public:
    AutoController(
        RclNode&,
        const GenericPubMap&,
        const RobotParams&,
        const HopperState&);
    ~AutoController() = default;

public:
    void initialize();
    void setCancelled();

    void iterate(
        const JoyState& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        LOCALIZATION,
        MINING,
        TRAVERSAL,
        OFFLOAD,
        RETRAVERSAL,
        UNKNOWN
    };

protected:
    const GenericPubMap& pub_map;
    const RobotParams& params;

    Stage stage{Stage::LOCALIZATION};

    LocalizationController localization_controller;
    TraversalController traversal_controller;
    AutoMiningController mining_controller;
    AutoOffloadController offload_controller;
};



// --- Implementation ----------------------------------------------------------

AutoController::AutoController(
    RclNode& node,
    const GenericPubMap& pub_map,
    const RobotParams& params,
    const HopperState& hopper_state) :
    pub_map{pub_map},
    params{params},
    localization_controller{node, pub_map, params},
    traversal_controller{node, pub_map, params},
    mining_controller{
        node,
        pub_map,
        params,
        hopper_state,
        traversal_controller},
    offload_controller{
        node,
        pub_map,
        params,
        hopper_state,
        traversal_controller}
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
        case Stage::MINING:
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
        case Stage::OFFLOAD:
        {
            this->offload_controller.setCancelled();
            break;
        }
        default:
        {
        }
    }
}

void AutoController::iterate(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->stage)
    {
        case Stage::LOCALIZATION:
        {
            this->localization_controller.iterate(motor_status, commands);
            if (!this->localization_controller.isFinished())
            {
                break;
            }

            this->mining_controller.initialize();
            this->stage = Stage::MINING;
            [[fallthrough]];
        }
        MINING_STAGE_L:
        case Stage::MINING:
        {
            this->mining_controller.iterate(motor_status, commands);
            if (!this->mining_controller.isFinished())
            {
                break;
            }

            // TODO: pass traversal destination here -->
            this->traversal_controller.initialize();
            this->stage = Stage::TRAVERSAL;
            [[fallthrough]];
        }
        case Stage::TRAVERSAL:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            this->offload_controller.initialize();
            this->stage = Stage::OFFLOAD;
            [[fallthrough]];
        }
        case Stage::OFFLOAD:
        {
            this->offload_controller.iterate(joy, motor_status, commands);
            if (!this->offload_controller.isFinished())
            {
                break;
            }

            // TODO: pass traversal destination here -->
            this->traversal_controller.initialize();
            this->stage = Stage::RETRAVERSAL;
            [[fallthrough]];
        }
        case Stage::RETRAVERSAL:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            this->mining_controller.initialize();
            // chatgpt says I should change this to a while loop that wraps the entire switch-case
            this->stage = Stage::MINING;
            goto MINING_STAGE_L;
        }
        default:
        {
        }
    }
}
