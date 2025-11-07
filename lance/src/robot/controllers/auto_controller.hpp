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

#include "../motor_interface.hpp"
#include "../collection_state.hpp"
#include "../../util/pub_map.hpp"
#include "../../util/joy_utils.hpp"

#include "mining_controller.hpp"
#include "offload_controller.hpp"
#include "traversal_controller.hpp"
#include "localization_controller.hpp"


class AutoController
{
    using RclNode = rclcpp::Node;
    using JoyState = util::JoyState;
    using GenericPubMap = util::GenericPubMap;

public:
    AutoController(RclNode&, const GenericPubMap&, const HopperState&);
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
        MINING_INIT,
        MINING_EXEC,
        TRAVERSAL,
        OFFLOAD_INIT,
        OFFLOAD_EXEC,
        RETRAVERSAL,
        UNKNOWN
    };

protected:
    const GenericPubMap& pub_map;

    Stage stage{Stage::LOCALIZATION};

    MiningController mining_controller;
    OffloadController offload_controller;
    TraversalController traversal_controller;
    LocalizationController localization_controller;
};



// --- Implementation ----------------------------------------------------------

AutoController::AutoController(
    RclNode& node,
    const GenericPubMap& pub_map,
    const HopperState& hopper_state) :
    pub_map{pub_map},
    mining_controller{node, pub_map, hopper_state},
    offload_controller{node, pub_map, hopper_state},
    traversal_controller{node, pub_map},
    localization_controller{node, pub_map}
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
        default: {}
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
        default: {}
    }
}
