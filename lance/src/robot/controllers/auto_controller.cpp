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

#include "auto_controller.hpp"

#include <memory>


AutoController::AutoController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    const HopperState& hopper_state) :
    pub_map{pub_map},
    params{params},
    tf_buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)},
    tf_listener{tf_buffer, &node},
    localization_controller{node, pub_map, params, tf_buffer},
    traversal_controller{node, pub_map, params, tf_buffer},
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
    else
    {
        this->localization_controller.initialize();
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
        case Stage::TRAVERSE_TO_MINING:
        case Stage::TRAVERSE_TO_OFFLOAD:
        {
            this->traversal_controller.setCancelled();
            break;
        }
        case Stage::MINING:
        {
            this->mining_controller.setCancelled();
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
        case Stage::UNKNOWN:
        {
            // algo to determine what stage we should be in...
            this->localization_controller.initialize();
            this->stage = Stage::LOCALIZATION;
            [[fallthrough]];
        }
        case Stage::LOCALIZATION:
        {
            this->localization_controller.iterate(motor_status, commands);
            if (!this->localization_controller.isFinished())
            {
                break;
            }

            this->traversal_controller.initializeZone(
                this->params.mining_zone_bounds.min(),
                this->params.mining_zone_bounds.max());
            this->stage = Stage::TRAVERSE_TO_MINING;
            [[fallthrough]];
        }
        TRAVERSE_TO_MINING_L:
        case Stage::TRAVERSE_TO_MINING:
        {
            this->traversal_controller.iterate(motor_status, commands, &joy);
            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            this->mining_controller.initialize();
            this->stage = Stage::MINING;
            [[fallthrough]];
        }
        case Stage::MINING:
        {
            this->mining_controller.iterate(motor_status, commands);
            if (!this->mining_controller.isFinished())
            {
                break;
            }

            this->traversal_controller.initializeZone(
                this->params.offload_zone_bounds.min(),
                this->params.offload_zone_bounds.max());
            this->stage = Stage::TRAVERSE_TO_OFFLOAD;
            [[fallthrough]];
        }
        case Stage::TRAVERSE_TO_OFFLOAD:
        {
            this->traversal_controller.iterate(motor_status, commands, &joy);
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
            this->offload_controller.iterate(motor_status, commands);
            if (!this->offload_controller.isFinished())
            {
                break;
            }

            this->traversal_controller.initializeZone(
                this->params.mining_zone_bounds.min(),
                this->params.mining_zone_bounds.max());
            // chatgpt says I should change this to a while loop that wraps the entire switch-case
            this->stage = Stage::TRAVERSE_TO_MINING;
            goto TRAVERSE_TO_MINING_L;
        }
        default:
        {
        }
    }

    this->publishState();
}

void AutoController::publishState()
{
    static constexpr char const* STAGE_STRINGS[] = {
        "Auto Localization",
        "Auto Traverse To Mining",
        "Auto Mining",
        "Auto Traverse To Offload",
        "Auto Offload",
        "Auto [unknown]"};

    this->pub_map.publish<std_msgs::msg::String, std::string>(
        "/lance/op_status",
        STAGE_STRINGS[static_cast<size_t>(this->stage)]);
}
