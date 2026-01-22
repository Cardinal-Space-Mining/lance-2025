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

#include "robot_controller.hpp"


inline constexpr RobotController::ControlMode getMode(int32_t watchdog)
{
    return watchdog > 0
               ? RobotController::ControlMode::TELEOP
               : (watchdog < 0 ? RobotController::ControlMode::AUTO
                               : RobotController::ControlMode::DISABLED);
}
inline constexpr int encodeTransition(
    RobotController::ControlMode from,
    RobotController::ControlMode to)
{
    return (static_cast<int>(from) << 2) | static_cast<int>(to);
}

template<RobotController::ControlMode FromV, RobotController::ControlMode ToV>
inline constexpr int transition_v = encodeTransition(FromV, ToV);



RobotController::RobotController(RclNode& node, GenericPubMap& pub_map) :
    pub_map{pub_map},
    params{node},
    auto_controller{
        node,
        pub_map,
        params,
        this->collection_state.getHopperState()},
    teleop_controller{
        node,
        pub_map,
        params,
        this->collection_state.getHopperState()}
{
    this->collection_state.setParams(
        this->params.collection_model_initial_volume_liters,
        this->params.collection_model_capacity_volume_liters,
        this->params.collection_model_initial_belt_footprint_meters,
        this->params.collection_model_belt_capacity_meters,
        this->params.collection_model_belt_offload_length_meters);
}

const HopperState& RobotController::hopperState() const
{
    return this->collection_state.getHopperState();
}

const RobotParams& RobotController::getParams() const
{
    return this->params;
}

void RobotController::iterate(
    int32_t watchdog,
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->collection_state.update(motor_status);

    const ControlMode prev_mode = this->control_mode;
    this->control_mode = getMode(watchdog);

    // process transition actions
    switch (encodeTransition(prev_mode, this->control_mode))
    {
        case transition_v<ControlMode::DISABLED, ControlMode::TELEOP>:
        {
            this->teleop_controller.initialize();
            break;
        }
        case transition_v<ControlMode::DISABLED, ControlMode::AUTO>:
        {
            this->auto_controller.initialize();
            break;
        }
        case transition_v<ControlMode::TELEOP, ControlMode::DISABLED>:
        {
            this->teleop_controller.setCancelled();
            break;
        }
        case transition_v<ControlMode::TELEOP, ControlMode::AUTO>:
        {
            this->teleop_controller.setCancelled();
            this->auto_controller.initialize();
            break;
        }
        case transition_v<ControlMode::AUTO, ControlMode::DISABLED>:
        {
            this->auto_controller.setCancelled();
            break;
        }
        case transition_v<ControlMode::AUTO, ControlMode::TELEOP>:
        {
            this->auto_controller.setCancelled();
            this->teleop_controller.initialize();
            break;
        }
        default:
        {
        }
    }

    // process current state actions
    switch (this->control_mode)
    {
        case ControlMode::TELEOP:
        {
            this->teleop_controller.iterate(joy, motor_status, commands);
            break;
        }
        case ControlMode::AUTO:
        {
            this->auto_controller.iterate(joy, motor_status, commands);
            break;
        }
        case ControlMode::DISABLED:
        {
            this->pub_map.publish<std_msgs::msg::String, std::string>(
                "lance/op_status",
                "Disabled");
        }
    }
}
