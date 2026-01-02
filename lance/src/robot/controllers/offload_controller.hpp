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
        GenericPubMap&,
        const RobotParams&,
        const HopperState&);
    ~OffloadController() = default;

public:
    void initialize(float traversal_dist_m = 0.f);
    bool isFinished();
    void setCancelled();

    void setUseHopperModel(bool enabled);

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
    GenericPubMap& pub_map;
    const RobotParams& params;
    const HopperState& hopper_state;

    Stage stage{Stage::FINISHED};
    TraversalState traversal_state{};
    double calculated_target_belt_pos{0.};
    bool using_hopper_model{true};
};
