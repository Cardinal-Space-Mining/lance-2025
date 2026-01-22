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

#include <limits>
#include <chrono>

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
        GenericPubMap&,
        const RobotParams&,
        const HopperState&);
    ~MiningController() = default;

public:
    /* Restart the routine. If traversal distance is provided,
     * the command will track the travelled distance and end if
     * the traversal distance is exceeded. */
    void initialize(float traversal_dist_m = 0.f);
    /* Check if the command is finished, either as a result
     * of being cancelled or automatically shutting down
     * due to a stop state. */
    bool isFinished();
    /* Mark the command as cancelled, i.e. it will no longer be
     * executed. */
    void setCancelled();

    /* Update the remaining traversal distance. */
    void setRemaining(float traversal_dist_m);
    /* Set whether the hopper model should be used to determine finished state. */
    void setUseHopperModel(bool enabled);

    /* Iterate the controller in "full auto" mode (no user input). */
    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);
    /* Iterate the controller in "assisted" mode (user input). */
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

    struct TraversalState
    {
        void init(float remaining_dist = 0.f);
        void setRemaining(float remaining_dist);
        void updateOdom(float odom);
        bool hasRemaining();

    private:
        float remaining_dist{0.f};
        float prev_odom{0.f};
    };
    struct BeltDutyCycleState
    {
        void setMoved();
        void setStopped();
        bool canMove(float thresh_s);

    private:
        std::chrono::system_clock::time_point prev_belt_stop_time;
        bool belt_moving{false};
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
    BeltDutyCycleState belt_duty_cycle{};
    bool using_hopper_model{true};
};
