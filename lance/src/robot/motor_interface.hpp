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

#include "phoenix_ros_driver/msg/talon_ctrl.hpp"
#include "phoenix_ros_driver/msg/talon_info.hpp"


using TalonCtrlMsg = phoenix_ros_driver::msg::TalonCtrl;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;

/** Contains TalonInfo for each motor */
struct RobotMotorStatus
{
    TalonInfoMsg track_right;
    TalonInfoMsg track_left;
    TalonInfoMsg trencher;
    TalonInfoMsg hopper_belt;
    TalonInfoMsg hopper_actuator;

    inline double getHopperActNormalizedValue() const
    {
        return this->hopper_actuator.position / 1000.;
    }
};

/** Contains TalonCtrl for each motor */
struct RobotMotorCommands
{
    TalonCtrlMsg track_right;
    TalonCtrlMsg track_left;
    TalonCtrlMsg trencher;
    TalonCtrlMsg hopper_belt;
    TalonCtrlMsg hopper_actuator;

    inline void setTracksVelocity(double left_rps, double right_rps)
    {
        this->track_left.set__mode(TalonCtrlMsg::VELOCITY).set__value(left_rps);
        this->track_right.set__mode(TalonCtrlMsg::VELOCITY)
            .set__value(right_rps);
    }
    inline void setTrencherVelocity(double rps)
    {
        this->trencher.set__mode(TalonCtrlMsg::VELOCITY).set__value(rps);
    }
    inline void setHopperBeltVelocity(double rps)
    {
        this->hopper_belt.set__mode(TalonCtrlMsg::VELOCITY).set__value(rps);
    }
    inline void setHopperActPercent(double percent)
    {
        this->hopper_actuator.set__mode(TalonCtrlMsg::PERCENT_OUTPUT)
            .set__value(percent);
    }

    inline void disableTracks()
    {
        this->track_left.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
        this->track_right.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
    }
    inline void disableTrencher()
    {
        this->trencher.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
    }
    inline void disableHopperBelt()
    {
        this->hopper_belt.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
    }
    inline void disableHopperAct()
    {
        this->hopper_actuator.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
    }

    inline void disableAll()
    {
        this->disableTracks();
        this->disableTrencher();
        this->disableHopperBelt();
        this->disableHopperAct();
    }
};
