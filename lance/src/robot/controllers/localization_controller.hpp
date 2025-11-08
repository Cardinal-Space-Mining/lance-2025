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
#include "../../util/pub_map.hpp"


class LocalizationController
{
    using RclNode = rclcpp::Node;
    using GenericPubMap = util::GenericPubMap;

public:
    LocalizationController(RclNode&, const GenericPubMap&, const RobotParams&);
    ~LocalizationController() = default;

public:
    void initialize();
    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        INITIALIZATION,
        SEARCHING,
        TARGETTING,
        FINISHED
    };

protected:
    const GenericPubMap& pub_map;
    const RobotParams& params;

    Stage stage{Stage::FINISHED};
};


// --- Implementation ----------------------------------------------------------

LocalizationController::LocalizationController(
    RclNode& node,
    const GenericPubMap& pub_map,
    const RobotParams& params) :
    pub_map{pub_map},
    params{params}
{
}

void LocalizationController::initialize()
{
    this->stage = Stage::INITIALIZATION;
}

bool LocalizationController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void LocalizationController::setCancelled() {}

void LocalizationController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
        }
        case Stage::SEARCHING:
        {
        }
        case Stage::TARGETTING:
        {
        }
        case Stage::FINISHED:
        {
        }
    }
}
