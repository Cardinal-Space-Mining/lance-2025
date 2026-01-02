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

#include <tf2_ros/buffer.h>

#include <std_srvs/srv/set_bool.hpp>

#include <cardinal_perception/msg/reflector_hint.hpp>

#include "../robot_params.hpp"
#include "../motor_interface.hpp"
#include "../../util/pub_map.hpp"


class LocalizationController
{
    using RclNode = rclcpp::Node;
    using Tf2Buffer = tf2_ros::Buffer;
    using SetBoolSrv = std_srvs::srv::SetBool;
    using ReflectorHintMsg = cardinal_perception::msg::ReflectorHint;
    using GenericPubMap = util::GenericPubMap;

    template<typename T>
    using RclSubPtr = typename rclcpp::Subscription<T>::SharedPtr;
    template<typename T>
    using RclClientPtr = typename rclcpp::Client<T>::SharedPtr;

public:
    LocalizationController(
        RclNode&,
        GenericPubMap&,
        const RobotParams&,
        const Tf2Buffer&);
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
    void setLfdControl(bool enabled);

protected:
    GenericPubMap& pub_map;
    const RobotParams& params;
    const Tf2Buffer& tf_buffer;

    RclSubPtr<ReflectorHintMsg> hint_sub;
    RclClientPtr<SetBoolSrv> lfd_control_client;

    Stage stage{Stage::FINISHED};
    ReflectorHintMsg::ConstSharedPtr last_hint{nullptr};
};
