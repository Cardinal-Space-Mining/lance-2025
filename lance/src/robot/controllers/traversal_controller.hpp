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

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>

#include <nav_msgs/msg/path.hpp>

#include <cardinal_perception/srv/update_path_planning_mode.hpp>

#include "../robot_params.hpp"
#include "../motor_interface.hpp"
#include "../../util/pub_map.hpp"
#include "../../util/joy_utils.hpp"


class TraversalController
{
    using RclNode = rclcpp::Node;
    using Tf2Buffer = tf2_ros::Buffer;
    using PathMsg = nav_msgs::msg::Path;
    using UpdatePathPlanSrv = cardinal_perception::srv::UpdatePathPlanningMode;
    using JoyState = util::JoyState;
    using GenericPubMap = util::GenericPubMap;

    template<typename T>
    using RclSubPtr = typename rclcpp::Subscription<T>::SharedPtr;
    template<typename T>
    using RclClientPtr = typename rclcpp::Client<T>::SharedPtr;

    using Vec2f = Eigen::Vector2f;
    using Vec3f = Eigen::Vector3f;
    using Quatf = Eigen::Quaternionf;
    using Box2f = Eigen::AlignedBox2f;

public:
    TraversalController(
        RclNode&,
        GenericPubMap&,
        const RobotParams&,
        const Tf2Buffer&);
    ~TraversalController() = default;

public:
    void initializePoint(
        const Vec2f& dest,
        const Vec2f& dest_direction = Vec2f::Zero());
    void initializeZone(const Vec2f& dest_min, const Vec2f& dest_max);

    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands,
        const JoyState* joy = nullptr);

protected:
    enum class State
    {
        INITIALIZATION,
        FOLLOW_PATH,
        REORIENT,
        FINISHED
    };
    enum class DestinationType
    {
        POINT,
        POSE,
        ZONE
    };

protected:
    void initPlanningService(const Vec3f&);
    void stopPlanningService();

    bool computeTraversal(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    GenericPubMap& pub_map;
    const RobotParams& params;
    const Tf2Buffer& tf_buffer;

    RclSubPtr<PathMsg> path_sub;
    RclClientPtr<UpdatePathPlanSrv> pplan_control_client;

    State state{State::FINISHED};

    PathMsg::ConstSharedPtr last_path{nullptr};
    Box2f arena_dest_zone{};
    Vec2f arena_dest_direction{};
    DestinationType destination_type{DestinationType::POINT};
};
