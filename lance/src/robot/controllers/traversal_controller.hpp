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


class TraversalController
{
    using RclNode = rclcpp::Node;
    using Tf2Buffer = tf2_ros::Buffer;
    using PathMsg = nav_msgs::msg::Path;
    using UpdatePathPlanSrv = cardinal_perception::srv::UpdatePathPlanningMode;
    using GenericPubMap = util::GenericPubMap;

    template<typename T>
    using RclSubPtr = typename rclcpp::Subscription<T>::SharedPtr;
    template<typename T>
    using RclClientPtr = typename rclcpp::Client<T>::SharedPtr;

    using Vec2f = Eigen::Vector2f;
    using Vec3f = Eigen::Vector3f;
    using Quatf = Eigen::Quaternionf;
    using Box3f = Eigen::AlignedBox3f;

public:
    TraversalController(
        RclNode&,
        GenericPubMap&,
        const RobotParams&,
        const Tf2Buffer&);
    ~TraversalController() = default;

public:
    void initialize(const Vec2f& dest);
    void initialize(
        const Vec2f& dest_min,
        const Vec2f& dest_max);
    void initialize(const Vec3f& dest);

    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class State
    {
        INITIALIZATION,
        TRAVERSING,
        FINISHED
    };

protected:
    void initPlanningService(const Vec3f&);
    void stopPlanningService();

    void computeTraversal(RobotMotorCommands& commands);

protected:
    GenericPubMap& pub_map;
    const RobotParams& params;
    const Tf2Buffer& tf_buffer;

    RclSubPtr<PathMsg> path_sub;
    RclClientPtr<UpdatePathPlanSrv> pplan_control_client;

    State state{State::FINISHED};
    PathMsg::ConstSharedPtr last_path{nullptr};
    Box3f dest_zone{};
    bool using_zone{false};
};
