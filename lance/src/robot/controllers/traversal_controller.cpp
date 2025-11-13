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

#include "traversal_controller.hpp"

#include <chrono>
#include <memory>

#include "../../util/geometry.hpp"
#include "../../util/ros_utils.hpp"


#define PERCEPTION_PATH_TOPIC "/cardinal_perception/planned_path"
#define PERCEPTION_PPLAN_CONTROL_TOPIC          \
    "/cardinal_perception/update_path_planning"
#define ARENA_FRAME_ID "map"
#define ROBOT_FRAME_ID "base_link"

using system_clock = std::chrono::system_clock;
using namespace util::geom::cvt::ops;

using Iso3f = Eigen::Isometry3f;


TraversalController::TraversalController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    const Tf2Buffer& tf_buffer) :
    pub_map{pub_map},
    params{params},
    tf_buffer{tf_buffer},
    path_sub{node.create_subscription<PathMsg>(
        PERCEPTION_PATH_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const PathMsg::ConstSharedPtr& msg) { this->last_path = msg; })},
    pplan_control_client{
        node.create_client<UpdatePathPlanSrv>(PERCEPTION_PPLAN_CONTROL_TOPIC)}
{
}

void TraversalController::initialize(const Vec2f& dest)
{
    this->using_zone = false;

    this->initPlanningService(Vec3f{dest.x(), dest.y(), 0.f});
    this->last_path = nullptr;

    this->state = State::INITIALIZATION;
}
void TraversalController::initialize(
    const Vec2f& dest_min,
    const Vec2f& dest_max)
{
    this->dest_zone.min().template head<2>() = dest_min;
    this->dest_zone.min().z() = -std::numeric_limits<float>::infinity();
    this->dest_zone.max().template head<2>() = dest_max;
    this->dest_zone.max().z() = std::numeric_limits<float>::infinity();
    this->using_zone = true;

    this->initPlanningService(
        Vec3f{
            (dest_min.x() + dest_max.x()) * 0.5f,
            (dest_min.y() + dest_max.y()) * 0.5f,
            0.f});
    this->last_path = nullptr;

    this->state = State::INITIALIZATION;
}
void TraversalController::initialize(const Vec3f& dest)
{
    this->using_zone = false;

    this->initPlanningService(dest);
    this->last_path = nullptr;

    this->state = State::INITIALIZATION;
}

bool TraversalController::isFinished()
{
    return this->state == State::FINISHED;
}

void TraversalController::setCancelled()
{
    this->stopPlanningService();

    this->state = State::FINISHED;
}

void TraversalController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->state)
    {
        case State::INITIALIZATION:
        {
            if (!this->last_path)
            {
                break;
            }

            this->state = State::TRAVERSING;
            [[fallthrough]];
        }
        case State::TRAVERSING:
        {
            this->computeTraversal(commands);
        }
        case State::FINISHED:
        {
            this->stopPlanningService();
        }
    }
}

void TraversalController::initPlanningService(const Vec3f& dest)
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->target.header.frame_id = ARENA_FRAME_ID;
    req->target.header.stamp = util::toTimeStamp(system_clock::now());
    req->target.pose.position.x = dest.x();
    req->target.pose.position.y = dest.y();
    req->target.pose.position.z = dest.z();
    req->completed = false;

    this->pplan_control_client->async_send_request(
        req,
        [](rclcpp::Client<UpdatePathPlanSrv>::SharedFuture) {});
}
void TraversalController::stopPlanningService()
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->completed = true;

    this->pplan_control_client->async_send_request(
        req,
        [](rclcpp::Client<UpdatePathPlanSrv>::SharedFuture) {});
}

void TraversalController::computeTraversal(RobotMotorCommands& commands)
{
    // 1. OBTAIN KEYPOINTS RELATIVE TO BASE LINK
    std::vector<Vec3f> keypoints_local;
    keypoints_local.resize(this->last_path->poses.size());

    for (size_t i = 0; i < keypoints_local.size(); i++)
    {
        keypoints_local[i] << this->last_path->poses[i].pose.position;
    }

    if (this->last_path->header.frame_id != ROBOT_FRAME_ID)
    {
        try
        {
            Iso3f tf;
            tf << this->tf_buffer
                      .lookupTransform(
                          ROBOT_FRAME_ID,
                          this->last_path->header.frame_id,
                          tf2::TimePointZero)
                      .transform;

            for (Vec3f& p : keypoints_local)
            {
                p = tf * p;
            }
        }
        catch (const std::exception& e)
        {
            // failed to transform to robot frame
            return;
        }
    }

    // 2. ???
    float dist = 0.f;
    size_t beg_idx = 0;
    size_t end_idx = 0;
    for(size_t i = 1; i < keypoints_local.size(); i++)
    {
        const Vec2f& prev = keypoints_local[i - 1];
        const Vec2f& curr = keypoints_local[i];

        
    }
}
