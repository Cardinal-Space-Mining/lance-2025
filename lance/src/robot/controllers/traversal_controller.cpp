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

#include "../robot_math.hpp"
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

void TraversalController::initializePoint(const Vec2f& dest, const Vec2f& dir)
{
    this->last_path = nullptr;
    this->arena_dest_direction = dir.normalized();
    this->destination_type = dir.squaredNorm() > 0.f ? DestinationType::POSE
                                                     : DestinationType::POINT;

    this->initPlanningService(Vec3f{dest.x(), dest.y(), 0.f});

    this->state = State::INITIALIZATION;
}
void TraversalController::initializeZone(
    const Vec2f& dest_min,
    const Vec2f& dest_max)
{
    this->last_path = nullptr;
    this->arena_dest_zone.min() = dest_min;
    this->arena_dest_zone.max() = dest_max;
    this->arena_dest_direction = Vec2f::Zero();
    this->destination_type = DestinationType::ZONE;

    this->initPlanningService(
        Vec3f{
            (dest_min.x() + dest_max.x()) * 0.5f,
            (dest_min.y() + dest_max.y()) * 0.5f,
            0.f});

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
            this->computeTraversal(motor_status, commands);
            break;
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

void TraversalController::computeTraversal(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    constexpr float LOOKAHEAD_PATH_DISTANCE = 0.25f;
    constexpr float TARGETTING_HEADING_THRESH = std::numbers::pi_v<float> / 4.f;
    constexpr float KEYPOINT_THRESH = 0.05f;
    constexpr float NOMINAL_VELOCITY = 0.25f;

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

    // 2. FIND TARGET SEGMENT OR KEYPOINT
    size_t seg_beg_idx = 0;
    size_t seg_end_idx = 0;
    float seg_proj_t = 0.f;
    float seg_proj_dist = 0.f;
    for (size_t i = 1; i < keypoints_local.size(); i++)
    {
        const auto prev = keypoints_local[i - 1].template head<2>();
        const auto curr = keypoints_local[i].template head<2>();

        // project the robot base onto the segment formed by the current
        // two keypoints
        Vec2f diff = curr - prev;
        seg_proj_t = (diff.dot(-prev)) / diff.squaredNorm();

        // proj_t > 1.f --> "after" second keypoint
        // proj_t = 1.f --> at second keypoint
        // proj_t = 0.f --> at first keypoint
        // proj_t < 0.f --> "before" first keypoint
        if (seg_proj_t < 1.f)   // "before" second keypoint
        {
            seg_end_idx = i;
            seg_beg_idx = i - 1;
            seg_proj_dist = (prev + diff * seg_proj_t).norm();
            break;
        }
        else
        {
            seg_beg_idx = seg_end_idx = i;
        }
    }

    // 3. ALGO
    if (seg_beg_idx == seg_end_idx)
    {
        // target final keypoint
    }
    else
    {
        // if seg_proj_dist > thresh (off path), target directly to segment
        // otherwise, lookahead for target and analyze deceleration topology

        // run algo
        const double fb_l_vel_mps =
            track_motor_rps_to_ground_mps(motor_status.track_left.velocity);
        const double fb_r_vel_mps =
            track_motor_rps_to_ground_mps(motor_status.track_right.velocity);
        const double avg_vel_mps = (fb_l_vel_mps + fb_r_vel_mps) * 0.5f;
        const double decell_dist_m =
            1.5f * avg_vel_mps * avg_vel_mps /
            this->params.auto_traversal_max_acceleration_mpss;
        const double target_dist_m =
            avg_vel_mps * this->params.iteration_period_seconds;

        // find target point:
        // if robot is before first segment keypoint, interpolate along seg or add distance to first keypoint
        // loop subsequent keypoints, once dist to 1st < target dist < dist to 2nd, interpolate seg

        // 
    }
}
