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

#include "traversal_controller.hpp"

#include <chrono>
#include <memory>
#include <sstream>
#include <iostream>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "../robot_math.hpp"
#include "../hid_bindings.hpp"
#include "../../util/geometry.hpp"
#include "../../util/time_cvt.hpp"


#define PERCEPTION_PATH_TOPIC "/cardinal_perception/planned_path"
#define PERCEPTION_PPLAN_CONTROL_TOPIC          \
    "/cardinal_perception/update_path_planning"

using system_clock = std::chrono::system_clock;
using namespace util::geom::cvt::ops;

using Iso3f = Eigen::Isometry3f;



void debugTracksControl(
    const util::JoyState& joy,
    const RobotParams& params,
    RobotMotorCommands& commands)
{
    using namespace Bindings;

    const float x = TeleopDriveXAxis::rawValue(joy);
    const float y = TeleopDriveYAxis::rawValue(joy);
    if ((x * x + y * y) <
        (params.driving_magnitude_deadzone * params.driving_magnitude_deadzone))
    {
        commands.setTracksVelocity(0., 0.);
    }
    else
    {
        const float l = y - x;  // y + (-x)
        const float r = y + x;  // y - (-x)
        const float s =
            ((params.tracks_max_velocity_rps * params.driving_medium_scalar) /
             std::max({1.f, std::abs(l), std::abs(r)}));
        commands.setTracksVelocity((l * s), (r * s));
    }
}

// compute the coefficient representing the ratio of radius-to-corner-
// deviation-width when turning at a path junction
template<typename FloatT>
inline FloatT computeJunctionRadiusCoeff(
    FloatT cos_theta)  // sqrt is not constexpr until C++26 :(
{
    const FloatT cos_half_theta =
        std::sqrt((FloatT)0.5 + cos_theta * (FloatT)0.5);
    return cos_half_theta / ((FloatT)1 - cos_half_theta);
}
// compute the maximum starting velocity such that it is still possible to decelerate
// to the target velocity in the given distance at the given max decelleration
template<typename FloatT>
inline FloatT
    backpropegateMaxVelocity(FloatT end_vel, FloatT dist, FloatT max_acc)
{
    // v_f^2 = v_i^2 + 2*a*x
    return std::sqrt((FloatT)2 * dist * max_acc + end_vel * end_vel);
}



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
    RobotMotorCommands& commands,
    const JoyState* joy)
{
    switch (this->state)
    {
        case State::INITIALIZATION:
        {
            if (!this->last_path)
            {
                break;
            }

            this->state = State::FOLLOW_PATH;
            [[fallthrough]];
        }
        case State::FOLLOW_PATH:
        {
            if (!this->computeTraversal(motor_status, commands))
            {
                break;
            }
            [[fallthrough]];
        }
        case State::REORIENT:
        {
            [[fallthrough]];
        }
        case State::FINISHED:
        {
            this->stopPlanningService();
        }
    }

    // if (joy)
    // {
    //     debugTracksControl(*joy, this->params, commands);
    // }
}

void TraversalController::initPlanningService(const Vec3f& dest)
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->target.header.frame_id = this->params.arena_frame_id;
    req->target.header.stamp = util::toTimeMsg(system_clock::now());
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



bool TraversalController::computeTraversal(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    // 1. OBTAIN KEYPOINTS RELATIVE TO BASE LINK
    std::vector<Vec2f> keypoints;
    keypoints.resize(this->last_path->poses.size());

    if (this->last_path->header.frame_id != this->params.robot_frame_id)
    {
        try
        {
            Iso3f tf;
            tf << this->tf_buffer
                      .lookupTransform(
                          this->params.robot_frame_id,
                          this->last_path->header.frame_id,
                          tf2::TimePointZero)
                      .transform;

            Vec3f tmp;
            for (size_t i = 0; i < keypoints.size(); i++)
            {
                const auto& pt = this->last_path->poses[i].pose.position;
                keypoints[i] = (tf * (tmp << pt)).template head<2>();
            }
        }
        catch (const std::exception& e)
        {
            // failed to transform to robot frame
            // std::cout
            //     << "--- TRAVERSAL ITERATION ---\nFailed to transform keypoints to robot frame\n"
            //     << std::endl;
            return false;
        }
    }
    else
    {
        for (size_t i = 0; i < keypoints.size(); i++)
        {
            const auto& pt = this->last_path->poses[i].pose.position;
            keypoints[i].x() = static_cast<float>(pt.x);
            keypoints[i].y() = static_cast<float>(pt.y);
        }
    }

    if (keypoints.back().norm() <=
        this->params.auto_traversal_keypoint_thresh_m)
    {
        commands.disableTracks();
        return true;
    }

    // 2. FIND TARGET SEGMENT OR KEYPOINT
    size_t seg_beg_idx = 0;
    size_t seg_end_idx = 0;
    float seg_proj_t = 0.f;
    float seg_proj_dist = 0.f;
    for (size_t i = 1; i < keypoints.size(); i++)
    {
        const auto& prev = keypoints[i - 1];
        const auto& curr = keypoints[i];

        // project the robot base onto the segment formed by the current
        // two keypoints
        Vec2f diff = curr - prev;
        seg_proj_t = (diff.dot(-prev)) / diff.squaredNorm();

        // proj_t > 1.f --> "after" second keypoint
        // proj_t = 1.f --> at second keypoint
        // proj_t = 0.f --> at first keypoint
        // proj_t < 0.f --> "before" first keypoint
        if (seg_proj_t < 1.f)  // "before" second keypoint
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
    Vec2f target_pt;
    float backprop_max_vel = std::numeric_limits<float>::infinity();
    // float lv_max = this->params.auto_traversal_max_track_velocity_mps;

    // std::ostringstream os;
    // os << "--- TRAVERSAL ITERATION ---"
    //     "\n#kp : " << keypoints.size() <<
    //     "\nmatched seg : (" << seg_beg_idx << ", " << seg_end_idx << ")"
    //     "\nseg proj t : " << seg_proj_t <<
    //     "\nseg proj dist : " << seg_proj_dist;

    // a. target final keypoint
    if (seg_beg_idx == seg_end_idx)
    {
        target_pt = keypoints[seg_end_idx];
        backprop_max_vel = backpropegateMaxVelocity(
            0.f,
            target_pt.norm(),
            this->params.auto_traversal_max_track_acceleration_mpss);

        // os << "\ncontrol mode : \"final keypoint\"";
    }
    // b. off the path
    else if (seg_proj_dist > this->params.auto_traversal_max_path_deviation_m)
    {
        // target segment projection
        const auto& a = keypoints[seg_beg_idx];
        const auto& b = keypoints[seg_end_idx];
        target_pt = a + (b - a) * seg_proj_t;

        constexpr float PERP_RADIUS_COEFF = 2.41421356237f;  // 1 + sqrt(2)
        const float pt_vmax =
            this->params.auto_traversal_max_angular_velocity_rps *
            this->params.auto_traversal_max_path_deviation_m *
            PERP_RADIUS_COEFF;
        backprop_max_vel = backpropegateMaxVelocity(
            pt_vmax,
            seg_proj_dist,
            this->params.auto_traversal_max_track_acceleration_mpss);

        // os <<
        //     "\ncontrol mode : \"return to path\""
        //     "\njunction vmax : " << pt_vmax;
    }
    // c. follow the path
    else
    {
        const double fb_l_vel_mps =
            track_motor_rps_to_ground_mps(motor_status.track_left.velocity);
        const double fb_r_vel_mps =
            track_motor_rps_to_ground_mps(motor_status.track_right.velocity);
        const double avg_vel_mps = (fb_l_vel_mps + fb_r_vel_mps) * 0.5f;

        const float max_iter_vel =
            static_cast<float>(avg_vel_mps) +
            (this->params.auto_traversal_max_track_acceleration_mpss *
             this->params.iteration_period_seconds);
        const float decell_dist_m =
            1.5f * max_iter_vel * max_iter_vel /
            this->params.auto_traversal_max_track_acceleration_mpss;
        const float target_dist_m =
            max_iter_vel * this->params.iteration_period_seconds;

        // os <<
        //     "\ncontrol mode : \"follow path\""
        //     "\nmax iter vel : " << max_iter_vel <<
        //     "\ndecell dist : " << decell_dist_m <<
        //     "\ntarget dist : " << target_dist_m;

        target_pt = keypoints.back();

        Vec2f prev = Vec2f::Zero();
        float dist = 0.f;
        for (size_t i = (seg_proj_t < 0.f ? seg_beg_idx : seg_end_idx);
             (i < keypoints.size()) &&
             (dist < target_dist_m || dist < decell_dist_m);
             i++)
        {
            const auto& curr = keypoints[i];
            const Vec2f prev_diff = (curr - prev);
            const float mag = prev_diff.norm();

            if ((dist < target_dist_m) && (target_dist_m < dist + mag))
            {
                float seg_t = (target_dist_m - dist) / mag;
                target_pt = prev + prev_diff * seg_t;
            }

            if (dist + mag < decell_dist_m)
            {
                if (i + 1 < keypoints.size())
                {
                    const auto& next = keypoints[i + 1];
                    const Vec2f next_diff = (next - curr);

                    // max corner-cutting radius over max deviation coeff
                    const float s = computeJunctionRadiusCoeff(
                        prev_diff.normalized().dot(next_diff.normalized()));
                    // max radius at this junction
                    const float r =
                        s * this->params.auto_traversal_max_path_deviation_m;
                    // max vel = radius * max angular velocity
                    const float pt_vmax =
                        r *
                        this->params.auto_traversal_max_angular_velocity_rps;
                    // backpropegated max current velocity to "make the turn"
                    const float bp_vmax = backpropegateMaxVelocity(
                        pt_vmax,
                        dist + mag,
                        this->params
                            .auto_traversal_max_track_acceleration_mpss);

                    backprop_max_vel = std::min(backprop_max_vel, bp_vmax);
                }
                else
                {
                    // decelerate to stop at final keypoint
                    const float bp_vmax = backpropegateMaxVelocity(
                        0.f,
                        (dist + mag),
                        this->params
                            .auto_traversal_max_track_acceleration_mpss);

                    backprop_max_vel = std::min(backprop_max_vel, bp_vmax);
                }
            }

            dist += mag;
        }
    }

    const float max_vel = std::min(
        backprop_max_vel,
        this->params.auto_traversal_max_track_velocity_mps);
    const Vec2f dir =
        Vec2f{target_pt.x() * 10.f, std::max(0.f, target_pt.y())}.normalized();

    // relative direction to track proportions
    const float l1 = dir.y() + dir.x();
    const float r1 = dir.y() - dir.x();
    // renormalize and scale by max vel
    const float s = max_vel / std::max({1.f, std::abs(l1), std::abs(r1)});
    const float l2 = l1 * s;
    const float r2 = r1 * s;
    // compute normalization to constrain angular velocity
    const float a = std::max(
        1.f,
        std::abs(
            (r2 - l2) /
            (static_cast<float>(TRACK_SEPARATION_M) *
             this->params.auto_traversal_max_angular_velocity_rps)));
    // apply results
    commands.setTracksVelocity(
        ground_mps_to_track_motor_rps(l2 / a),
        ground_mps_to_track_motor_rps(r2 / a));

    // os <<
    //     "\ntarget pt : (x: " << target_pt.x() << ", y: " << target_pt.y() << ")"
    //     "\nbp max vel : " << backprop_max_vel << "\n";
    // std::cout << os.str() << std::endl;

    // geometry_msgs::msg::PoseStamped pub_pt;
    // pub_pt.pose.position.x = target_pt.x();
    // pub_pt.pose.position.y = target_pt.y();
    // pub_pt.pose.position.z = 0.;
    // pub_pt.header.frame_id = this->params.robot_frame_id;
    // this->pub_map.publish("traversal_target_point", pub_pt);

    return false;
}
