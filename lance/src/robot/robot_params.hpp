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

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>


struct RobotParams
{
    using Box2f = Eigen::AlignedBox2f;

public:
    const float default_stick_deadzone;
    const float driving_magnitude_deadzone;
    const float driving_low_scalar;
    const float driving_medium_scalar;
    const float driving_high_scalar;

    const float trencher_max_velocity_rps;
    const float trencher_mining_velocity_rps;
    const float hopper_belt_max_velocity_rps;
    const float hopper_belt_mining_velocity_rps;
    const float tracks_max_velocity_rps;
    const float tracks_mining_velocity_rps;
    const float tracks_mining_adjustment_range_rps;
    const float tracks_offload_velocity_rps;

    const float hopper_actuator_max_speed;
    const float hopper_actuator_plunge_speed;
    const float hopper_actuator_extract_speed;

    const float hopper_actuator_offload_target;
    const float hopper_actuator_traversal_target;
    const float hopper_actuator_transport_target;
    const float hopper_actuator_mining_target;
    const float hopper_actuator_mining_min;
    const float hopper_actuator_targetting_thresh;

    const float hopper_belt_mining_duty_cycle_base_seconds;

    const float collection_model_initial_volume_liters;
    const float collection_model_capacity_volume_liters;
    const float collection_model_initial_belt_footprint_meters;
    const float collection_model_belt_capacity_meters;
    const float collection_model_belt_offload_length_meters;

    const float preset_mining_traversal_dist_meters;
    const float preset_offload_backup_dist_meters;

    const float iteration_period_seconds;

    const std::string robot_frame_id;
    const std::string odom_frame_id;
    const std::string arena_frame_id;

    Box2f mining_zone_bounds;
    Box2f offload_zone_bounds;

    const float auto_traversal_max_track_velocity_mps;
    const float auto_traversal_max_angular_velocity_rps;
    const float auto_traversal_max_track_acceleration_mpss;
    const float auto_traversal_keypoint_thresh_m;
    const float auto_traversal_max_path_deviation_m;

public:
    RobotParams(rclcpp::Node&);
};
