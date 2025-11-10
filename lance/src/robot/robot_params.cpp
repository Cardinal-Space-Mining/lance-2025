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

#include "robot_params.hpp"

#include "../util/ros_utils.hpp"


using namespace util;


RobotParams::RobotParams(rclcpp::Node& node) :
    default_stick_deadzone{
        declare_and_get_param(node, "default_stick_deadzone", 0.05f)},
    driving_magnitude_deadzone{
        declare_and_get_param(node, "driving_magnitude_deadzone", 0.1f)},
    driving_low_scalar{declare_and_get_param(node, "driving_low_scalar", 0.3f)},
    driving_medium_scalar{
        declare_and_get_param(node, "driving_medium_scalar", 0.7f)},
    driving_high_scalar{
        declare_and_get_param(node, "driving_high_scalar", 1.f)},

    trencher_max_velocity_rps{
        declare_and_get_param(node, "trencher.max_velocity_rps", 80.f)},
    trencher_mining_velocity_rps{
        declare_and_get_param(node, "trencher.mining_velocity_rps", 80.f)},
    hopper_belt_max_velocity_rps{
        declare_and_get_param(node, "hopper_belt.max_velocity_rps", 45.f)},
    hopper_belt_mining_velocity_rps{
        declare_and_get_param(node, "hopper_belt.mining_velocity_rps", 10.f)},
    tracks_max_velocity_rps{
        declare_and_get_param(node, "tracks.max_velocity_rps", 125.f)},
    tracks_mining_velocity_rps{
        declare_and_get_param(node, "tracks.mining_velocity_rps", 8.f)},
    tracks_mining_adjustment_range_rps{
        declare_and_get_param(node, "tracks.mining_adjustment_range_rps", 6.f)},
    tracks_offload_velocity_rps{
        declare_and_get_param(node, "tracks.offload_velocity_rps", 30.f)},

    hopper_actuator_max_speed{
        declare_and_get_param(node, "hopper_actuator.max_speed", 1.f)},
    hopper_actuator_plunge_speed{
        declare_and_get_param(node, "hopper_actuator.plunge_speed", 0.4f)},
    hopper_actuator_extract_speed{
        declare_and_get_param(node, "hopper_actuator.extract_speed", 0.8f)},

    hopper_actuator_offload_target{declare_and_get_param(
        node,
        "hopper_actuator.offload_target_val",
        0.95f)},
    hopper_actuator_traversal_target{declare_and_get_param(
        node,
        "hopper_actuator.traversal_target_val",
        0.6f)},
    hopper_actuator_transport_target{declare_and_get_param(
        node,
        "hopper_actuator.transport_target_val",
        0.55f)},
    hopper_actuator_mining_target{declare_and_get_param(
        node,
        "hopper_actuator.mining_target_val",
        0.21f)},
    hopper_actuator_mining_min{
        declare_and_get_param(node, "hopper_actuator.mining_min_val", 0.03f)},
    hopper_actuator_targetting_thresh{declare_and_get_param(
        node,
        "hopper_actuator.targetting_thresh",
        0.01f)},

    hopper_belt_mining_duty_cycle_base_seconds{declare_and_get_param(
        node,
        "hopper_belt.mining_duty_cycle_base_seconds",
        1.f)},

    collection_model_initial_volume_liters{declare_and_get_param(
        node,
        "collection_model.initial_volume_liters",
        5.f)},
    collection_model_capacity_volume_liters{declare_and_get_param(
        node,
        "collection_model.capacity_volume_liters",
        25.f)},
    collection_model_initial_belt_footprint_meters{declare_and_get_param(
        node,
        "collection_model.initial_belt_footprint_meters",
        0.2f)},
    collection_model_belt_capacity_meters{declare_and_get_param(
        node,
        "collection_model.belt_capacity_meters",
        0.6f)},
    collection_model_belt_offload_length_meters{declare_and_get_param(
        node,
        "collection_model.belt_offload_length_meters",
        0.7f)},

    preset_mining_traversal_dist_meters{declare_and_get_param(
        node,
        "preset_mining_traversal_dist_meters",
        0.25f)},
    preset_offload_backup_dist_meters{
        declare_and_get_param(node, "preset_offload_backup_dist_meters", 0.25f)}
{
}
