#pragma once

#include <rclcpp/rclcpp.hpp>


struct RobotParams
{
public:
    float default_stick_deadzone;
    float driving_magnitude_deadzone;
    float driving_low_scalar;
    float driving_medium_scalar;
    float driving_high_scalar;

    float trencher_max_velocity_rps;
    float trencher_mining_velocity_rps;
    float hopper_belt_max_velocity_rps;
    float hopper_belt_mining_velocity_rps;
    float tracks_max_velocity_rps;
    float tracks_mining_velocity_rps;
    float tracks_mining_adjustment_range_rps;
    float tracks_offload_velocity_rps;

    float hopper_actuator_max_speed;
    float hopper_actuator_plunge_speed;
    float hopper_actuator_extract_speed;

    float hopper_actuator_offload_target;
    float hopper_actuator_traversal_target;
    float hopper_actuator_transport_target;
    float hopper_actuator_mining_target;
    float hopper_actuator_mining_min;
    float hopper_actuator_targetting_thresh;

    float hopper_belt_mining_duty_cycle_base_seconds;

    float collection_model_initial_volume_liters;
    float collection_model_capacity_volume_liters;
    float collection_model_initial_belt_footprint_meters;
    float collection_model_belt_capacity_meters;
    float collection_model_belt_offload_length_meters;

public:
    RobotParams(rclcpp::Node&);
};


// --- Implementation ----------------------------------------------------------

RobotParams::RobotParams(rclcpp::Node& node) :
    default_stick_deadzone{
        node.get_parameter_or("default_stick_deadzone", 0.05f)},
    driving_magnitude_deadzone{
        node.get_parameter_or("driving_magnitude_deadzone", 0.1f)},
    driving_low_scalar{node.get_parameter_or("driving_low_scalar", 0.3f)},
    driving_medium_scalar{node.get_parameter_or("driving_medium_scalar", 0.7f)},
    driving_high_scalar{node.get_parameter_or("driving_high_scalar", 1.f)},

    trencher_max_velocity_rps{
        node.get_parameter_or("trencher.max_velocity_rps", 80.f)},
    trencher_mining_velocity_rps{
        node.get_parameter_or("trencher.mining_velocity_rps", 80.f)},
    hopper_belt_max_velocity_rps{
        node.get_parameter_or("hopper_belt.max_velocity_rps", 45.f)},
    hopper_belt_mining_velocity_rps{
        node.get_parameter_or("hopper_belt.mining_velocity_rps", 10.f)},
    tracks_max_velocity_rps{
        node.get_parameter_or("tracks.max_velocity_rps", 125.f)},
    tracks_mining_velocity_rps{
        node.get_parameter_or("tracks.mining_velocity_rps", 8.f)},
    tracks_mining_adjustment_range_rps{
        node.get_parameter_or("tracks.mining_adjustment_range_rps", 6.f)},
    tracks_offload_velocity_rps{
        node.get_parameter_or("tracks.offload_velocity_rps", 30.f)},

    hopper_actuator_max_speed{
        node.get_parameter_or("hopper_actuator.max_speed", 1.f)},
    hopper_actuator_plunge_speed{
        node.get_parameter_or("hopper_actuator.plunge_speed", 0.4f)},
    hopper_actuator_extract_speed{
        node.get_parameter_or("hopper_actuator.extract_speed", 0.8f)},

    hopper_actuator_offload_target{
        node.get_parameter_or("hopper_actuator.offload_target_val", 0.95f)},
    hopper_actuator_traversal_target{
        node.get_parameter_or("hopper_actuator.traversal_target_val", 0.6f)},
    hopper_actuator_transport_target{
        node.get_parameter_or("hopper_actuator.transport_target_val", 0.55f)},
    hopper_actuator_mining_target{
        node.get_parameter_or("hopper_actuator.mining_target_val", 0.21f)},
    hopper_actuator_mining_min{
        node.get_parameter_or("hopper_actator.mining_min_val", 0.03f)},
    hopper_actuator_targetting_thresh{
        node.get_parameter_or("hopper_actuator.targetting_thresh", 0.01f)},

    hopper_belt_mining_duty_cycle_base_seconds{node.get_parameter_or(
        "hopper_belt.mining_duty_cycle_base_seconds",
        1.f)},

    collection_model_initial_volume_liters{
        node.get_parameter_or("collection_model.initial_volume_liters", 5.f)},
    collection_model_capacity_volume_liters{
        node.get_parameter_or("collection_model.capacity_volume_liters", 25.f)},
    collection_model_initial_belt_footprint_meters{node.get_parameter_or(
        "collection_model.initial_belt_footprint_meters",
        0.2f)},
    collection_model_belt_capacity_meters{
        node.get_parameter_or("collection_model.belt_capacity_meters", 0.6f)},
    collection_model_belt_offload_length_meters{node.get_parameter_or(
        "collection_model.belt_offload_length_meters",
        0.7f)}
{
}
