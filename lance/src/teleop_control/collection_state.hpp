#pragma once

#include <limits>

#include "motor_interface.hpp"


class HopperState
{
public:
    // set initial params
    void setParams(
        double initial_volume_l,
        double capacity_volume_l,
        double initial_footprint_m,
        double capacity_len_m,
        double offload_len_m);

    void update(double delta_volume_l, double belt_rotations);

public:
    // estimated volume in liters
    inline double volume() const { return this->total_vol_l; }
    // tracked belt position in meters
    inline double beltPosMeters() const { return this->belt_pos_m; }
    // belt position of "head" of regolith pile (closest to trencher)
    inline double startPosMeters() const { return this->high_pos_m; }
    // belt position of "end" of regloith pile (closest to opening)
    inline double endPosMeters() const { return this->low_pos_m; }
    // region of belt occupiled by regolith pile, in meters
    inline double beltUsageMeters() const { return this->occupied_delta_m(); }

    // have we reached the max configured volume
    inline bool isVolCapacity() const
    {
        return this->total_vol_l >= this->cap_vol_l;
    }
    // has the belt reached the end
    inline bool isBeltCapacity() const
    {
        return this->occupied_delta_m() >= this->cap_len_m;
    }

    // output is in motor rotations
    double miningTargetMotorPosition() const;
    // outut is in motor rotations
    double offloadTargetMotorPosition() const;

protected:
    inline double occupied_delta_m() const
    {
        return this->high_pos_m - this->low_pos_m;
    }
    inline double cutoff_pos_m() const
    {
        return this->belt_pos_m - this->offload_len_m;
    }

protected:
    double initial_vol_l = 12.;
    double cap_vol_l = 30.;
    double initial_footprint_m = 0.2;
    double cap_len_m = 0.6;
    double offload_len_m = 0.7;

    double total_vol_l = 0.;
    double belt_pos_m = 0.;
    double high_pos_m = 0.;
    double low_pos_m = 0.;
};

class CollectionState
{
    static constexpr double DOUBLE_UNINITTED_VALUE =
        std::numeric_limits<double>::infinity();

public:
    void setParams(
        double initial_volume_l,
        double capacity_volume_l,
        double initial_footprint_m,
        double capacity_len_m,
        double offload_len_m);

    void update(const RobotMotorStatus& motors_status);

public:
    inline const HopperState& getHopperState() const
    {
        return this->hopper_state;
    }

protected:
    void handleInit(
        const RobotMotorStatus& motors_status,
        double mining_depth,
        double impact_volume);

protected:
    HopperState hopper_state;

    double prev_trencher_rotations = DOUBLE_UNINITTED_VALUE;
    double prev_ltrack_rotations = DOUBLE_UNINITTED_VALUE;
    double prev_rtrack_rotations = DOUBLE_UNINITTED_VALUE;

    double prev_mining_depth = DOUBLE_UNINITTED_VALUE;
    double prev_impact_volume = DOUBLE_UNINITTED_VALUE;
};



// ---

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

class CollectionStatePublisher
{
    using BoolMsg = std_msgs::msg::Bool;
    using Float64Msg = std_msgs::msg::Float64;

public:
    inline CollectionStatePublisher(rclcpp::Node& n) :
        is_vol_cap_pub{n.create_publisher<BoolMsg>(
            "/collection_state/is_full_volume",
            rclcpp::SensorDataQoS{})},
        is_full_occ_pub{n.create_publisher<BoolMsg>(
            "/collection_state/is_full_occ",
            rclcpp::SensorDataQoS{})},
        vol_pub{n.create_publisher<Float64Msg>(
            "/collection_state/volume",
            rclcpp::SensorDataQoS{})},
        mining_target_pub{n.create_publisher<Float64Msg>(
            "/collection_state/mining_target",
            rclcpp::SensorDataQoS{})},
        offload_target_pub{n.create_publisher<Float64Msg>(
            "/collection_state/offload_target",
            rclcpp::SensorDataQoS{})},
        belt_pos_pub{n.create_publisher<Float64Msg>(
            "/collection_state/belt_pos_m",
            rclcpp::SensorDataQoS{})},
        high_pos_pub{n.create_publisher<Float64Msg>(
            "/collection_state/high_pos_m",
            rclcpp::SensorDataQoS{})},
        low_pos_pub{n.create_publisher<Float64Msg>(
            "/collection_state/low_pos_m",
            rclcpp::SensorDataQoS{})},
        belt_usage_pub{n.create_publisher<Float64Msg>(
            "/collection_state/belt_usage_m",
            rclcpp::SensorDataQoS{})}
    {
    }

public:
    inline void publish(const CollectionState& col)
    {
        this->is_vol_cap_pub->publish(
            BoolMsg{}.set__data(col.getHopperState().isVolCapacity()));
        this->is_full_occ_pub->publish(
            BoolMsg{}.set__data(col.getHopperState().isBeltCapacity()));
        this->vol_pub->publish(
            Float64Msg{}.set__data(col.getHopperState().volume()));
        this->mining_target_pub->publish(
            Float64Msg{}.set__data(
                col.getHopperState().miningTargetMotorPosition()));
        this->offload_target_pub->publish(
            Float64Msg{}.set__data(
                col.getHopperState().offloadTargetMotorPosition()));
        this->belt_pos_pub->publish(
            Float64Msg{}.set__data(col.getHopperState().beltPosMeters()));
        this->high_pos_pub->publish(
            Float64Msg{}.set__data(col.getHopperState().startPosMeters()));
        this->low_pos_pub->publish(
            Float64Msg{}.set__data(col.getHopperState().endPosMeters()));
        this->belt_usage_pub->publish(
            Float64Msg{}.set__data(col.getHopperState().beltUsageMeters()));
    }

protected:
    rclcpp::Publisher<BoolMsg>::SharedPtr is_vol_cap_pub, is_full_occ_pub;
    rclcpp::Publisher<Float64Msg>::SharedPtr vol_pub, mining_target_pub,
        offload_target_pub, belt_pos_pub, high_pos_pub, low_pos_pub,
        belt_usage_pub;
};
