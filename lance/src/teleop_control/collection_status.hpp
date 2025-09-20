#pragma once

#include <limits>

#include "robot_math.hpp"
#include "motor_interface.hpp"


class HopperState
{
    friend class CollectionStatePublisher;

    static constexpr double DOUBLE_UNINITTED_VALUE =
        std::numeric_limits<double>::infinity();

public:
    inline void setParams(
        double initial_volume_l,
        double capacity_volume_l,
        double initial_footprint_m,
        double capacity_len_m,
        double offload_len_m)
    {
        this->initial_vol_l = initial_volume_l;
        this->cap_vol_l = capacity_volume_l;
        this->initial_footprint_m = initial_footprint_m;
        this->cap_len_m = capacity_len_m;
        this->offload_len_m = offload_len_m;
    }

    inline void update(double delta_volume_l, double belt_rotations)
    {
        this->belt_pos_m = hopper_belt_motor_rps_to_belt_mps(belt_rotations);

        // add new material
        if (delta_volume_l > 0.)
        {
            // set initial footprint if starting fresh
            if (this->total_vol_l <= 0.)
            {
                this->low_pos_m =
                    (this->belt_pos_m - this->initial_footprint_m);
                this->high_pos_m = this->belt_pos_m;
            }
            // handle backwards belt
            else if (this->belt_pos_m > this->high_pos_m)
            {
                this->high_pos_m = this->belt_pos_m;
            }
            this->total_vol_l += delta_volume_l;
        }

        if (this->total_vol_l > 0.)
        {
            // belt moved backwards -- shift positions to mitigate bugs
            if (this->high_pos_m > this->belt_pos_m)
            {
                double occ_delta_m = this->occupied_delta_m();
                this->high_pos_m = this->belt_pos_m;
                this->low_pos_m = this->belt_pos_m - occ_delta_m;
            }

            // handle offloading
            double cutoff_pos_m = this->cutoff_pos_m();

            if (this->high_pos_m < cutoff_pos_m)
            {
                this->total_vol_l = 0.;
                this->high_pos_m = this->low_pos_m = this->belt_pos_m;
            }
            else if (this->low_pos_m < cutoff_pos_m)
            {
                double cutoff_delta_m = (cutoff_pos_m - this->low_pos_m);
                double remainder_proportion =
                    1. - (cutoff_delta_m / this->occupied_delta_m());

                this->total_vol_l *= remainder_proportion;
                this->low_pos_m = cutoff_pos_m;
            }
        }
        else
        {
            this->high_pos_m = this->low_pos_m = this->belt_pos_m;
        }
    }

    inline double volume() const { return this->total_vol_l; }
    inline bool hasVolCapacity() const
    {
        return this->total_vol_l >= this->cap_vol_l;
    }
    inline bool hasFullOccupancy() const
    {
        return this->occupied_delta_m() >= this->cap_len_m;
    }

    inline double miningTargetMotorPosition() const
    {
        if (this->total_vol_l < this->initial_vol_l &&
            this->occupied_delta_m() <= this->initial_footprint_m)
        {
            return hopper_belt_mps_to_motor_rps(this->belt_pos_m);
        }
        else
        {
            return hopper_belt_mps_to_motor_rps(
                std::max(
                    (this->low_pos_m + (std::min(
                                            this->total_vol_l / this->cap_vol_l,
                                            this->cap_vol_l) *
                                        this->cap_len_m)),
                    this->belt_pos_m));
        }
    }
    inline double offloadTargetMotorPosition() const
    {
        return hopper_belt_mps_to_motor_rps(
            std::max(this->high_pos_m + this->offload_len_m, this->belt_pos_m));
    }

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
    double initial_vol_l = 0.;
    double cap_vol_l = 0.;
    double initial_footprint_m = 0.;
    double cap_len_m = 0.;
    double offload_len_m = 0.;

    double total_vol_l = 0.;
    double belt_pos_m = 0.;
    double high_pos_m = 0.;
    double low_pos_m = 0.;
};

class CollectionState
{
    friend class CollectionStatePublisher;

    static constexpr double DOUBLE_UNINITTED_VALUE =
        std::numeric_limits<double>::infinity();

public:
    inline void setParams(
        double initial_volume_l,
        double capacity_volume_l,
        double initial_footprint_m,
        double capacity_len_m,
        double offload_len_m)
    {
        this->hopper_state.setParams(
            initial_volume_l,
            capacity_volume_l,
            initial_footprint_m,
            capacity_len_m,
            offload_len_m);
    }

    inline void update(const RobotMotorStatus& motors_status)
    {
        const double& trencher_rotations = motors_status.trencher.position;
        const double& belt_rotations = motors_status.hopper_belt.position;
        const double& ltrack_rotations = motors_status.track_left.position;
        const double& rtrack_rotations = motors_status.track_right.position;
        double curr_mining_depth_m = linear_actuator_to_mining_depth_clamped(
            motors_status.hopper_actuator.position / 1000.);
        double curr_impact_volume =
            mining_depth_to_trencher_impact_volume(curr_mining_depth_m);

        this->handleInit(
            motors_status,
            curr_mining_depth_m,
            curr_impact_volume);

        // calculate maximum possible volume material transferred given number of trencher rotations
        double delta_trencher_rotations =
            trencher_rotations - this->prev_trencher_rotations;
        double trencher_max_delta_volume =
            trencher_motor_rps_to_max_volume_rate(delta_trencher_rotations);
        // ^ f(r/s) -> l/s <=> f(r) -> l

        // calculate maximum possible volume material 'swept' given change in track rotations (linear distance)
        double avg_mining_depth_m =
            (curr_mining_depth_m + this->prev_mining_depth) * 0.5;
        double avg_track_delta_rotations =
            ((ltrack_rotations - this->prev_ltrack_rotations) +
             (rtrack_rotations - this->prev_rtrack_rotations)) *
            0.5;
        double delta_sweep_volume = track_motor_rps_to_volume_rate(
            avg_track_delta_rotations,
            avg_mining_depth_m);
        // ^ f(m/s) -> l/s <=> f(m) -> l

        // calculate the volume which we have dug into the ground just by lowering the trencher
        double delta_impact_volume =
            std::max(curr_impact_volume - this->prev_impact_volume, 0.);
        // ^ TODO: this will break if trencher is continually actuated up and down in the same spot!

        // calculate transmitted material volume
        double transmitted_volume = std::min(
            (delta_impact_volume + delta_sweep_volume),
            trencher_max_delta_volume);

        this->hopper_state.update(transmitted_volume, belt_rotations);

        this->prev_trencher_rotations = trencher_rotations;
        this->prev_ltrack_rotations = ltrack_rotations;
        this->prev_rtrack_rotations = rtrack_rotations;

        this->prev_mining_depth = curr_mining_depth_m;
        this->prev_impact_volume = curr_impact_volume;
    }

protected:
    inline void handleInit(
        const RobotMotorStatus& motors_status,
        double mining_depth,
        double impact_volume)
    {
        // clang-format off
        #define SET_IF_UNINITTED(var, val)                \
            if ((var) == DOUBLE_UNINITTED_VALUE) (var) = (val);
        // clang-format on

        SET_IF_UNINITTED(
            this->prev_trencher_rotations,
            motors_status.trencher.position)
        SET_IF_UNINITTED(
            this->prev_ltrack_rotations,
            motors_status.track_left.position)
        SET_IF_UNINITTED(
            this->prev_rtrack_rotations,
            motors_status.track_right.position)
        SET_IF_UNINITTED(this->prev_mining_depth, mining_depth)
        SET_IF_UNINITTED(this->prev_impact_volume, impact_volume)

        // clang-format off
        #undef SET_IF_UNINITTED
        // clang-format on
    }

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
            rclcpp::SensorDataQoS{})}
    {
    }

public:
    inline void publish(const CollectionState& col)
    {
        this->is_vol_cap_pub->publish(
            BoolMsg{}.set__data(col.hopper_state.hasVolCapacity()));
        this->is_full_occ_pub->publish(
            BoolMsg{}.set__data(col.hopper_state.hasFullOccupancy()));
        this->vol_pub->publish(
            Float64Msg{}.set__data(col.hopper_state.volume()));
        this->mining_target_pub->publish(
            Float64Msg{}.set__data(
                col.hopper_state.miningTargetMotorPosition()));
        this->offload_target_pub->publish(
            Float64Msg{}.set__data(
                col.hopper_state.offloadTargetMotorPosition()));
        this->belt_pos_pub->publish(
            Float64Msg{}.set__data(col.hopper_state.belt_pos_m));
    }

protected:
    rclcpp::Publisher<BoolMsg>::SharedPtr is_vol_cap_pub, is_full_occ_pub;
    rclcpp::Publisher<Float64Msg>::SharedPtr vol_pub, mining_target_pub,
        offload_target_pub, belt_pos_pub;
};
