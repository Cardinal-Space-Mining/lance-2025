#pragma once

#include <math.h>

static constexpr double TRACK_GEARING = 64.;
static constexpr double TRACK_EFFECTIVE_OUTPUT_RADIUS_M = 0.07032851;

static constexpr double TRENCHER_WIDTH_M = 0.254;
static constexpr double TRENCHER_GEARING = 32.;
static constexpr double TRENCHER_LITERS_PER_OUTPUT_ROTATION = (0.04096766 * 6.);
static constexpr double TRENCHER_IMPACT_EFFECTIVE_RADIUS = 0.09270911;

static constexpr double HOPPER_BELT_GEARING = 30.;
static constexpr double HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M = 0.0508;
static constexpr double HOPPER_BELT_CONTAINER_LENGTH_M = 0.6;
static constexpr double HOPPER_BELT_SAFE_OFFLOAD_DIST_M = 0.7;
static constexpr double CONSERVATIVE_HOPPER_CAPACITY = 30.;

static constexpr double MINING_DEPTH_FX_OFFSET = 0.112918;
static constexpr double MINING_DEPTH_FX_SLOPE = -0.315855;


template<typename T>
constexpr inline T track_motor_rps_to_ground_mps(const T& rps)
{
    return rps *
           ((1. / TRACK_GEARING) * TRACK_EFFECTIVE_OUTPUT_RADIUS_M * 2 * M_PI);
}
template<typename T>
constexpr inline T ground_mps_to_track_motor_rps(const T& mps)
{
    return mps *
           (1. / (TRACK_EFFECTIVE_OUTPUT_RADIUS_M * 2 * M_PI) * TRACK_GEARING);
}

template<typename T>
constexpr inline T linear_actuator_to_mining_depth_unclamped(
    const T& actuator_normalized_pos)
{
    return MINING_DEPTH_FX_OFFSET +
           MINING_DEPTH_FX_SLOPE * actuator_normalized_pos;
}
template<typename T>
constexpr inline T mining_depth_to_linear_actuator_unclamped(const T& depth_m)
{
    return (depth_m - MINING_DEPTH_FX_OFFSET) * (1. / MINING_DEPTH_FX_SLOPE);
}
// Domain/range: [0.03, ~0.35] -> [0m, 0.1016m (4in)]
template<typename T>
constexpr inline T linear_actuator_to_mining_depth_clamped(
    const T& actuator_normalized_pos)
{
    return std::clamp(
        linear_actuator_to_mining_depth_unclamped(actuator_normalized_pos),
        0.,
        0.1016);
}
// Domain/range: [0m, 0.1016m (4in)] -> [0.03, ~0.35]
template<typename T>
constexpr inline T mining_depth_to_linear_actuator_clamped(const T& depth_m)
{
    return mining_depth_to_linear_actuator_unclamped(
        std::clamp(depth_m, 0., 0.1016));
}

template<typename T>
constexpr inline T mining_depth_to_trencher_impact_volume(const T& depth_m)
{
    constexpr double R = TRENCHER_IMPACT_EFFECTIVE_RADIUS;
    constexpr double R2 = (R * R);

    if (depth_m <= 0.)
    {
        return 0.;
    }
    else if (depth_m < R)
    {
        double x = (R - depth_m);
        double cross_section_area =
            (R2 * acos(x / R) - x * std::sqrt(R2 - x * x));
        // cross-section * width * 1000 liters/m^3
        return cross_section_area * TRENCHER_WIDTH_M * 1000.;
    }
    else
    {
        // (full semi-circle cross-section + additional depth rect) * width * 1000 liters/m^3
        return ((M_PI * R2) + ((depth_m - R) * TRENCHER_WIDTH_M * R)) * 1000.;
    }
}

template<typename T>
constexpr inline T track_motor_rps_to_volume_rate(
    const T& motor_rps,
    const T& depth_m)
{
    // dist * depth * width * 1000 liters/m^3
    return track_motor_rps_to_ground_mps(motor_rps) * depth_m *
           (TRENCHER_WIDTH_M * 1000.);
}
template<typename T>
constexpr inline T volume_rate_to_track_motor_rps(
    const T& vol_rate_lps,
    const T& depth_m)
{
    // (vol rate * 0.001 m^3/liter) / (depth * width)
    return ground_mps_to_track_motor_rps(
        (vol_rate_lps / depth_m) * ((1. / TRENCHER_WIDTH_M) / 1000.));
}

template<typename T>
constexpr inline T trencher_motor_rps_to_max_volume_rate(const T& rps)
{
    return rps *
           ((1. / TRENCHER_GEARING) * TRENCHER_LITERS_PER_OUTPUT_ROTATION);
}
template<typename T>
constexpr inline T target_vol_rate_to_trencher_motor_rps(const T& vol_rate_lps)
{
    return vol_rate_lps *
           ((1. / TRENCHER_LITERS_PER_OUTPUT_ROTATION) * TRENCHER_GEARING);
}

template<typename T>
constexpr inline T trencher_motor_rps_to_max_track_motor_rps(
    const T& trencher_rps,
    const T& depth_m)
{
    return volume_rate_to_track_motor_rps(
        trencher_motor_rps_to_max_volume_rate(trencher_rps),
        depth_m);
}
template<typename T>
constexpr inline T track_motor_rps_to_trencher_motor_rps(
    const T& track_rps,
    const T& depth_m)
{
    return target_vol_rate_to_trencher_motor_rps(
        track_motor_rps_to_volume_rate(track_rps, depth_m));
}

template<typename T>
constexpr inline T volume_rate_to_hopper_full_time(const T& vol_rate_lps)
{
    return CONSERVATIVE_HOPPER_CAPACITY / vol_rate_lps;
}

template<typename T>
constexpr inline T hopper_belt_motor_rps_to_belt_mps(const T& motor_rps)
{
    return motor_rps * ((1. / HOPPER_BELT_GEARING) *
                        HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M * 2 * M_PI);
}
template<typename T>
constexpr inline T hopper_belt_mps_to_motor_rps(const T& belt_mps)
{
    return belt_mps *
           ((1. / (HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M * 2 * M_PI)) *
            HOPPER_BELT_GEARING);
}
