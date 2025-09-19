#pragma once

#include <math.h>

static constexpr double TRACK_GEARING = 64.;
static constexpr double TRACK_EFFECTIVE_OUTPUT_RADIUS_M = 0.07032851;

static constexpr double TRENCHER_WIDTH_M = 0.254;
static constexpr double TRENCHER_GEARING = 32.;
static constexpr double TRENCHER_LITERS_PER_OUTPUT_ROTATION = (0.04096766 * 6.);

static constexpr double HOPPER_BELT_GEARING = 30.;
static constexpr double HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M = 0.0508;
static constexpr double HOPPER_BELT_LENGTH_M = 0.6;
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
constexpr inline T linear_range_to_mining_depth_m(const T& lr_val)
{
    return MINING_DEPTH_FX_OFFSET + MINING_DEPTH_FX_SLOPE * lr_val;
}
template<typename T>
constexpr inline T mining_depth_to_linear_range(const T& depth_m)
{
    return (depth_m - MINING_DEPTH_FX_OFFSET) * (1. / MINING_DEPTH_FX_SLOPE);
}

template<typename T>
constexpr inline T track_motor_rps_to_volume_rate(
    const T& motor_rps,
    const T& depth_m)
{
    return track_motor_rps_to_ground_mps(motor_rps) * depth_m *
           (TRENCHER_WIDTH_M * 1000.);
}
template<typename T>
constexpr inline T volume_rate_to_track_motor_rps(
    const T& vol_rate_lps,
    const T& depth_m)
{
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
constexpr inline T trecher_motor_rps_to_max_track_motor_rps(
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
