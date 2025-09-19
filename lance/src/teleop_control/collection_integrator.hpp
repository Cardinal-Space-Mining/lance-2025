#pragma once

#include <limits>

#include "calculations.hpp"


class CollectionIntegrator
{
public:
    void update(
        double trencher_rotations,
        double normalized_actuator_pos,
        double hopper_belt_rotations)
    {
        // convert act pos to mining depth
        // avg mining depth, delta rotations --> delta volume input
        // 
    }

protected:
    double prev_trencher_rotations = std::numeric_limits<double>::infinity();
    double prev_belt_rotations = std::numeric_limits<double>::infinity();
    double prev_mining_depth = -1.;

    double estimated_volume_liters = 0.;

};
