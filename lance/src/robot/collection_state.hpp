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

    double calcOffloadTargetMotorPosition(double beg_motor_pos) const;

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
