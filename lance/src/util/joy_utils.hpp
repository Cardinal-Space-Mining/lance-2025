/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
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

#include <cmath>

#include <sensor_msgs/msg/joy.hpp>

#include "time_cvt.hpp"


namespace util
{

struct JoyState
{
    using JoyMsg = sensor_msgs::msg::Joy;

public:
    JoyMsg::_axes_type prev_axes, axes;
    JoyMsg::_buttons_type prev_buttons, buttons;
    double dt{0.};
    double stamp{0.};
    bool continuous{true};

public:
    inline void update(const JoyMsg& joy)
    {
        this->continuous =
            (joy.axes.size() == this->axes.size() &&
             joy.buttons.size() == this->buttons.size());

        this->prev_axes.swap(this->axes);
        this->prev_buttons.swap(this->buttons);
        this->axes = joy.axes;
        this->buttons = joy.buttons;

        const double t = util::toFloatSeconds(joy.header.stamp);
        if (this->stamp > 0.)
        {
            this->dt = t - stamp;
        }
        this->stamp = t;
    }

public:
    inline bool hasButtonIdx(int idx) const
    {
        return this->buttons.size() > static_cast<size_t>(idx);
    }
    inline bool hasAxisIdx(int idx) const
    {
        return this->axes.size() > static_cast<size_t>(idx);
    }
    inline bool isButtonContinuous(int idx) const
    {
        return this->continuous && this->hasButtonIdx(idx);
    }
    inline bool isAxisContinuous(int idx) const
    {
        return this->continuous && this->hasAxisIdx(idx);
    }

public:
    inline bool getRawButton(int idx) const
    {
        return this->hasButtonIdx(idx) && this->buttons[idx];
    }
    inline bool getButtonPressed(int idx) const
    {
        return this->isButtonContinuous(idx) && !this->prev_buttons[idx] &&
               this->buttons[idx];
    }
    inline bool getButtonReleased(int idx) const
    {
        return this->isButtonContinuous(idx) && this->prev_buttons[idx] &&
               !this->buttons[idx];
    }

    inline float getRawAxis(int idx) const
    {
        return this->hasAxisIdx(idx) ? this->axes[idx] : 0.f;
    }
    inline float getAxisDelta(int idx) const
    {
        return this->isAxisContinuous(idx)
                   ? (this->axes[idx] - this->prev_axes[idx])
                   : 0.f;
    }
    inline float getAxisVelocity(int idx) const
    {
        return this->isAxisContinuous(idx)
                   ? (this->axes[idx] - this->prev_axes[idx]) / this->dt
                   : 0.f;
    }

    inline bool getRawPov(int idx, int sgn) const
    {
        return this->hasAxisIdx(idx) &&
               (static_cast<int>(this->axes[idx]) * sgn) > 0;
    }
    inline bool getPovPressed(int idx, int sgn) const
    {
        return this->isAxisContinuous(idx) &&
               (static_cast<int>(this->prev_axes[idx]) * sgn) <= 0 &&
               (static_cast<int>(this->axes[idx]) * sgn) > 0;
    }
    inline bool getPovReleased(int idx, int sgn) const
    {
        return this->isAxisContinuous(idx) &&
               (static_cast<int>(this->prev_axes[idx]) * sgn) > 0 &&
               (static_cast<int>(this->axes[idx]) * sgn) <= 0;
    }
};


// --- Non-constexpr binding wrappers ------------------------------------------
struct JoyButton
{
    int idx{0};

    inline bool rawValue(const JoyState& joy) const
    {
        return joy.getRawButton(this->idx);
    }
    inline bool wasPressed(const JoyState& joy) const
    {
        return joy.getButtonPressed(this->idx);
    }
    inline bool wasReleased(const JoyState& joy) const
    {
        return joy.getButtonReleased(this->idx);
    }
};
struct JoyAxis
{
    int idx{0};

    inline float rawValue(const JoyState& joy) const
    {
        return joy.getRawAxis(this->idx);
    }
    inline float deadzoneValue(const JoyState& joy, float deadzone) const
    {
        float v = this->rawValue(joy);
        return std::abs(v) >= deadzone ? v : 0.f;
    }
    inline float triggerValue(const JoyState& joy) const
    {
        return (1.f - this->rawValue(joy)) / 2.f;
    }
    inline float delta(const JoyState& joy) const
    {
        return joy.getAxisDelta(this->idx);
    }
    inline float velocity(const JoyState& joy) const
    {
        return joy.getAxisVelocity(this->idx);
    }
};
struct JoyPov
{
    int idx{0};
    int sgn{0};

    inline bool rawValue(const JoyState& joy) const
    {
        return joy.getRawPov(this->idx, this->sgn);
    }
    inline bool wasPressed(const JoyState& joy) const
    {
        return joy.getPovPressed(this->idx, this->sgn);
    }
    inline bool wasReleased(const JoyState& joy) const
    {
        return joy.getPovReleased(this->idx, this->sgn);
    }
};

// --- Constexpr binding wrappers ----------------------------------------------
template<int Idx>
struct StaticJoyButton
{
    static inline bool rawValue(const JoyState& joy)
    {
        return joy.getRawButton(Idx);
    }
    static inline bool wasPressed(const JoyState& joy)
    {
        return joy.getButtonPressed(Idx);
    }
    static inline bool wasReleased(const JoyState& joy)
    {
        return joy.getButtonReleased(Idx);
    }
};
template<int Idx>
struct StaticJoyAxis
{
    static inline float rawValue(const JoyState& joy)
    {
        return joy.getRawAxis(Idx);
    }
    static inline float deadzoneValue(const JoyState& joy, float deadzone)
    {
        float v = rawValue(joy);
        return std::abs(v) >= deadzone ? v : 0.f;
    }
    static inline float triggerValue(const JoyState& joy)
    {
        return (1.f - rawValue(joy)) / 2.f;
    }
    static inline float delta(const JoyState& joy)
    {
        return joy.getAxisDelta(Idx);
    }
    static inline float velocity(const JoyState& joy)
    {
        return joy.getAxisVelocity(Idx);
    }
};
template<int Idx, int Sgn>
struct StaticJoyPov
{
    static inline bool rawValue(const JoyState& joy)
    {
        return joy.getRawPov(Idx, Sgn);
    }
    static inline bool wasPressed(const JoyState& joy)
    {
        return joy.getPovPressed(Idx, Sgn);
    }
    static inline bool wasReleased(const JoyState& joy)
    {
        return joy.getPovReleased(Idx, Sgn);
    }
};

};  // namespace util
