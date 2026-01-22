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

#include "../util/joy_utils.hpp"
#include "hid_constants.hpp"


namespace Bindings
{
using namespace util;
using namespace LogitechController;

using DisableAllActionsButton = StaticJoyButton<Buttons::A>;

using TeleopLowSpeedButton = StaticJoyButton<Buttons::B>;
using TeleopMediumSpeedButton = StaticJoyButton<Buttons::Y>;
using TeleopHighSpeedButton = StaticJoyButton<Buttons::X>;

using TeleopDriveXAxis = StaticJoyAxis<Axes::LEFTX>;
using TeleopDriveYAxis = StaticJoyAxis<Axes::LEFTY>;

using TeleopTrencherSpeedAxis = StaticJoyAxis<Axes::R_TRIGGER>;
using TeleopTrencherInvertButton = StaticJoyButton<Buttons::RB>;

using TeleopHopperSpeedAxis = StaticJoyAxis<Axes::L_TRIGGER>;
using TeleopHopperInvertButton = StaticJoyButton<Buttons::LB>;
using TeleopHopperActuateAxis = StaticJoyAxis<Axes::RIGHTY>;

using AssistedMiningToggleButton = StaticJoyButton<Buttons::L_STICK>;
using AssistedOffloadToggleButton = StaticJoyButton<Buttons::R_STICK>;

using PresetMiningInitButton = StaticJoyButton<Buttons::BACK>;
using PresetOffloadInitButton = StaticJoyButton<Buttons::START>;

using ToggleTraversalCursorMode = StaticJoyButton<Buttons::LOGITECH>;

// using PresetMiningStartButton =
//     StaticJoyPov<Axes::DPAD_U_D, Axes::DPAD_K::DPAD_UP>;
// using PresetMiningStopButton =
//     StaticJoyPov<Axes::DPAD_U_D, Axes::DPAD_K::DPAD_DOWN>;
// using PresetOffloadStartButton =
//     StaticJoyPov<Axes::DPAD_R_L, Axes::DPAD_K::DPAD_RIGHT>;
// using PresetOffloadStopButton =
//     StaticJoyPov<Axes::DPAD_R_L, Axes::DPAD_K::DPAD_LEFT>;
};  // namespace Bindings
