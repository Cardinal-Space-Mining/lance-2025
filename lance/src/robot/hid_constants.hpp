#pragma once

namespace LogitechMapping
{
    namespace Buttons
    {
        constexpr int A = 0;
        constexpr int B = 1;
        constexpr int X = 2;
        constexpr int Y = 3;
        constexpr int LB = 4;
        constexpr int RB = 5;
        constexpr int BACK = 6;
        constexpr int START = 7;
        constexpr int LOGITECH = 8;
        constexpr int L_STICK = 9;
        constexpr int R_STICK = 10;

        constexpr int NUM_BUTTONS = 11;
    } // namespace Buttons

    namespace Axes
    {
        constexpr int LEFTX = 0;
        constexpr int LEFTY = 1;
        constexpr int L_TRIGGER = 2;

        constexpr int RIGHTX = 3;
        constexpr int RIGHTY = 4;
        constexpr int R_TRIGGER = 5;

        constexpr int DPAD_R_L = 6;
        constexpr int DPAD_U_D = 7;

        namespace DPAD_K
        {
            constexpr float DPAD_DOWN = -1.f;
            constexpr float DPAD_UP = 1.f;

            constexpr float DPAD_RIGHT = -1.f;
            constexpr float DPAD_LEFT = 1.f;
        } // namespace DPAD_K

        constexpr int NUM_AXES = 8;

    } // namespace Axes
} // namespace LogitechMapping
