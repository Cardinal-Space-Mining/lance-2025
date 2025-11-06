#pragma once

// Equivalent to Xbox controller mappings
namespace LogitechController
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
} // namespace LogitechController

namespace Bindings
{
    using namespace LogitechController;

    constexpr int DISABLE_ALL_ACTIONS_BUTTON_IDX = Buttons::A;

    constexpr int TELEOP_LOW_SPEED_BUTTON_IDX = Buttons::B;
    constexpr int TELEOP_MEDIUM_SPEED_BUTTON_IDX = Buttons::Y;
    constexpr int TELEOP_HIGH_SPEED_BUTTON_IDX = Buttons::X;

    constexpr int TELEOP_DRIVE_X_AXIS_IDX = Axes::LEFTX;
    constexpr int TELEOP_DRIVE_Y_AXIS_IDX = Axes::LEFTY;

    constexpr int TELEOP_TRENCHER_SPEED_AXIS_IDX = Axes::R_TRIGGER;
    constexpr int TELEOP_TRENCHER_INVERT_BUTTON_IDX = Buttons::RB;

    constexpr int TELEOP_HOPPER_SPEED_AXIS_IDX = Axes::L_TRIGGER;
    constexpr int TELEOP_HOPPER_INVERT_BUTTON_IDX = Buttons::LB;
    constexpr int TELEOP_HOPPER_ACTUATE_AXIS_IDX = Axes::RIGHTY;

    constexpr int ASSISTED_MINING_TOGGLE_BUTTON_IDX = Buttons::L_STICK;
    constexpr int ASSISTED_OFFLOAD_TOGGLE_BUTTON_IDX = Buttons::R_STICK;

    constexpr int ASSISTED_HOPPER_ENABLE_BUTTON_IDX = Buttons::BACK;
    constexpr int ASSISTED_HOPPER_DISABLE_BUTTON_IDX = Buttons::START;

    constexpr int TELEAUTO_MINING_INIT_POV_ID = Axes::DPAD_U_D;
    constexpr int TELEAUTO_MINING_STOP_POV_ID = Axes::DPAD_U_D;
    constexpr int TELEAUTO_OFFLOAD_INIT_POV_ID = Axes::DPAD_R_L;
    constexpr int TELEAUTO_OFFLOAD_STOP_POV_ID = Axes::DPAD_R_L;

    constexpr float TELEAUTO_MINING_INIT_POV_VAL = Axes::DPAD_K::DPAD_UP;
    constexpr float TELEAUTO_MINING_STOP_POV_VAL = Axes::DPAD_K::DPAD_DOWN;
    constexpr float TELEAUTO_OFFLOAD_INIT_POV_VAL = Axes::DPAD_K::DPAD_RIGHT;
    constexpr float TELEAUTO_OFFLOAD_STOP_POV_VAL = Axes::DPAD_K::DPAD_LEFT;
};
