#pragma once

#include <cstdint>
#include <chrono>
#include <string>

#include "sensor_msgs/msg/joy.hpp"

#include "motor_interface.hpp"
#include "logitech_map.hpp"


using JoyMsg = sensor_msgs::msg::Joy;

class RobotControl
{
public:
    inline RobotControl() { this->stop_all(); }
    ~RobotControl() = default;

public:
    /** Iterates control logic based on robot mode, joystick input, and motor statuses */
    RobotMotorCommands& update(
        int32_t robot_status,
        const JoyMsg& joystick_values,
        const RobotMotorStatus& motor_status);

    void getStatusStrings(
        std::string& control_level,
        std::string& mining_status,
        std::string& offload_status);

protected:
    using system_time = std::chrono::system_clock;
    using system_time_point = system_time::time_point;

    enum RobotMode
    {
        DISABLED,
        ENABLED,
        AUTONOMOUS
    };

    class State
    {
    public:
        enum class ControlLevel
        {
            MANUAL = 0,
            ASSISTED_MANUAL = 1,
            TELEAUTO_OP = 2,
            FULL_AUTO = 3
        };
        enum class MiningStage
        {
            INITIALIZING = 0,
            LOWERING_HOPPER = 1,
            TRAVERSING = 2,
            RAISING_HOPPER = 3,
            FINISHED = 4
        };
        enum class OffloadingStage
        {
            INITIALIZING = 0,
            BACKING_UP = 1,
            RAISING_HOPPER = 2,
            OFFLOADING = 3,
            LOWERING_HOPPER = 4,
            FINISHED = 5
        };

    public:
        double driving_speed_scalar = RobotControl::DRIVING_MEDIUM_SPEED_SCALAR;

        ControlLevel control_level = ControlLevel::MANUAL;
        ControlLevel last_manual_control_level = control_level;

        struct  // MINING ROUTINE VARS
        {
            bool enabled = false;
            bool cancelled = false;

            MiningStage stage = MiningStage::FINISHED;

            system_time_point traversal_start_time;

            double target_mining_time = RobotControl::MINING_RUN_TIME_SECONDS;

        } mining;

        struct  // OFFLOAD ROUTINE VARS
        {
            bool enabled = false;
            bool cancelled = false;

            OffloadingStage stage = OffloadingStage::FINISHED;

            system_time_point start_time;
            system_time_point dump_start_time;

            double tele_target_backup_time =
                RobotControl::TELE_OFFLOAD_BACKUP_TIME_SECONDS;
            double auto_target_backup_time =
                RobotControl::AUTO_OFFLOAD_BACKUP_TIME_SECONDS;
            double target_dump_time = RobotControl::OFFLOAD_DUMP_TIME;

        } offload;

    public:
        void reset_auto_states();

        bool mining_is_soft_shutdown();
        bool offload_is_soft_shutdown();

        void handle_change_control_level(
            RobotControl::State::ControlLevel new_level);
    };

protected:
    inline static RobotMode getMode(int32_t status)
    {
        return status > 0
                   ? RobotMode::ENABLED
                   : (status < 0 ? RobotMode::AUTONOMOUS : RobotMode::DISABLED);
    }

    void disable_motors();
    inline void stop_all()
    {
        this->state.reset_auto_states();
        this->disable_motors();
    }

    inline void set_hopper_act_percent(double percent)
    {
        this->motor_commands.hopper_actuator
            .set__mode(TalonCtrl::PERCENT_OUTPUT)
            .set__value(percent);
    }
    inline void set_hopper_belt_velocity(double rps)
    {
        this->motor_commands.hopper_belt.set__mode(TalonCtrl::VELOCITY)
            .set__value(rps);
    }
    inline void set_trencher_velocity(double rps)
    {
        this->motor_commands.trencher.set__mode(TalonCtrl::VELOCITY)
            .set__value(rps);
    }
    inline void set_tracks_velocity(double left_rps, double right_rps)
    {
        this->motor_commands.track_left.set__mode(TalonCtrl::VELOCITY)
            .set__value(left_rps);
        this->motor_commands.track_right.set__mode(TalonCtrl::VELOCITY)
            .set__value(right_rps);
    }

    inline double get_hopper_normalized()
    {
        return this->curr_motor_states.hopper_actuator.position / 1000.;
    }

    inline bool hasButtonIdx(size_t id)
    {
        return this->curr_joystick_values.buttons.size() > id;
    }
    inline bool prevHasButtonIdx(size_t id)
    {
        return this->prev_joystick_values.buttons.size() > id;
    }
    inline bool hasAxisIdx(size_t id)
    {
        return this->curr_joystick_values.axes.size() > id;
    }

    inline bool getRawButton(int id)
    {
        return this->hasButtonIdx(id) && this->curr_joystick_values.buttons[id];
    }
    inline bool getButtonPressed(int id)
    {
        return this->prevHasButtonIdx(id) &&
               !this->prev_joystick_values.buttons[id] &&
               this->curr_joystick_values.buttons[id];
    }
    inline float getRawAxis(int id)
    {
        return this->hasAxisIdx(id) ? this->curr_joystick_values.axes[id] : 0.f;
    }
    inline bool getPov(int id, float val)
    {
        return this->hasAxisIdx(id) &&
               this->curr_joystick_values.axes[id] == val;
    }

protected:
    void start_mining(RobotControl::State::ControlLevel op_level);
    void cancel_mining();
    void start_offload(RobotControl::State::ControlLevel op_level);
    void cancel_offload();

protected:
    void periodic_handle_mining();
    void periodic_handle_offload();
    void periodic_handle_teleop_input();

private:
    JoyMsg prev_joystick_values;
    JoyMsg curr_joystick_values;
    RobotMode prev_robot_mode{RobotMode::DISABLED};
    RobotMode curr_robot_mode{RobotMode::DISABLED};
    RobotMotorStatus curr_motor_states;
    RobotMotorCommands motor_commands;
    State state;

public:
    // clang-format off
    static constexpr double
    // motor physical speed targets
        TRENCHER_MAX_VELO = 80,                         // maximum mining speed -- TURNS PER SECOND
        TRENCHER_NOMINAL_MINING_VELO = 80,              // base trenching speed -- TURNS PER SECOND
        HOPPER_BELT_MAX_VELO = 45,                      // TURNS PER SECOND
        HOPPER_BELT_MAX_MINING_VELO = 10,               // TURNS PER SECOND
        TRACKS_MAX_VELO = 125,                          // TURNS PER SECOND
        TRACKS_MINING_VELO = 8,                         // TURNS PER SECOND
        TRACKS_MAX_ADDITIONAL_MINING_VEL = 6,           // TURNS PER SECOND
        TRACKS_OFFLOAD_VELO = TRACKS_MAX_VELO * 0.25;   // TURNS PER SECOND

    static constexpr double
    // driving
        DRIVING_MAGNITUDE_DEADZONE_SCALAR = 0.1,
        DRIVING_LOW_SPEED_SCALAR = 0.3,
        DRIVING_MEDIUM_SPEED_SCALAR = 0.7,
        DRIVING_HIGH_SPEED_SCALAR = 1.0,
        GENERIC_DEADZONE_SCALAR = 0.05,
    // hopper
        HOPPER_ACTUATOR_PLUNGE_SPEED = 0.40,
        HOPPER_ACTUATOR_EXTRACT_SPEED = 0.80,
        HOPPER_ACUTATOR_MOVE_SPEED = 1.0,       // all other movement (ie. dumping)
    // actuator potentiometer target values
        OFFLOAD_POT_VALUE = 0.95,               // dump height
        TRAVERSAL_POT_VALUE = 0.60,             // traversal height
        AUTO_TRANSPORT_POT_VALUE = 0.55,        // height for transporting regolith
        MINING_DEPTH_NOMINAL_POT_VALUE = 0.21,  // nominal mining depth from which manual adjustments can be made
        MINING_DEPTH_LIMIT_POT_VALUE = 0.03,    // lowest depth we ever want to go
        HOPPER_POT_TARGETING_EPSILON = 0.01,
    // timed operations
        MINING_RUN_TIME_SECONDS = 1.0,              // teleauto mining run time
        TELE_OFFLOAD_BACKUP_TIME_SECONDS = 3.0,     // teleauto offload duration
        AUTO_OFFLOAD_BACKUP_TIME_SECONDS = 2.0,
        OFFLOAD_DUMP_TIME = 6.0,
    // auto belt duty cycle
        HOPPER_BELT_TIME_ON_SECONDS = 1.0,
        HOPPER_BELT_TIME_OFF_SECONDS = 2.5;

    static constexpr int
        DISABLE_ALL_ACTIONS_BUTTON_IDX = LogitechMapping::Buttons::A,

        TELEOP_LOW_SPEED_BUTTON_IDX = LogitechMapping::Buttons::B,
        TELEOP_MEDIUM_SPEED_BUTTON_IDX = LogitechMapping::Buttons::Y,
        TELEOP_HIGH_SPEED_BUTTON_IDX = LogitechMapping::Buttons::X,

        TELEOP_DRIVE_X_AXIS_IDX = LogitechMapping::Axes::LEFTX,
        TELEOP_DRIVE_Y_AXIS_IDX = LogitechMapping::Axes::LEFTY,

        TELEOP_TRENCHER_SPEED_AXIS_IDX = LogitechMapping::Axes::R_TRIGGER,
        TELEOP_TRENCHER_INVERT_BUTTON_IDX = LogitechMapping::Buttons::RB,

        TELEOP_HOPPER_SPEED_AXIS_IDX = LogitechMapping::Axes::L_TRIGGER,
        TELEOP_HOPPER_INVERT_BUTTON_IDX = LogitechMapping::Buttons::LB,
        TELEOP_HOPPER_ACTUATE_AXIS_IDX = LogitechMapping::Axes::RIGHTY,
        
        ASSISTED_MINING_TOGGLE_BUTTON_IDX = LogitechMapping::Buttons::L_STICK,
        ASSISTED_OFFLOAD_TOGGLE_BUTTON_IDX = LogitechMapping::Buttons::R_STICK,

        TELEAUTO_MINING_INIT_POV_ID = LogitechMapping::Axes::DPAD_U_D,
        TELEAUTO_MINING_STOP_POV_ID = LogitechMapping::Axes::DPAD_U_D,
        TELEAUTO_OFFLOAD_INIT_POV_ID = LogitechMapping::Axes::DPAD_R_L,
        TELEAUTO_OFFLOAD_STOP_POV_ID = LogitechMapping::Axes::DPAD_R_L;

    static constexpr float
        TELEAUTO_MINING_INIT_POV_VAL = LogitechMapping::Axes::DPAD_K::DPAD_UP,
        TELEAUTO_MINING_STOP_POV_VAL = LogitechMapping::Axes::DPAD_K::DPAD_DOWN,
        TELEAUTO_OFFLOAD_INIT_POV_VAL = LogitechMapping::Axes::DPAD_K::DPAD_RIGHT,
        TELEAUTO_OFFLOAD_STOP_POV_VAL = LogitechMapping::Axes::DPAD_K::DPAD_LEFT;
    // clang-format on
};
