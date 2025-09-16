#include "controller.hpp"

#include <array>
#include <cmath>
#include <ctime>
#include <chrono>
#include <numeric>


using system_time = std::chrono::system_clock;
using system_time_point = system_time::time_point;

static constexpr long double PI = 3.14159265358979323846;

namespace util
{

inline double seconds_since(const system_time_point& tp)
{
    return std::chrono::duration<double>{system_time::now() - tp}.count();
}

inline double apply_deadband(double val, double deadband)
{
    return std::abs(val) < deadband ? 0. : val;
}

inline float normalizeJoyTriggerAxis(float in) { return (1.f - in) / 2.f; }

inline void disableMotor(TalonCtrl& info)
{
    info.set__mode(TalonCtrl::DISABLED);
    info.set__value(0.);
}

std::array<double, 2>
    computeWheelScalars(double x, double y, double mag_deadzone)
{
    const double augmented_angle =
        std::atan2(x, y) +
        (PI / 4.0);  // x and y are inverted to make a CW "heading" angle
    double magnitude = std::sqrt(x * x + y * y);
    if (magnitude < mag_deadzone)
    {
        return {0.0, 0.0};
    }

    return {
        magnitude * std::sin(augmented_angle),
        magnitude *
            std::cos(
                augmented_angle)  // this is the same as cos("raw theta" - pi/4) like from the original code
    };
}
}  // namespace util



RobotMotorCommands& RobotControl::update(
    int32_t robot_status,
    const JoyMsg& joystick_values,
    const RobotMotorStatus& motor_status)
{
    this->prev_robot_mode = this->curr_robot_mode;
    this->curr_robot_mode = getMode(robot_status);

    this->prev_joystick_values = this->curr_joystick_values;
    this->curr_joystick_values = joystick_values;

    this->curr_motor_states = motor_status;
    this->disable_motors();  // This resets all the command states to be disabled! (gets overwritten by any other actions!)

    // handle robot mode transitions
    {
        switch (this->curr_robot_mode)
        {
            case RobotMode::DISABLED:
            {
                switch (this->prev_robot_mode)
                {
                    case RobotMode::ENABLED:
                    {
                        this->state.last_manual_control_level =
                            this->state.control_level;
                        // fallthrough on purpose
                    }
                    case RobotMode::AUTONOMOUS:
                    {
                        this->stop_all();
                        break;
                    }
                    default:
                    {
                    }
                }

                break;
            }
            case RobotMode::ENABLED:
            {
                switch (this->prev_robot_mode)
                {
                    case RobotMode::DISABLED:
                    case RobotMode::AUTONOMOUS:
                    {
                        this->stop_all();
                        this->state.control_level =
                            this->state.last_manual_control_level;
                        break;
                    }
                    default:
                    {
                    }
                    break;
                }

                this->periodic_handle_teleop_input();
                break;
            }
            case RobotMode::AUTONOMOUS:
            {
                switch (this->prev_robot_mode)
                {
                    case RobotMode::ENABLED:
                    {
                        this->state.last_manual_control_level =
                            this->state.control_level;
                        // fallthrough on purpose
                    }
                    case RobotMode::DISABLED:
                    {
                        this->stop_all();
                        this->state.control_level =
                            RobotControl::State::ControlLevel::FULL_AUTO;
                        break;
                    }
                    default:
                    {
                    }
                }

                break;
            }
        }

        this->periodic_handle_mining();
        this->periodic_handle_offload();
    }

    return this->motor_commands;
}

void RobotControl::getStatusStrings(
    std::string& control_level,
    std::string& mining_status,
    std::string& offload_status)
{
    static constexpr char const* CONTROL_LEVEL_NAMES[]{
        "Manual",
        "Assisted Manual",
        "Teleauto Operation",
        "Full Auto"};
    static constexpr char const* MINING_STAGE_NAMES[]{
        "Initializing",
        "Lowering Hopper",
        "Traversing",
        "Raising Hopper",
        "Finished"};
    static constexpr char const* OFFLOAD_STAGE_NAMES[]{
        "Initializing",
        "Backing Up",
        "Raising Hopper",
        "Offloading",
        "Lowering Hopper",
        "Finished"};

    control_level.assign(
        CONTROL_LEVEL_NAMES[static_cast<size_t>(this->state.control_level)]);
    mining_status.assign(
        (this->state.mining.cancelled &&
         this->state.mining.stage == RobotControl::State::MiningStage::FINISHED)
            ? "Cancelled"
        : this->state.mining.enabled
            ? MINING_STAGE_NAMES[static_cast<size_t>(this->state.mining.stage)]
            : "Disabled");
    offload_status.assign(
        (this->state.offload.cancelled &&
         this->state.offload.stage ==
             RobotControl::State::OffloadingStage::FINISHED)
            ? "Cancelled"
        : this->state.offload.enabled ? OFFLOAD_STAGE_NAMES[static_cast<size_t>(
                                            this->state.offload.stage)]
                                      : "Disabled");
}


void RobotControl::State::reset_auto_states()
{
    this->mining.enabled = false;
    this->offload.enabled = false;
    this->mining.stage = RobotControl::State::MiningStage::FINISHED;
    this->offload.stage = RobotControl::State::OffloadingStage::FINISHED;
}

bool RobotControl::State::mining_is_soft_shutdown()
{
    switch (this->control_level)
    {
        case RobotControl::State::ControlLevel::ASSISTED_MANUAL:
        case RobotControl::State::ControlLevel::FULL_AUTO:
        {
            return this->mining.cancelled;
        }
        case RobotControl::State::ControlLevel::MANUAL:
        case RobotControl::State::ControlLevel::TELEAUTO_OP:
        default:
        {
            return false;
        }
    }
}

bool RobotControl::State::offload_is_soft_shutdown()
{
    switch (this->control_level)
    {
        case RobotControl::State::ControlLevel::ASSISTED_MANUAL:
        case RobotControl::State::ControlLevel::FULL_AUTO:
        {
            return this->offload.cancelled;
        }
        case RobotControl::State::ControlLevel::MANUAL:
        case RobotControl::State::ControlLevel::TELEAUTO_OP:
        default:
        {
            return false;
        }
    }
}

void RobotControl::State::handle_change_control_level(
    RobotControl::State::ControlLevel new_level)
{
    switch (this->control_level)  // make this more sophisticated in the future
    {
        case RobotControl::State::ControlLevel::MANUAL:
        case RobotControl::State::ControlLevel::ASSISTED_MANUAL:
        {
            this->last_manual_control_level = this->control_level;
            this->control_level = new_level;
            break;
        }
        case RobotControl::State::ControlLevel::TELEAUTO_OP:
        case RobotControl::State::ControlLevel::FULL_AUTO:
        {
            this->control_level = new_level;
            break;
        }
        default:
        {
        }
    }
}


void RobotControl::disable_motors()
{
    util::disableMotor(this->motor_commands.track_left);
    util::disableMotor(this->motor_commands.track_right);
    util::disableMotor(this->motor_commands.trencher);
    util::disableMotor(this->motor_commands.hopper_belt);
    util::disableMotor(this->motor_commands.hopper_actuator);
}



// ------------- Mining/Offload init and shutdown --------------

void RobotControl::start_mining(RobotControl::State::ControlLevel op_level)
{
    this->state.handle_change_control_level(op_level);
    if (!this->state.mining.enabled && !this->state.offload.enabled &&
        this->state.control_level != RobotControl::State::ControlLevel::MANUAL)
    {
        this->stop_all();

        this->state.mining.enabled = true;
        this->state.mining.cancelled = false;
        this->state.mining.stage =
            RobotControl::State::MiningStage::INITIALIZING;
    }
}

void RobotControl::cancel_mining()
{
    if (this->state.mining.enabled && !this->state.offload.enabled)
    {
        switch (this->state.control_level)
        {
            case RobotControl::State::ControlLevel::ASSISTED_MANUAL:
            case RobotControl::State::ControlLevel::FULL_AUTO:
            {
                this->state.mining.cancelled = true;  // trigger automated exit
                break;
            }
            case RobotControl::State::ControlLevel::MANUAL:
            case RobotControl::State::ControlLevel::TELEAUTO_OP:
            default:
            {
                this->state.handle_change_control_level(
                    RobotControl::State::ControlLevel::MANUAL);
                this->stop_all();  // hard stop... and reset all states

                // this->state.mining.enabled = false;
                // this->state.mining.stage =
                //     RobotControl::State::MiningStage::FINISHED;
            }
        }
    }
}

void RobotControl::start_offload(RobotControl::State::ControlLevel op_level)
{
    this->state.handle_change_control_level(op_level);
    if (!this->state.mining.enabled && !this->state.offload.enabled &&
        this->state.control_level != RobotControl::State::ControlLevel::MANUAL)
    {
        this->stop_all();

        this->state.offload.enabled = true;
        this->state.offload.cancelled = false;
        this->state.offload.stage =
            RobotControl::State::OffloadingStage::INITIALIZING;
    }
}

void RobotControl::cancel_offload()
{
    if (this->state.offload.enabled && !this->state.mining.enabled)
    {
        switch (this->state.control_level)
        {
            case RobotControl::State::ControlLevel::ASSISTED_MANUAL:
            case RobotControl::State::ControlLevel::FULL_AUTO:
            {
                this->state.offload.cancelled = true;  // trigger automated exit
                break;
            }
            case RobotControl::State::ControlLevel::MANUAL:
            case RobotControl::State::ControlLevel::TELEAUTO_OP:
            default:
            {
                this->state.handle_change_control_level(
                    RobotControl::State::ControlLevel::MANUAL);
                this->stop_all();  // hard stop... and reset all states

                // this->state.offload.enabled = false;
                // this->state.offload.stage =
                //     RobotControl::State::OffloadingStage::FINISHED;
            }
        }
    }
}





// ----------------- Periodic Handlers ----------------------

void RobotControl::periodic_handle_mining()
{
    if (this->state.mining.enabled)
    {
        const bool cancelled = this->state.mining_is_soft_shutdown();

        switch (this->state.mining.stage)
        {
            case RobotControl::State::MiningStage::INITIALIZING:
            {
                this->state.mining.stage =
                    RobotControl::State::MiningStage::LOWERING_HOPPER;
                // fallthrough to process the next stage
            }
            case RobotControl::State::MiningStage::LOWERING_HOPPER:
            {
                const double pot_val = this->get_hopper_pot();
                if (!cancelled &&
                    pot_val > RobotControl::MINING_DEPTH_NOMINAL_POT_VALUE)
                {
                    // set trencher
                    this->motor_commands.trencher.set__mode(TalonCtrl::VELOCITY)
                        .set__value(RobotControl::TRENCHER_NOMINAL_MINING_VELO);

                    // set actuator
                    if (pot_val > RobotControl::TRAVERSAL_POT_VALUE)
                    {
                        this->motor_commands.hopper_actuator
                            .set__mode(TalonCtrl::PERCENT_OUTPUT)
                            .set__value(
                                -RobotControl::HOPPER_ACUTATOR_MOVE_SPEED);
                    }
                    else
                    {
                        this->motor_commands.hopper_actuator
                            .set__mode(TalonCtrl::PERCENT_OUTPUT)
                            .set__value(
                                -RobotControl::HOPPER_ACTUATOR_PLUNGE_SPEED);
                    }
                    break;
                }
                else
                {
                    util::disableMotor(this->motor_commands.hopper_actuator);
                    this->state.mining.traversal_start_time =
                        system_time::now();
                    this->state.mining.stage =
                        RobotControl::State::MiningStage::TRAVERSING;
                    // allow fallthrough bc we might as well start processing traversal
                }
            }
            case RobotControl::State::MiningStage::TRAVERSING:
            {
                if (!cancelled &&
                    (this->state.control_level ==
                         RobotControl::State::ControlLevel::ASSISTED_MANUAL ||
                     util::seconds_since(
                         this->state.mining.traversal_start_time) <
                         this->state.mining.target_mining_time))
                {
                    double trencher_setpt =
                        RobotControl::TRENCHER_NOMINAL_MINING_VELO;
                    double tracks_setpt = RobotControl::TRACKS_MINING_VELO;
                    // handle manual adjustments
                    if (this->state.control_level ==
                        RobotControl::State::ControlLevel::ASSISTED_MANUAL)
                    {
                        // adjust trencher speed (slows down)
                        {
                            const double adjustment_raw = util::apply_deadband(
                                util::normalizeJoyTriggerAxis(this->getRawAxis(
                                    RobotControl::
                                        TELEOP_TRENCHER_SPEED_AXIS_IDX)),
                                RobotControl::GENERIC_DEADZONE_SCALAR);

                            trencher_setpt =
                                RobotControl::TRENCHER_NOMINAL_MINING_VELO *
                                (1.0 - adjustment_raw);

                            // set trencher
                            this->motor_commands.trencher
                                .set__mode(TalonCtrl::VELOCITY)
                                .set__value(trencher_setpt);
                        }
                        // adjust trencher depth with right stick -- WARNING: DEPENDS ON POTENTIOMENTER
                        {
                            double target_depth =
                                RobotControl::MINING_DEPTH_NOMINAL_POT_VALUE;
                            const double adjustment_raw = util::apply_deadband(
                                this->getRawAxis(
                                    RobotControl::
                                        TELEOP_HOPPER_ACTUATE_AXIS_IDX),
                                RobotControl::GENERIC_DEADZONE_SCALAR);

                            if (adjustment_raw > 0.0)
                            {
                                target_depth +=
                                    (RobotControl::AUTO_TRANSPORT_POT_VALUE -
                                     RobotControl::
                                         MINING_DEPTH_NOMINAL_POT_VALUE) *
                                    adjustment_raw;
                            }
                            if (adjustment_raw < 0.0)
                            {
                                target_depth +=
                                    (RobotControl::
                                         MINING_DEPTH_NOMINAL_POT_VALUE -
                                     RobotControl::
                                         MINING_DEPTH_LIMIT_POT_VALUE) *
                                    adjustment_raw;
                            }

                            // servo based on target
                            const double pot_val = this->get_hopper_pot();
                            if (std::abs(target_depth - pot_val) <
                                RobotControl::
                                    HOPPER_POT_TARGETING_EPSILON)  // in range
                            {
                                util::disableMotor(
                                    this->motor_commands.hopper_actuator);
                            }
                            else if (pot_val < target_depth)
                            {
                                this->motor_commands.hopper_actuator
                                    .set__mode(TalonCtrl::PERCENT_OUTPUT)
                                    .set__value(
                                        RobotControl::
                                            HOPPER_ACTUATOR_PLUNGE_SPEED);
                            }
                            else if (pot_val > target_depth)
                            {
                                this->motor_commands.hopper_actuator
                                    .set__mode(TalonCtrl::PERCENT_OUTPUT)
                                    .set__value(
                                        -RobotControl::
                                            HOPPER_ACTUATOR_PLUNGE_SPEED);
                            }
                        }
                        // adjust tracks speed
                        {
                            const double adjustment_raw = util::apply_deadband(
                                this->getRawAxis(
                                    RobotControl::TELEOP_DRIVE_Y_AXIS_IDX),
                                RobotControl::
                                    DRIVING_MAGNITUDE_DEADZONE_SCALAR);

                            if (adjustment_raw > 0.0)
                            {
                                tracks_setpt +=
                                    (RobotControl::
                                         TRACKS_MAX_ADDITIONAL_MINING_VEL *
                                     adjustment_raw);
                            }
                            if (adjustment_raw < 0.0)
                            {
                                tracks_setpt +=
                                    (RobotControl::TRACKS_MINING_VELO *
                                     adjustment_raw);
                            }
                        }
                    }

                    // handle hopper duty cycle -- scaled by trencher relative velocity --> did not work :(
                    // const double trencher_percent = trencher_setpt / RobotControl::TRENCHER_NOMINAL_MINING_VELO;
                    // if( trencher_percent != 0.0 && RobotControl::HOPPER_BELT_TIME_ON_SECONDS > std::abs(
                    //      std::fmod( util::seconds_since(state.offload.start_time), (RobotControl::HOPPER_BELT_TIME_ON_SECONDS / trencher_percent) + RobotControl::HOPPER_BELT_TIME_OFF_SECONDS)) )
                    if (RobotControl::HOPPER_BELT_TIME_ON_SECONDS *
                            (trencher_setpt /
                             RobotControl::TRENCHER_NOMINAL_MINING_VELO) *
                            (tracks_setpt / RobotControl::TRACKS_MINING_VELO) >
                        std::abs(
                            std::fmod(
                                util::seconds_since(state.offload.start_time),
                                RobotControl::HOPPER_BELT_TIME_ON_SECONDS +
                                    RobotControl::
                                        HOPPER_BELT_TIME_OFF_SECONDS)))
                    {
                        this->motor_commands.hopper_belt
                            .set__mode(TalonCtrl::VELOCITY)
                            .set__value(
                                RobotControl::HOPPER_BELT_MAX_MINING_VELO);
                    }
                    else
                    {
                        util::disableMotor(this->motor_commands.hopper_belt);
                    }

                    // set tracks
                    this->motor_commands.track_left
                        .set__mode(TalonCtrl::VELOCITY)
                        .set__value(tracks_setpt);
                    this->motor_commands.track_right
                        .set__mode(TalonCtrl::VELOCITY)
                        .set__value(tracks_setpt);

                    break;
                }
                else
                {
                    util::disableMotor(this->motor_commands.track_left);
                    util::disableMotor(this->motor_commands.track_right);
                    util::disableMotor(this->motor_commands.hopper_belt);
                    this->state.mining.stage =
                        RobotControl::State::MiningStage::RAISING_HOPPER;
                    // allow fallthrough
                }
            }
            case RobotControl::State::MiningStage::RAISING_HOPPER:
            {
                const double pot_val = this->get_hopper_pot();
                if (pot_val < RobotControl::AUTO_TRANSPORT_POT_VALUE)
                {
                    // set trencher
                    this->motor_commands.trencher.set__mode(TalonCtrl::VELOCITY)
                        .set__value(RobotControl::TRENCHER_NOMINAL_MINING_VELO);
                    // set actuator
                    this->motor_commands.hopper_actuator
                        .set__mode(TalonCtrl::PERCENT_OUTPUT)
                        .set__value(
                            RobotControl::HOPPER_ACTUATOR_EXTRACT_SPEED);
                    break;
                }
                else
                {
                    util::disableMotor(this->motor_commands.hopper_actuator);
                    this->state.mining.stage =
                        RobotControl::State::MiningStage::FINISHED;
                    // fallthrough to call shutdown
                }
            }
            case RobotControl::State::MiningStage::FINISHED:
            {
                this->stop_all();
                this->state.mining.enabled = false;
                this->state.handle_change_control_level(
                    RobotControl::State::ControlLevel::
                        MANUAL);  // TODO: needs to change for full auto?
            }
            default:
            {
                // nothing
            }
        }
    }
}

void RobotControl::periodic_handle_offload()
{
    if (this->state.offload.enabled)
    {
        const bool is_full_auto = this->state.control_level ==
                                  RobotControl::State::ControlLevel::FULL_AUTO,
                   is_assisted =
                       this->state.control_level ==
                       RobotControl::State::ControlLevel::ASSISTED_MANUAL,
                   cancelled = this->state.offload_is_soft_shutdown();

        // control the tracks manually if in auto assist
        if (is_assisted)
        {
            // control tracks
            const double vel_cmd =
                (RobotControl::TRACKS_MAX_VELO *
                 RobotControl::DRIVING_LOW_SPEED_SCALAR *
                 this->state.driving_speed_scalar) *
                util::apply_deadband(
                    this->getRawAxis(RobotControl::TELEOP_DRIVE_Y_AXIS_IDX),
                    RobotControl::DRIVING_MAGNITUDE_DEADZONE_SCALAR);

            this->motor_commands.track_right.set__mode(TalonCtrl::VELOCITY)
                .set__value(vel_cmd);
            this->motor_commands.track_left.set__mode(TalonCtrl::VELOCITY)
                .set__value(vel_cmd);
        }

        switch (this->state.offload.stage)
        {
            case RobotControl::State::OffloadingStage::INITIALIZING:
            {
                this->state.offload.start_time = system_time::now();
                this->state.offload.stage =
                    RobotControl::State::OffloadingStage::BACKING_UP;
                // fallthrough
            }
            case RobotControl::State::OffloadingStage::BACKING_UP:
            {
                if (!is_assisted)
                {
                    const double duration =
                        util::seconds_since(this->state.offload.start_time);
                    if (!cancelled &&
                        ((is_full_auto &&
                          duration <
                              this->state.offload.auto_target_backup_time) ||
                         (!is_full_auto &&
                          duration <
                              this->state.offload
                                  .tele_target_backup_time)))  // use serial_control state for this if deemed reliable enough
                    {
                        // drive backwards
                        this->motor_commands.track_right
                            .set__mode(TalonCtrl::VELOCITY)
                            .set__value(-RobotControl::TRACKS_OFFLOAD_VELO);
                        this->motor_commands.track_left
                            .set__mode(TalonCtrl::VELOCITY)
                            .set__value(-RobotControl::TRACKS_OFFLOAD_VELO);

                        break;
                    }
                    else
                    {
                        util::disableMotor(this->motor_commands.track_left);
                        util::disableMotor(this->motor_commands.track_right);
                        // fallthrough to apply next stage
                    }
                }
                this->state.offload.stage =
                    RobotControl::State::OffloadingStage::RAISING_HOPPER;
                // fallthrough and process the next stage
            }
            case RobotControl::State::OffloadingStage::RAISING_HOPPER:
            {
                if (!cancelled &&
                    this->get_hopper_pot() < RobotControl::OFFLOAD_POT_VALUE)
                {
                    this->motor_commands.hopper_actuator
                        .set__mode(TalonCtrl::PERCENT_OUTPUT)
                        .set__value(
                            RobotControl::HOPPER_ACUTATOR_MOVE_SPEED);  // dump
                    break;
                }
                else
                {
                    util::disableMotor(this->motor_commands.hopper_actuator);
                    this->state.offload.dump_start_time = system_time::now();
                    this->state.offload.stage =
                        RobotControl::State::OffloadingStage::OFFLOADING;
                    // fallthrough
                }
            }
            case RobotControl::State::OffloadingStage::OFFLOADING:
            {
                if (!cancelled &&
                    util::seconds_since(this->state.offload.dump_start_time) <
                        this->state.offload.target_dump_time)
                {
                    // set hopper belt
                    this->motor_commands.hopper_belt
                        .set__mode(TalonCtrl::VELOCITY)
                        .set__value(RobotControl::HOPPER_BELT_MAX_VELO);
                    break;
                }
                else
                {
                    util::disableMotor(this->motor_commands.hopper_belt);
                    this->state.offload.stage =
                        RobotControl::State::OffloadingStage::LOWERING_HOPPER;
                    // fallthrough
                }
            }
            case RobotControl::State::OffloadingStage::LOWERING_HOPPER:
            {
                if (this->get_hopper_pot() > RobotControl::TRAVERSAL_POT_VALUE)
                {
                    this->motor_commands.hopper_actuator
                        .set__mode(TalonCtrl::PERCENT_OUTPUT)
                        .set__value(-RobotControl::HOPPER_ACUTATOR_MOVE_SPEED);
                    break;
                }
                else
                {
                    util::disableMotor(this->motor_commands.hopper_actuator);
                    this->state.offload.stage =
                        RobotControl::State::OffloadingStage::FINISHED;
                    // fallthrough
                }
            }
            case RobotControl::State::OffloadingStage::FINISHED:
            {
                this->stop_all();
                this->state.offload.enabled = false;
                this->state.handle_change_control_level(
                    RobotControl::State::ControlLevel::
                        MANUAL);  // TODO: needs to change for full auto?
            }
            default:
            {
                // nothing
            }
        }
    }
}



void RobotControl::periodic_handle_teleop_input()
{
    // speed changes -- these don't affect autos so we can run them during every loop to make it as intuitive as possible
    {
        if (this->getButtonPressed(RobotControl::TELEOP_LOW_SPEED_BUTTON_IDX))
        {
            this->state.driving_speed_scalar =
                RobotControl::DRIVING_LOW_SPEED_SCALAR;
        }
        if (this->getButtonPressed(
                RobotControl::TELEOP_MEDIUM_SPEED_BUTTON_IDX))
        {
            this->state.driving_speed_scalar =
                RobotControl::DRIVING_MEDIUM_SPEED_SCALAR;
        }
        if (this->getButtonPressed(RobotControl::TELEOP_HIGH_SPEED_BUTTON_IDX))
        {
            this->state.driving_speed_scalar =
                RobotControl::DRIVING_HIGH_SPEED_SCALAR;
        }
    }

    // ------------ HARD RESET ------------
    if (this->getRawButton(RobotControl::DISABLE_ALL_ACTIONS_BUTTON_IDX))
    {
        this->stop_all();  // gets called later
        this->state.control_level = RobotControl::State::ControlLevel::MANUAL;
        this->state.last_manual_control_level = this->state.control_level;
        this->cancel_mining();
        this->cancel_offload();
        return;
    }

    const bool is_mining = this->state.mining.enabled,
               is_offload = this->state.offload.enabled,
               any_ops_running = (is_mining || is_offload),
               is_teleauto =
                   (this->state.control_level ==
                    RobotControl::State::ControlLevel::TELEAUTO_OP);

    // ---------- TELEAUTO CONTROl ----------
    {
        if (!any_ops_running &&
            this->getPov(
                RobotControl::TELEAUTO_MINING_INIT_POV_ID,
                RobotControl::TELEAUTO_MINING_INIT_POV_VAL))  // dpad top
        {
            this->start_mining(RobotControl::State::ControlLevel::TELEAUTO_OP);
        }
        else if (
            is_teleauto && is_mining &&
            this->getPov(
                RobotControl::TELEAUTO_MINING_STOP_POV_ID,
                RobotControl::TELEAUTO_MINING_STOP_POV_VAL))  // dpad bottom
        {
            this->cancel_mining();
        }
        else if (
            !any_ops_running &&
            this->getPov(
                RobotControl::TELEAUTO_OFFLOAD_INIT_POV_ID,
                RobotControl::TELEAUTO_OFFLOAD_INIT_POV_VAL))  // dpad right
        {
            this->start_offload(RobotControl::State::ControlLevel::TELEAUTO_OP);
        }
        else if (
            is_teleauto && is_offload &&
            this->getPov(
                RobotControl::TELEAUTO_OFFLOAD_STOP_POV_ID,
                RobotControl::TELEAUTO_OFFLOAD_STOP_POV_VAL))  // dpad left
        {
            this->cancel_offload();
        }
    }

    // -------------- ASSISTED CONTROL ------------
    if (this->getButtonPressed(
            RobotControl::ASSISTED_MINING_TOGGLE_BUTTON_IDX) &&
        !this->state.offload.enabled)
    {
        if (this->state.mining.enabled)
        {
            if (this->state.control_level ==
                RobotControl::State::ControlLevel::ASSISTED_MANUAL)
            {
                this->cancel_mining();
            }
        }
        else
        {
            this->start_mining(
                RobotControl::State::ControlLevel::ASSISTED_MANUAL);
        }
    }
    else if (
        this->getButtonPressed(
            RobotControl::ASSISTED_OFFLOAD_TOGGLE_BUTTON_IDX) &&
        !this->state.mining.enabled)
    {
        if (this->state.offload.enabled)
        {
            if (this->state.control_level ==
                RobotControl::State::ControlLevel::ASSISTED_MANUAL)
            {
                this->cancel_offload();
            }
        }
        else
        {
            this->start_offload(
                RobotControl::State::ControlLevel::ASSISTED_MANUAL);
        }
    }

    if (this->state.mining.enabled || this->state.offload.enabled)
    {
        return;
    }

    // ------------ DRIVE CONTROL ----------------
    {
        const double stick_x = this->getRawAxis(
                         RobotControl::TELEOP_DRIVE_X_AXIS_IDX),
                     stick_y = this->getRawAxis(
                         RobotControl::
                             TELEOP_DRIVE_Y_AXIS_IDX),  // forward y is positive
            scaling_speed = RobotControl::TRACKS_MAX_VELO *
                            this->state.driving_speed_scalar;

        auto track_speeds = util::computeWheelScalars(
            stick_x,
            stick_y,
            RobotControl::DRIVING_MAGNITUDE_DEADZONE_SCALAR);

        // set drive velocities
        this->motor_commands.track_right.set__mode(TalonCtrl::VELOCITY)
            .set__value(scaling_speed * track_speeds[0]);
        this->motor_commands.track_left.set__mode(TalonCtrl::VELOCITY)
            .set__value(scaling_speed * track_speeds[1]);
    }
    // ------------ TRENCHER CONTROL -------------
    {
        double trencher_speed = util::normalizeJoyTriggerAxis(
            this->getRawAxis(RobotControl::TELEOP_TRENCHER_SPEED_AXIS_IDX));
        if (this->getRawButton(RobotControl::TELEOP_TRENCHER_INVERT_BUTTON_IDX))
        {
            trencher_speed *= -1.0;
        }

        // set trencher velocity
        this->motor_commands.trencher.set__mode(TalonCtrl::VELOCITY)
            .set__value(RobotControl::TRENCHER_MAX_VELO * trencher_speed);
    }
    // ------------- HOPPER CONTROL --------------
    {
        double hopper_belt_speed = util::normalizeJoyTriggerAxis(
            this->getRawAxis(RobotControl::TELEOP_HOPPER_SPEED_AXIS_IDX));
        if (this->getRawButton(RobotControl::TELEOP_HOPPER_INVERT_BUTTON_IDX))
        {
            hopper_belt_speed *= -1.0;
        }

        // set hopper belt
        this->motor_commands.hopper_belt.set__mode(TalonCtrl::VELOCITY)
            .set__value(RobotControl::HOPPER_BELT_MAX_VELO * hopper_belt_speed);
        // set actutor power
        this->motor_commands.hopper_actuator
            .set__mode(TalonCtrl::PERCENT_OUTPUT)
            .set__value(
                util::apply_deadband(
                    this->getRawAxis(
                        RobotControl::TELEOP_HOPPER_ACTUATE_AXIS_IDX),
                    RobotControl::GENERIC_DEADZONE_SCALAR));
    }
}
