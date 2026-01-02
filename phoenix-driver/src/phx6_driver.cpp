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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <cstring>
#include <iostream>
#include <functional>
#include <string_view>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#define Phoenix_No_WPI  // remove WPI dependencies
#include <ctre/phoenix/cci/Diagnostics_CCI.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include "ros_utils.hpp"
#include "phx6_utils.hpp"


using namespace util;
using namespace util::ros_aliases;
using namespace std::chrono_literals;


// --- Program configs ---------------------------------------------------------

#define DIAG_SERVER_PORT       1250
#define DEFAULT_ARDUINO_DEVICE "/dev/ttyACM0"
#define CAN_INTERFACE          "can_phx6"
#define RIGHT_TRACK_CANID      0
#define LEFT_TRACK_CANID       1
#define TRENCHER_CANID         2
#define HOPPER_BELT_CANID      3

#define TFX_DEFAULT_KP 0.2
// ^ 0.5 volts added for every turn per second error
#define TFX_DEFAULT_KI 0.05
// ^ 0.2 volts added for every rotation integrated error
#define TFX_DEFAULT_KD 0.0001
// ^ 0.0001 volts added for every rotation per second^2 change in error [per second]
#define TFX_DEFAULT_KV 0.12
// ^ Falcon 500 is a 500kV motor, 500rpm / 1V = 8.333 rps / 1V --> 1/8.33 = 0.12 volts / rps

#define TFX_DEFAULT_NEUTRAL_DEADBAND     0.05
#define TFX_DEFAULT_STATOR_CURRENT_LIMIT 30.
#define TFX_DEFAULT_SUPPLY_CURRENT_LIMIT 20.
#define TFX_DEFAULT_PEAK_VOLTAGE         12.

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_SUB_QOS    1

#define MOTOR_INFO_PUB_FREQ               20.
#define MOTOR_FAULT_PUB_FREQ              4.
#define MOTOR_POWER_CYCLE_DELAY           0.5s
#define MOTOR_DEFAULT_FAULT_TIME_THRESH_S 1.5

static constexpr auto NOMINAL_INFO_PUB_DT =
    std::chrono::duration_cast<std::chrono::system_clock::duration>(
        1s / MOTOR_INFO_PUB_FREQ);
static constexpr auto NOMINAL_FAULT_PUB_DT =
    std::chrono::duration_cast<std::chrono::system_clock::duration>(
        1s / MOTOR_FAULT_PUB_FREQ);


// --- Driver node -------------------------------------------------------------

class Phoenix6Driver : public rclcpp::Node
{
    using Int8Msg = std_msgs::msg::Int8;
    using Int32Msg = std_msgs::msg::Int32;
    using StringMsg = std_msgs::msg::String;
    using HeaderMsg = std_msgs::msg::Header;
    using system_clock = std::chrono::system_clock;

    // Temporary parameter buffer
    struct ParamConfig
    {
        struct RclMotorConfig
        {
            int can_id{-1};
            std::string_view canbus;
            std::string topic_prefix;

            int neutral_mode_val;
            int invert_mode_val;
            double kP;
            double kI;
            double kD;
            double kV;
            double neutral_deadband;
            double stator_current_limit;
            double supply_current_limit;
            double voltage_limit;

            TalonFXConfiguration buildFXConfig() const;
        };

        int diagnostics_server_port{0};
        std::string arduino_device;
        std::string canbus;
        std::vector<RclMotorConfig> motor_configs;
    };

    // Phx motor instance + pubsubs + state managers
    struct RclTalonFX
    {
        enum
        {
            DISABLE_WATCHDOG = 1 << 0,
            DISABLE_REINIT = 1 << 1,
            DISABLE_MASK = DISABLE_WATCHDOG | DISABLE_REINIT
        };

    public:
        TalonFX motor;
        TalonFXConfiguration config{};

        SharedPub<TalonInfoMsg> info_pub;
        SharedPub<TalonFaultsMsg> faults_pub;
        SharedSub<TalonCtrlMsg> ctrl_sub;

        TalonInfoMsg last_info;
        TalonFaultsMsg last_faults;

        double elapsed_faulted_time{0.};
        system_clock::time_point prev_update_time{
            system_clock::time_point::min()};
        // system_clock::time_point last_reset_time;
        units::angle::turn_t cached_position{0_tr};

        // start fully disabled (require config and pos_set)
        std::atomic<uint8_t> disable_bits{DISABLE_WATCHDOG | DISABLE_REINIT};
        struct State
        {
            bool watchdog_disabled : 1 {false};
            bool reinit_disabled   : 1 {false};
            bool hardware_disabled : 1 {false};
            bool disconnected      : 1 {false};
            bool phx_disabled      : 1 {false};
            bool has_reset         : 1 {false};
            bool need_config       : 1 {true};
            bool need_set_pos      : 1 {true};

            inline bool isFault() const
            {
                return !phx_disabled && hardware_disabled && !disconnected;
            }
        }  //
        state;

    public:
        RclTalonFX(
            const ParamConfig::RclMotorConfig& config,
            RclNode& node,
            std::function<void(const TalonCtrlMsg&)> ctrl_cb);
        RclTalonFX(RclTalonFX&&);

    public:
        bool isDisabled() const;
        bool isFaulted() const;
        double elapsedFaultTime() const;

    public:
        void updateState();
        void updateAndPubInfo(const HeaderMsg& header);
        void pubFaults(const HeaderMsg& header);
        phx_::StatusCode acceptCtrl(const TalonCtrlMsg& ctrl);
        void setWatchdogEnabled();
        void setWatchdogDisabled();

        void notifyRestart();
        void handleReinit(std::chrono::duration<double> timeout = 100ms);

    protected:
        phx_::StatusCode neutralOut();
    };

    // Serial interface for triggering relay
    class SerialRelay
    {
    public:
        SerialRelay(const char* device = "");
        ~SerialRelay();

    public:
        void init(const char* device);
        void enable();
        void disable();
        inline bool isValid() const { return this->port >= 0; }
        inline bool isEnabled() const { return this->state; }

    private:
        int port{-1};
        std::atomic<bool> state{false};
    };

public:
    Phoenix6Driver();
    ~Phoenix6Driver();

private:
    void getParams(ParamConfig& buff);
    void initSerial(const ParamConfig& buff);
    void initPhx(const ParamConfig& buff);
    void initMotors(const ParamConfig& buff);
    void initThread();

    void feed_watchdog_status(int32_t status);
    void handle_motor_states();

private:
    SerialRelay relay;
    std::vector<RclTalonFX> motors;

    SharedPub<Int8Msg> relay_state_pub;
    SharedSub<Int32Msg> watchdog_status_sub;

    std::thread motor_state_thread;
    std::atomic<bool> thread_enabled = true;

    double fault_thresh_s;
};





// --- Configs -----------------------------------------------------------------

TalonFXConfiguration
    Phoenix6Driver::ParamConfig::RclMotorConfig::buildFXConfig() const
{
    return ::buildFXConfig(
        this->kP,
        this->kI,
        this->kD,
        this->kV,
        this->neutral_deadband,
        this->neutral_mode_val,
        this->invert_mode_val,
        this->stator_current_limit,
        this->supply_current_limit,
        this->voltage_limit);
}

// --- RclTalonFX --------------------------------------------------------------

Phoenix6Driver::RclTalonFX::RclTalonFX(
    const ParamConfig::RclMotorConfig& config,
    RclNode& node,
    std::function<void(const TalonCtrlMsg&)> ctrl_cb) :
    motor{config.can_id, std::string{config.canbus}},
    config{config.buildFXConfig()},
    info_pub{node.create_publisher<TalonInfoMsg>(
        config.topic_prefix + "/info",
        rclcpp::SensorDataQoS{})},
    faults_pub{node.create_publisher<TalonFaultsMsg>(
        config.topic_prefix + "/faults",
        rclcpp::SensorDataQoS{})},
    ctrl_sub{node.create_subscription<TalonCtrlMsg>(
        config.topic_prefix + "/ctrl",
        TALON_CTRL_SUB_QOS,
        ctrl_cb)}
{
}
Phoenix6Driver::RclTalonFX::RclTalonFX(RclTalonFX&& fx) :
    motor{fx.motor.GetDeviceID(), fx.motor.GetNetwork()},
    config{fx.config},
    info_pub{std::move(fx.info_pub)},
    faults_pub{std::move(fx.faults_pub)},
    ctrl_sub{std::move(fx.ctrl_sub)},
    last_info{std::move(fx.last_info)},
    last_faults{std::move(fx.last_faults)},
    elapsed_faulted_time{fx.elapsed_faulted_time},
    prev_update_time{fx.prev_update_time},
    cached_position{fx.cached_position},
    disable_bits{fx.disable_bits.load()},
    state{fx.state}
{
}

bool Phoenix6Driver::RclTalonFX::isDisabled() const
{
    return static_cast<bool>(this->disable_bits & DISABLE_MASK);
}
bool Phoenix6Driver::RclTalonFX::isFaulted() const
{
    return this->elapsed_faulted_time > 0.;
}
double Phoenix6Driver::RclTalonFX::elapsedFaultTime() const
{
    return this->elapsed_faulted_time;
}

void Phoenix6Driver::RclTalonFX::updateState()
{
    system_clock::time_point curr_t = system_clock::now();
    if (this->prev_update_time == system_clock::time_point::min())
    {
        this->prev_update_time = curr_t;
    }

    // construct a new state - copy old values of need_config and need_set_pos, update everything else
    RclTalonFX::State new_state = this->state;
    new_state.watchdog_disabled = (this->disable_bits & DISABLE_WATCHDOG);
    new_state.reinit_disabled = (this->disable_bits & DISABLE_REINIT);
    new_state.hardware_disabled =
        !this->motor.GetDeviceEnable().GetValue().value;
    new_state.disconnected = !this->motor.IsConnected();
    new_state.phx_disabled = !ctre::phoenix::unmanaged::GetEnableState();
    new_state.has_reset = this->motor.HasResetOccurred();

    // if hard disabled while NOT externally disabled AND NOT internally disabled AND NOT disconnected --> fault state
    // accumulate time in fault state --> if 'rising edge', take half of update dt, otherwise full dt
    if (new_state.isFault())
    {
        const double dt =
            std::chrono::duration<double>(curr_t - this->prev_update_time)
                .count();

        // if previous state also fault, accumulate for full dt
        if (this->state.isFault())
        {
            this->elapsed_faulted_time += dt;
        }
        // otherwise, use half dt
        else
        {
            this->elapsed_faulted_time += dt * 0.5;
        }
    }
    // no fault, reset elapsed
    else
    {
        this->elapsed_faulted_time = 0.;
    }

    // on power-cycle, position gets reset and immediately cached unless we specifically check for this state
    if ((!new_state.disconnected && this->state.disconnected) ||
        new_state.has_reset)
    {
        // this->last_reset_time = curr_t;
        new_state.need_set_pos = true;
        this->disable_bits |= DISABLE_REINIT;
    }

    // if connected and don't need to set pos, cache last position
    if (!new_state.disconnected && !new_state.need_set_pos)
    {
        // this is as a backup - we also cache right before triggering a restart
        this->cached_position = units::angle::turn_t{this->last_info.position};
    }

    this->prev_update_time = curr_t;
    this->state = new_state;
}

void Phoenix6Driver::RclTalonFX::updateAndPubInfo(const HeaderMsg& header)
{
    this->updateState();

    // extract info and publish
    this->last_info.header = header;
    serializeTalonInfoNoStatus(this->last_info, this->motor);

    // if we are trying to re-apply the cached position, don't trust the current value from the motor
    if (this->state.need_set_pos)
    {
        this->last_info.position = this->cached_position.value();
    }

    // first 3 bits are 'message spec', rest are extra and only defined here
    this->last_info.status =
        // first bit "is hardware enabled"
        (static_cast<uint8_t>(!this->state.hardware_disabled) << 0 |
         // second bit "is connected"
         static_cast<uint8_t>(!this->state.disconnected) << 1 |
         // third bit "has reset occurred"
         static_cast<uint8_t>(this->state.has_reset) << 2 |
         // fourth bit "is phx enabled"
         static_cast<uint8_t>(!this->state.phx_disabled) << 3 |
         // fifth bit "is watchdog allowing enabled"
         static_cast<uint8_t>(!this->state.watchdog_disabled) << 4 |
         // sixth bit "is NOT disabled because of reinit"
         static_cast<uint8_t>(!this->state.reinit_disabled) << 5 |
         // seventh bit "has been configured properly"
         static_cast<uint8_t>(!this->state.need_config) << 6 |
         // eight bit "has position been set correctly"
         static_cast<uint8_t>(!this->state.need_set_pos) << 7);

    this->info_pub->publish(this->last_info);
}
void Phoenix6Driver::RclTalonFX::pubFaults(const HeaderMsg& header)
{
    this->last_faults.header = header;
    this->last_faults << this->motor;
    this->faults_pub->publish(this->last_faults);

    if (this->last_faults.sticky_faults)
    {
        this->motor.ClearStickyFaults();
    }
}

phx_::StatusCode Phoenix6Driver::RclTalonFX::acceptCtrl(
    const TalonCtrlMsg& ctrl)
{
    // if either watchdog or reset disable bits set, don't output
    if (this->isDisabled())
    {
        return this->neutralOut();
    }
    else
    {
        return (this->motor << ctrl);
    }
}
void Phoenix6Driver::RclTalonFX::setWatchdogEnabled()
{
    this->disable_bits &= ~DISABLE_WATCHDOG;
}
void Phoenix6Driver::RclTalonFX::setWatchdogDisabled()
{
    this->disable_bits |= DISABLE_WATCHDOG;
    this->neutralOut();
}

void Phoenix6Driver::RclTalonFX::notifyRestart()
{
    std::cout << "Notified reset for motor (" << this->motor.GetDeviceID()
              << ", " << this->motor.GetNetwork() << ")" << std::endl;
    // disable output -- does not conflict with watchdog disable bit
    // this doesn't get cleared until motor is reconnected and successfully gets currently cached pos
    this->disable_bits |= DISABLE_REINIT;
    this->neutralOut();

    // restart fault timer since a fault state may still be detected on restart
    this->elapsed_faulted_time = 0.;
    // cache position and set prereq bits for handler to resolve after the reset
    if (!this->state.disconnected)
    {
        // don't cache if not connected -- use last backup (or init value) if the motor ever reconnects
        this->cached_position = this->motor.GetPosition().GetValue();
    }
    // don't set `need config` bit -- this should persist? (init value has this set so handler should resolve on startup)
    this->state.need_set_pos = true;
}
void Phoenix6Driver::RclTalonFX::handleReinit(
    std::chrono::duration<double> timeout)
{
    system_clock::time_point beg_t = system_clock::now();

    // handle refreshing state (after relay restart state will be old)
    if ((beg_t - this->prev_update_time) > NOMINAL_INFO_PUB_DT)
    {
        this->updateState();
    }
    // skip if disconnected
    if (this->state.disconnected)
    {
        return;
    }

    // attempt to configure if bit is set
    if (this->state.need_config)
    {
        if (this->motor.GetConfigurator()
                .Apply(this->config, units::time::second_t{timeout.count()})
                .IsOK())
        {
            std::cout << "Successfully configured motor! ("
                      << this->motor.GetDeviceID() << ", "
                      << this->motor.GetNetwork() << ")" << std::endl;
            // clear bit on success
            this->state.need_config = false;
        }
    }

    // attempt to apply last pos if bit is set and timeout not reached
    if (this->state.need_set_pos)
    {
        const std::chrono::duration<double> elapsed =
            (system_clock::now() - beg_t);
        if (elapsed >= timeout)
        {
            return;
        }
        if (this->motor
                .SetPosition(
                    this->cached_position,
                    units::time::second_t{(timeout - elapsed).count()})
                .IsOK())
        {
            std::cout << "Successfully set position for motor! ("
                      << this->motor.GetDeviceID() << ", "
                      << this->motor.GetNetwork() << ") -- "
                      << this->cached_position.value() << std::endl;
            // clear bit on success
            this->state.need_set_pos = false;
        }
    }

    // if both bits clear, remove internal control blocking
    if (!this->state.need_config && !this->state.need_set_pos)
    {
        this->disable_bits &= ~DISABLE_REINIT;
    }
}

phx_::StatusCode Phoenix6Driver::RclTalonFX::neutralOut()
{
    return this->motor.SetControl(phx6::controls::NeutralOut{});
}

// --- Serial Relay ------------------------------------------------------------

Phoenix6Driver::SerialRelay::SerialRelay(const char* device)
{
    if (device && *device)
    {
        this->init(device);
    }
}
Phoenix6Driver::SerialRelay::~SerialRelay()
{
    if (this->isValid())
    {
        close(this->port);
    }
}

void Phoenix6Driver::SerialRelay::init(const char* device)
{
    this->port = open(device, O_RDWR | O_NOCTTY | O_SYNC);

    if (this->port < 0)
    {
        std::cout << "Failed to open serial device (" << device
                  << ") for motor resets!" << std::endl;
        return;
    }

    // Configure port
    termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(this->port, &tty) != 0)
    {
        std::cout << "Error configuring (tcgetattr) serial port " << device
                  << " for motor resets!" << std::endl;
        this->port = -1;
        return;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    // enable receiver, ignore modem ctrl lines
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    // 8 bits
    tty.c_cflag |= CS8;
    // no parity
    tty.c_cflag &= ~PARENB;
    // 1 stop bit
    tty.c_cflag &= ~CSTOPB;
    // no flow control
    tty.c_cflag &= ~CRTSCTS;

    // no signaling chars, no echo
    tty.c_lflag = 0;
    // no remapping, no delays
    tty.c_oflag = 0;
    // read doesn't return until at least 1 char
    tty.c_cc[VMIN] = 1;
    // 0.5 seconds read timeout
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(this->port, TCSANOW, &tty) != 0)
    {
        std::cout << "Error configuring (tcsetattr) serial port " << device
                  << " for motor resets!" << std::endl;
        this->port = -1;
        return;
    }

    std::cout << "Successfully configured serial device (" << device
              << ") for motor resets!" << std::endl;
}
void Phoenix6Driver::SerialRelay::enable()
{
    if (this->isValid())
    {
        if (write(this->port, "1", 1) == -1)
        {
            // Error, this should not happen
            std::cerr << "Phoenix6Driver::SerialRelay::enable write failure!"
                      << std::flush;
        };
        this->state = true;
    }
}
void Phoenix6Driver::SerialRelay::disable()
{
    if (this->isValid())
    {
        if (write(this->port, "0", 1) == -1)
        {
            // Error, this should not happen
            std::cerr << "Phoenix6Driver::SerialRelay::disable write failure!"
                      << std::flush;
        };
        this->state = false;
    }
}



// --- Driver Node -------------------------------------------------------------

Phoenix6Driver::Phoenix6Driver() :
    Node{"phoenix6_driver"},
    relay_state_pub{this->create_publisher<Int8Msg>(
        ROBOT_TOPIC("relay_status"),
        rclcpp::SensorDataQoS{})},
    watchdog_status_sub{this->create_subscription<Int32Msg>(
        ROBOT_TOPIC("watchdog_status"),
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& msg) { this->feed_watchdog_status(msg.data); })}
{
    ParamConfig params;
    this->getParams(params);
    this->initSerial(params);
    this->initPhx(params);
    this->initMotors(params);
    this->initThread();
}
Phoenix6Driver::~Phoenix6Driver()
{
    this->thread_enabled = false;
    if (this->motor_state_thread.joinable())
    {
        this->motor_state_thread.join();
    }

    this->relay.disable();

    c_Phoenix_Diagnostics_Dispose();
}


void Phoenix6Driver::getParams(ParamConfig& params)
{
    declare_param(
        *this,
        "diagnostics_port",
        params.diagnostics_server_port,
        DIAG_SERVER_PORT);
    declare_param<std::string>(*this, "canbus", params.canbus, CAN_INTERFACE);
    declare_param<std::string>(
        *this,
        "arduino_device",
        params.arduino_device,
        DEFAULT_ARDUINO_DEVICE);

    // --- IMPORTANT! ----------------------------------------------------------
    // Configure this to be the maximum amount of time that it takes any motor
    // to initialize after being powered on, or during a "soft"
    // (temporary disable) current limit event - from testing this has been
    // ~1.25 seconds, so the default is 1.5
    declare_param(
        *this,
        "power_cycle_fault_thresh_s",
        this->fault_thresh_s,
        MOTOR_DEFAULT_FAULT_TIME_THRESH_S);

    ParamConfig::RclMotorConfig defaults;
    defaults.canbus = params.canbus;

    declare_param(*this, "common.kP", defaults.kP, TFX_DEFAULT_KP);
    declare_param(*this, "common.kI", defaults.kI, TFX_DEFAULT_KI);
    declare_param(*this, "common.kD", defaults.kD, TFX_DEFAULT_KD);
    declare_param(*this, "common.kV", defaults.kV, TFX_DEFAULT_KV);
    declare_param(
        *this,
        "common.neutral_deadband",
        defaults.neutral_deadband,
        TFX_DEFAULT_NEUTRAL_DEADBAND);
    declare_param(
        *this,
        "common.stator_current_limit",
        defaults.stator_current_limit,
        TFX_DEFAULT_STATOR_CURRENT_LIMIT);
    declare_param(
        *this,
        "common.supply_current_limit",
        defaults.supply_current_limit,
        TFX_DEFAULT_SUPPLY_CURRENT_LIMIT);
    declare_param(
        *this,
        "common.voltage_limit",
        defaults.voltage_limit,
        TFX_DEFAULT_PEAK_VOLTAGE);

    bool use_neutral_brake{};
    declare_param(*this, "common.neutral_brake", use_neutral_brake, false);
    defaults.neutral_mode_val =
        use_neutral_brake ? NeutralModeValue::Brake : NeutralModeValue::Coast;

    params.motor_configs.resize(4, defaults);

    auto& track_right_config = params.motor_configs[0];
    track_right_config.topic_prefix = ROBOT_TOPIC("track_right");
    track_right_config.can_id = RIGHT_TRACK_CANID;
    track_right_config.invert_mode_val =
        InvertedValue::CounterClockwise_Positive;

    auto& track_left_config = params.motor_configs[1];
    track_left_config.topic_prefix = ROBOT_TOPIC("track_left");
    track_left_config.can_id = LEFT_TRACK_CANID;
    track_left_config.invert_mode_val = InvertedValue::Clockwise_Positive;

    auto& trencher_config = params.motor_configs[2];
    trencher_config.topic_prefix = ROBOT_TOPIC("trencher");
    trencher_config.can_id = TRENCHER_CANID;
    trencher_config.invert_mode_val = InvertedValue::Clockwise_Positive;

    auto& hopper_belt_config = params.motor_configs[3];
    hopper_belt_config.topic_prefix = ROBOT_TOPIC("hopper_belt");
    hopper_belt_config.can_id = HOPPER_BELT_CANID;
    hopper_belt_config.invert_mode_val = InvertedValue::Clockwise_Positive;
}

void Phoenix6Driver::initSerial(const ParamConfig& buff)
{
    this->relay.init(buff.arduino_device.c_str());
    this->relay.enable();
}

void Phoenix6Driver::initPhx(const ParamConfig& params)
{
    if (params.diagnostics_server_port > 0)
    {
        c_Phoenix_Diagnostics_Create_On_Port(params.diagnostics_server_port);
    }
    else
    {
        c_Phoenix_Diagnostics_SetSecondsToStart(-1);
    }
}

void Phoenix6Driver::initMotors(const ParamConfig& params)
{
    this->motors.reserve(params.motor_configs.size());
    for (const auto& config : params.motor_configs)
    {
        size_t idx = this->motors.size();
        this->motors.emplace_back(
            config,
            *this,
            [this, idx](const TalonCtrlMsg& ctrl)
            { this->motors[idx].acceptCtrl(ctrl); });
    }
}

void Phoenix6Driver::initThread()
{
    this->motor_state_thread =
        std::thread(&Phoenix6Driver::handle_motor_states, this);
}



void Phoenix6Driver::feed_watchdog_status(int32_t status)
{
    /* Watchdog feed decoding:
     * POSTIVE feed time --> enabled
     * ZERO feed time --> disabled
     * NEGATIVE feed time --> autonomous */
    if (!status)
    {
        for (auto& m : this->motors)
        {
            m.setWatchdogDisabled();
        }
    }
    else
    {
        ctre::phoenix::unmanaged::FeedEnable(std::abs(status));
        for (auto& m : this->motors)
        {
            m.setWatchdogEnabled();
        }
    }
}

void Phoenix6Driver::handle_motor_states()
{
    system_clock::time_point next_info_pub_t, next_fault_pub_t;
    next_info_pub_t = next_fault_pub_t = system_clock::now();

    while (this->thread_enabled)
    {
        // handle update motor state and pub info
        auto curr_t = system_clock::now();
        bool need_restart = false;
        if (curr_t >= next_info_pub_t)
        {
            HeaderMsg header;
            header.stamp = this->get_clock()->now();
            double longest_elapsed_fault = 0.;
            for (auto& m : this->motors)
            {
                m.updateAndPubInfo(header);
                longest_elapsed_fault =
                    std::max(longest_elapsed_fault, m.elapsedFaultTime());
            }

            if (longest_elapsed_fault > 0.)
            {
                std::cout << "Longest detected fault duration was "
                          << longest_elapsed_fault << "s";
                if (longest_elapsed_fault >= this->fault_thresh_s)
                {
                    need_restart = true;
                    std::cout << " -- triggering restart!" << std::endl;
                }
                else
                {
                    std::cout << std::endl;
                }
            }

            next_info_pub_t += NOMINAL_INFO_PUB_DT;
        }

        // handle pub faults
        curr_t = system_clock::now();
        if (curr_t >= next_fault_pub_t)
        {
            HeaderMsg header;
            header.stamp = this->get_clock()->now();
            for (auto& m : this->motors)
            {
                m.pubFaults(header);
            }

            next_fault_pub_t += NOMINAL_FAULT_PUB_DT;
        }

        // if any faults, conduct reset routine
        if (need_restart)
        {
            // notify motors of restart
            for (auto& m : this->motors)
            {
                m.notifyRestart();
            }

            // toggle relay
            this->relay_state_pub->publish(Int8Msg{}.set__data(0));
            this->relay.disable();
            std::this_thread::sleep_for(MOTOR_POWER_CYCLE_DELAY);
            this->relay.enable();

            // re-sync pub targets
            next_info_pub_t = next_fault_pub_t = system_clock::now();
        }
        this->relay_state_pub->publish(Int8Msg{}.set__data(1));

        // handle setting configs for each motor, timing out before the next iteration
        const auto next_loop_t = std::min(next_info_pub_t, next_fault_pub_t);
        for (auto& m : this->motors)
        {
            auto timeout = next_loop_t - system_clock::now();
            if (timeout > 0s)
            {
                m.handleReinit(timeout);
            }
            else
            {
                break;
            }
        }

        std::this_thread::sleep_until(next_loop_t);
    }
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Phoenix6Driver>();
    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) has started");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) shutting down...");
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
