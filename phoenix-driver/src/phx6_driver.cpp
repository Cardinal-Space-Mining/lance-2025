#include <array>
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
using namespace std::chrono_literals;


// --- Motor configs -----------------------------------------------------------

static constexpr double TFX_COMMON_KP = 0.2;
// ^ 0.5 volts added for every turn per second error
static constexpr double TFX_COMMON_KI = 0.05;
// ^ 0.2 volts added for every rotation integrated error
static constexpr double TFX_COMMON_KD = 0.0001;
// ^ 0.0001 volts added for every rotation per second^2 change in error [per second]
static constexpr double TFX_COMMON_KV = 0.12;
// ^ Falcon 500 is a 500kV motor, 500rpm / 1V = 8.333 rps / 1V --> 1/8.33 = 0.12 volts / rps

static constexpr double TFX_COMMON_NEUTRAL_DEADBAND = 0.05;

static constexpr double TFX_COMMON_STATOR_CURRENT_LIMIT = 30.;
static constexpr double TFX_COMMON_SUPPLY_CURRENT_LIMIT = 20.;
static constexpr double TFX_COMMON_PEAK_VOLTAGE = 12.;


// --- Program defaults --------------------------------------------------------

#define DIAG_SERVER_PORT       1250
#define DEFAULT_ARDUINO_DEVICE "/dev/ttyACM0"
#define CAN_INTERFACE          "can_phx6"
#define RIGHT_TRACK_CANID      0
#define LEFT_TRACK_CANID       1
#define TRENCHER_CANID         2
#define HOPPER_BELT_CANID      3

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_SUB_QOS    1

#define MOTOR_STATUS_PUB_DT       50ms
#define TALONFX_MAX_BOOTUP_DELAY  3s
#define TALONFX_POWER_CYCLE_DELAY 0.5s
#define MOTOR_RESTART_DT_THRESH   1.5s


// --- Driver node -------------------------------------------------------------

class Phoenix6Driver : public rclcpp::Node
{
    using Int8Msg = std_msgs::msg::Int8;
    using Int32Msg = std_msgs::msg::Int32;
    using StringMsg = std_msgs::msg::String;
    using HeaderMsg = std_msgs::msg::Header;
    using system_clock = std::chrono::system_clock;

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

    struct RclTalonFX
    {
        enum
        {
            DISABLE_EXTERNAL = 1 << 0,
            DISABLE_INTERNAL = 1 << 1,
            DISABLE_MASK = DISABLE_EXTERNAL | DISABLE_INTERNAL
        };

    public:
        TalonFX motor;
        TalonFXConfiguration config{};

        RclPubPtr<TalonInfoMsg> info_pub;
        RclPubPtr<TalonFaultsMsg> faults_pub;
        RclSubPtr<TalonCtrlMsg> ctrl_sub;

        TalonInfoMsg last_info;
        TalonFaultsMsg last_faults;

        double elapsed_faulted_time{0.};
        system_clock::time_point prev_update_time{
            system_clock::time_point::min()};
        units::angle::turn_t cached_position{0_tr};

        std::atomic<uint8_t> disable_bits{0b00};
        struct State
        {
            bool watchdog_disabled : 1 {false};
            bool internal_disabled : 1 {false};
            bool hardware_disabled : 1 {false};
            bool disconnected      : 1 {false};
            bool need_config       : 1 {true};
            bool need_set_pos      : 1 {true};
        }  //
        state;

    public:
        RclTalonFX(
            const ParamConfig::RclMotorConfig& config,
            rclcpp::Node& node,
            std::function<void(const TalonCtrlMsg&)> ctrl_cb);
        RclTalonFX(RclTalonFX&&);

    public:
        bool isDisabled() const;
        bool isFaulted() const;
        double elapsedFaultTime() const;

    public:
        void updateAndPubInfo(const HeaderMsg& header);
        void pubFaults(const HeaderMsg& header);
        void acceptCtrl(const TalonCtrlMsg& ctrl);
        void setExtEnabled();
        void setExtDisabled();

        void notifyRestart();
        void handleReinit(units::time::second_t timeout = 100_ms);

    protected:
        void neutralOut();
    };

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

    RclPubPtr<Int8Msg> relay_state_pub;
    RclPubPtr<StringMsg> op_pub;
    RclSubPtr<Int32Msg> watchdog_status_sub;

    std::thread motor_state_thread;

    // std::atomic<bool> is_disabled = true;
    std::atomic<bool> thread_enabled = true;
    // system_clock::time_point last_enable_beg_time;
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
    rclcpp::Node& node,
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

void Phoenix6Driver::RclTalonFX::updateAndPubInfo(const HeaderMsg& header)
{
    system_clock::time_point t = system_clock::now();
    if (this->prev_update_time == system_clock::time_point::min())
    {
        this->prev_update_time = t;
    }

    // extract info and publish
    this->last_info.header = header;
    this->last_info << this->motor;
    this->last_info.status |= static_cast<uint8_t>(!this->isDisabled());
    this->info_pub->publish(this->last_info);

    auto new_state = this->state;
    new_state.watchdog_disabled = (this->disable_bits & DISABLE_EXTERNAL);
    new_state.internal_disabled = (this->disable_bits & DISABLE_INTERNAL);
    new_state.hardware_disabled =
        !(this->last_info.status & TalonInfoMsg::STATUS_PHX_ENABLED);
    new_state.disconnected =
        !(this->last_info.status & TalonInfoMsg::STATUS_CONNECTED);

    // if hard disabled while NOT soft disabled AND NOT disconnected --> fault state
    // accumulate time in fault state --> if 'rising edge', take half of update dt, otherwise full dt
    if (new_state.hardware_disabled && !new_state.disconnected &&
        !new_state.watchdog_disabled)
    {
        const double dt =
            std::chrono::duration<double>(t - this->prev_update_time).count();

        // if previous state also fault, accumulate for full dt
        if (this->state.hardware_disabled && !this->state.disconnected &&
            !this->state.watchdog_disabled)
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

    // if connected and don't need to set pos, cache last position
    if (!new_state.disconnected && !this->state.need_set_pos)
    {
        // this is as a backup - we also cache right before triggering a restart
        this->cached_position = units::angle::turn_t{this->last_info.position};
    }

    this->state = new_state;
}
void Phoenix6Driver::RclTalonFX::pubFaults(const HeaderMsg& header)
{
    this->last_faults.header = header;
    this->last_faults << this->motor;
    this->faults_pub->publish(this->last_faults);
}

void Phoenix6Driver::RclTalonFX::acceptCtrl(const TalonCtrlMsg& ctrl)
{
    // if either watchdog or reset disable bits set, don't output
    if (this->isDisabled())
    {
        this->neutralOut();
    }
    else
    {
        this->motor << ctrl;
    }
    // TODO: return status
}
void Phoenix6Driver::RclTalonFX::setExtEnabled()
{
    this->disable_bits &= ~DISABLE_EXTERNAL;
}
void Phoenix6Driver::RclTalonFX::setExtDisabled()
{
    this->disable_bits |= DISABLE_EXTERNAL;
    this->neutralOut();
}

void Phoenix6Driver::RclTalonFX::notifyRestart()
{
    // disable output -- does not conflict with watchdog disable bit
    this->disable_bits |= DISABLE_INTERNAL;
    this->neutralOut();

    // cache position and set prereq bits for handler to resolve after the reset
    this->cached_position = this->motor.GetPosition().GetValue();
    // don't set `need config` bit -- this should persist? (init value has this set so handler should resolve on startup)
    this->state.need_set_pos = true;
}
void Phoenix6Driver::RclTalonFX::handleReinit(units::time::second_t timeout)
{
    system_clock::time_point beg_t = system_clock::now();
    const auto max_dt = std::chrono::duration<double>(timeout.value());

    if (this->state.need_config)
    {
        if (this->motor.GetConfigurator().Apply(this->config, timeout).IsOK())
        {
            this->state.need_config = false;
        }
    }

    const std::chrono::duration<double> elapsed = (system_clock::now() - beg_t);

    if (elapsed >= max_dt)
    {
        return;
    }
    if (this->state.need_set_pos)
    {
        if (this->motor
                .SetPosition(
                    this->cached_position,
                    units::time::second_t{(max_dt - elapsed).count()})
                .IsOK())
        {
            this->state.need_set_pos = false;
        }
    }
}

void Phoenix6Driver::RclTalonFX::neutralOut()
{
    this->motor.SetControl(phx6::controls::NeutralOut{});
}

// --- Serial Relay ------------------------------------------------------------

Phoenix6Driver::SerialRelay::SerialRelay(const char* device)
{
    this->init(device);
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
        std::cout << "Failed to open serial port " << device
                  << " for motor resets!" << std::endl;
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
}
void Phoenix6Driver::SerialRelay::enable()
{
    if (this->isValid())
    {
        (void)write(this->port, "1", 1);
        this->state = true;
    }
}
void Phoenix6Driver::SerialRelay::disable()
{
    if (this->isValid())
    {
        (void)write(this->port, "0", 1);
        this->state = false;
    }
}



// --- Driver Node -------------------------------------------------------------

Phoenix6Driver::Phoenix6Driver() :
    Node{"phoenix6_driver"},
    relay_state_pub{this->create_publisher<Int8Msg>(
        ROBOT_TOPIC("relay_status"),
        rclcpp::SensorDataQoS{})},
    op_pub{this->create_publisher<StringMsg>(
        ROBOT_TOPIC("phx6_op_status"),
        rclcpp::SensorDataQoS{})},
    watchdog_status_sub{this->create_subscription<Int32Msg>(
        ROBOT_TOPIC("watchdog_status"),
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& msg) { this->feed_watchdog_status(msg.data); })}
{
    ParamConfig params;
    this->getParams(params);
    this->initPhx(params);
    this->initMotors(params);
    this->initThread();

    RCLCPP_DEBUG(
        this->get_logger(),
        "Completed Phoenix6 Driver Node Initialization");
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
        this,
        "diagnostics_port",
        params.diagnostics_server_port,
        DIAG_SERVER_PORT);
    declare_param<std::string>(this, "canbus", params.canbus, CAN_INTERFACE);
    declare_param<std::string>(
        this,
        "arduino_device",
        params.arduino_device,
        DEFAULT_ARDUINO_DEVICE);

    ParamConfig::RclMotorConfig defaults;
    defaults.canbus = params.canbus;

    declare_param(this, "common.kP", defaults.kP, TFX_COMMON_KP);
    declare_param(this, "common.kI", defaults.kI, TFX_COMMON_KI);
    declare_param(this, "common.kD", defaults.kD, TFX_COMMON_KD);
    declare_param(this, "common.kV", defaults.kV, TFX_COMMON_KV);
    declare_param(
        this,
        "common.neutral_deadband",
        defaults.neutral_deadband,
        TFX_COMMON_NEUTRAL_DEADBAND);
    declare_param(
        this,
        "common.stator_current_limit",
        defaults.stator_current_limit,
        TFX_COMMON_STATOR_CURRENT_LIMIT);
    declare_param(
        this,
        "common.supply_current_limit",
        defaults.supply_current_limit,
        TFX_COMMON_SUPPLY_CURRENT_LIMIT);
    declare_param(
        this,
        "common.voltage_limit",
        defaults.voltage_limit,
        TFX_COMMON_PEAK_VOLTAGE);

    bool use_neutral_brake;
    declare_param(this, "common.neutral_brake", use_neutral_brake, false);
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
    for (const auto& m_conf : params.motor_configs)
    {
        size_t idx = this->motors.size();
        this->motors.emplace_back(
            m_conf,
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
    // std::cout << ">> begin feed watchdog status" << std::endl;
    /* Watchdog feed decoding:
     * POSTIVE feed time --> enabled
     * ZERO feed time --> disabled
     * NEGATIVE feed time --> autonomous */
    if (!status)
    {
        // this->neutralAll();
        for (auto& m : this->motors)
        {
            m.setExtDisabled();
        }
        // this->is_disabled = true;
    }
    else
    {
        for (auto& m : this->motors)
        {
            m.setExtEnabled();
        }
        // if (this->is_disabled)
        // {
        //     this->last_enable_beg_time = system_clock::now();
        //     // std::cout << "Applied new last enabled state change time "
        //     //           << this->last_enable_beg_time.time_since_epoch().count()
        //     //           << std::endl;
        // }
        ctre::phoenix::unmanaged::FeedEnable(std::abs(status));
        // this->is_disabled = false;
        // std::cout << "watchdog set disabled to false" << std::endl;
    }
    // this->relay_state_pub->publish(Int8Msg{}.set__data(1));
    // std::cout << ">> end feed watchdog status" << std::endl;
}

void Phoenix6Driver::handle_motor_states()
{
    constexpr double MAX_INFO_PUB_FREQ = 20.;
    constexpr double MAX_FAULT_PUB_FREQ = 4.;

    constexpr auto MIN_INFO_PUB_DT = (1s / MAX_INFO_PUB_FREQ);
    constexpr auto MIN_FAULT_PUB_DT = (1s / MAX_FAULT_PUB_FREQ);

    constexpr double FAULT_DT_THRESH = 1.0;

    system_clock::time_point prev_info_pub_t, prev_fault_pub_t;

    while (this->thread_enabled)
    {
        auto beg_t = system_clock::now();

        const auto info_dt = beg_t - prev_info_pub_t;
        bool any_faulted = false;
        if (info_dt > MIN_INFO_PUB_DT)
        {
            auto t1 = system_clock::now();
            HeaderMsg header;
            header.stamp = this->get_clock()->now();
            for (auto& m : this->motors)
            {
                m.updateAndPubInfo(header);
                any_faulted |= (m.elapsedFaultTime() >= FAULT_DT_THRESH);
            }
            auto t2 = system_clock::now();

            auto dt = t2 - t1;
            if (dt > 10ms)
            {
                std::cout << "Motor info pub took >10ms ("
                          << (std::chrono::duration<double>(dt).count() * 1000.)
                          << "ms)" << std::endl;
            }

            prev_info_pub_t = beg_t;
        }

        const auto fault_dt = beg_t - prev_fault_pub_t;
        if (fault_dt > MIN_FAULT_PUB_DT)
        {
            auto t1 = system_clock::now();
            HeaderMsg header;
            header.stamp = this->get_clock()->now();
            for (auto& m : this->motors)
            {
                m.pubFaults(header);
            }
            auto t2 = system_clock::now();

            auto dt = t2 - t1;
            if (dt > 10ms)
            {
                std::cout << "Motor faults pub took >10ms ("
                          << (std::chrono::duration<double>(dt).count() * 1000.)
                          << "ms)" << std::endl;
            }

            prev_fault_pub_t = beg_t;
        }

        // handle relay restart here
        // ... notify all motors to set states accordingly
        if (any_faulted)
        {
            for (auto& m : this->motors)
            {
                m.notifyRestart();
            }
            this->relay.disable();
            // pub status
            std::this_thread::sleep_for(TALONFX_POWER_CYCLE_DELAY);
            this->relay.enable();
            // pub status again
        }

        // call motor handlers for configuring/applying cached position (or add to pubInfo?)
        for (auto& m : this->motors)
        {
            // TODO: calculate timeout for each depending on time until next loop
            m.handleReinit();
        }

        std::this_thread::sleep_until(
            std::min(
                prev_info_pub_t + MIN_INFO_PUB_DT,
                prev_fault_pub_t + MIN_FAULT_PUB_DT));
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
