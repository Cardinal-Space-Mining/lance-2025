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

#include <phoenix_ros_driver/msg/talon_ctrl.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>
#include <phoenix_ros_driver/msg/talon_faults.hpp>

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
#define NUM_MOTORS             4

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
    public:
        TalonFX motor;
        TalonFXConfiguration config{};

        RclPubPtr<TalonInfoMsg> info_pub;
        RclPubPtr<TalonFaultsMsg> faults_pub;
        RclSubPtr<TalonCtrlMsg> ctrl_sub;

        units::angle::turn_t cached_position{0_tr};
        std::atomic<bool> disabled{false};

    public:
        RclTalonFX(
            const ParamConfig::RclMotorConfig& config,
            rclcpp::Node& node,
            std::function<void(const TalonCtrlMsg&)> ctrl_cb);
        RclTalonFX(RclTalonFX&&);

    public:
        void pubInfo(TalonInfoMsg& buff);
        void pubFaults(TalonFaultsMsg& buff);
        void acceptCtrl(const TalonCtrlMsg& ctrl);
        void setNeutral();
        void setEnabled();
        void setDisabled();

        void cachePos();
        void applyCachedPos();
        phx_::StatusCode applyConfig(units::time::second_t timeout = 100_ms);
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
    // inline void neutralAll()
    // {
    //     for (auto& m : this->motors)
    //     {
    //         m.setNeutral();
    //     }
    // }

    void getParams(ParamConfig& buff);
    void initSerial(const ParamConfig& buff);
    void initPhx(const ParamConfig& buff);
    void initMotors(const ParamConfig& buff);
    void initCallbacks(const ParamConfig& buff);

    // void parameterizeMotorConfigs();

    // void initSerial(const char*);
    // void sendSerialPowerDown();
    // void sendSerialPowerUp();

    void feed_watchdog_status(int32_t status);
    // Function to setup motors for the robot
    void configure_motors_cb(units::time::second_t timeout = 100_ms);
    // Periodic function for motor information updates
    void pub_motor_info_cb();
    void pub_motor_fault_cb();
    // Function to execute control commands on a motor
    void execute_ctrl_cb(TalonFX& motor, const TalonCtrlMsg& msg);

private:
    SerialRelay relay;
    std::vector<RclTalonFX> motors;

    RclPubPtr<Int8Msg> relay_state_pub;
    RclPubPtr<StringMsg> op_pub;
    RclSubPtr<Int32Msg> watchdog_status_sub;

    RclTimerPtr info_pub_timer;
    RclTimerPtr fault_pub_timer;

    bool is_disabled = true;
    std::chrono::system_clock::time_point last_enable_beg_time;
};




// --- Nested Class ------------------------------------------------------------

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
    this->applyConfig();
}
Phoenix6Driver::RclTalonFX::RclTalonFX(RclTalonFX&& fx) :
    motor{fx.motor.GetDeviceID(), fx.motor.GetNetwork()},
    config{fx.config},
    info_pub{std::move(fx.info_pub)},
    faults_pub{std::move(fx.faults_pub)},
    ctrl_sub{std::move(fx.ctrl_sub)},
    cached_position{fx.cached_position},
    disabled{fx.disabled.load()}
{
}

void Phoenix6Driver::RclTalonFX::pubInfo(TalonInfoMsg& buff)
{
    this->info_pub->publish((buff << this->motor));
}
void Phoenix6Driver::RclTalonFX::pubFaults(TalonFaultsMsg& buff)
{
    this->faults_pub->publish((buff << this->motor));
}
void Phoenix6Driver::RclTalonFX::acceptCtrl(const TalonCtrlMsg& ctrl)
{
    if (this->disabled.load())
    {
        this->setNeutral();
    }
    else
    {
        auto status = (this->motor << ctrl);
    }
}
void Phoenix6Driver::RclTalonFX::setNeutral()
{
    this->motor.SetControl(phx6::controls::NeutralOut{});
}
void Phoenix6Driver::RclTalonFX::setEnabled()
{
    this->disabled = false;
}
void Phoenix6Driver::RclTalonFX::setDisabled()
{
    this->disabled = true;
    this->setNeutral();
}

void Phoenix6Driver::RclTalonFX::cachePos()
{
    this->cached_position = this->motor.GetPosition().GetValue();
}
void Phoenix6Driver::RclTalonFX::applyCachedPos()
{
    this->motor.SetPosition(this->cached_position);
}
phx_::StatusCode Phoenix6Driver::RclTalonFX::applyConfig(
    units::time::second_t timeout)
{
    return this->motor.GetConfigurator().Apply(this->config, timeout);
}


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




// --- Main Class --------------------------------------------------------------

Phoenix6Driver::Phoenix6Driver() :
    Node{"phoenix6_driver"},
    relay_state_pub{this->create_publisher<Int8Msg>(
        ROBOT_TOPIC("relay_status"),
        rclcpp::SensorDataQoS{})},
    op_pub{this->create_publisher<StringMsg>(
        ROBOT_TOPIC("phx6_op_status"),
        rclcpp::SensorDataQoS{})}
{
    ParamConfig params;
    this->getParams(params);
    this->initPhx(params);
    this->initMotors(params);
    this->initCallbacks(params);

    // this->parameterizeMotorConfigs();

    // std::string arduino_device;
    // declare_param<std::string>(
    //     this,
    //     "arduino_device",
    //     arduino_device,
    //     DEFAULT_ARDUINO_DEVICE);

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "Using arduino device path : %s",
    //     arduino_device.c_str());

    // this->initSerial(arduino_device.c_str());
    // this->sendSerialPowerUp();

    RCLCPP_DEBUG(
        this->get_logger(),
        "Completed Phoenix6 Driver Node Initialization");
}

// #undef INIT_TALON_PUB_SUB

Phoenix6Driver::~Phoenix6Driver()
{
    // this->sendSerialPowerDown();
    // maybe need to pause here?
    // this->closeSerial();

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

void Phoenix6Driver::initCallbacks(const ParamConfig& params)
{
    this->watchdog_status_sub = this->create_subscription<Int32Msg>(
        ROBOT_TOPIC("watchdog_status"),
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& msg) { this->feed_watchdog_status(msg.data); });
    this->info_pub_timer = this->create_wall_timer(
        MOTOR_STATUS_PUB_DT,
        [this]() { this->pub_motor_info_cb(); });
    this->fault_pub_timer = this->create_wall_timer(
        250ms,
        [this]() { this->pub_motor_fault_cb(); });
}





// void Phoenix6Driver::sendSerialPowerDown()
// {
//     this->op_pub->publish(StringMsg{}.set__data("cache motor positions"));

//     for (size_t i = 0; i < NUM_MOTORS; i++)
//     {
//         TalonFX& m = this->motors[i].get();
//         m.SetControl(phx6::controls::NeutralOut());
//         this->cached_motor_positions[i] = m.GetPosition().GetValue();
//     }

//     this->op_pub->publish(StringMsg{}.set__data("send serial shutdown"));

//     RCLCPP_INFO(this->get_logger(), "Sending serial power down command...");
//     (void)write(this->serial_port, "0", 1);
//     this->relay_state_pub->publish(Int8Msg{}.set__data(0));
// }

// void Phoenix6Driver::sendSerialPowerUp()
// {
//     this->op_pub->publish(StringMsg{}.set__data("send serial power up"));

//     RCLCPP_INFO(this->get_logger(), "Sending serial power up command...");
//     (void)write(this->serial_port, "1", 1);
//     this->relay_state_pub->publish(Int8Msg{}.set__data(1));

//     this->configure_motors_cb(TALONFX_MAX_BOOTUP_DELAY);
//     this->neutralAll();
// }

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
            m.setDisabled();
        }
        this->is_disabled = true;
    }
    else
    {
        for (auto& m : this->motors)
        {
            m.setEnabled();
        }
        if (this->is_disabled)
        {
            this->last_enable_beg_time = system_clock::now();
            // std::cout << "Applied new last enabled state change time "
            //           << this->last_enable_beg_time.time_since_epoch().count()
            //           << std::endl;
        }
        ctre::phoenix::unmanaged::FeedEnable(std::abs(status));
        this->is_disabled = false;
        // std::cout << "watchdog set disabled to false" << std::endl;
    }
    this->relay_state_pub->publish(Int8Msg{}.set__data(1));
    // std::cout << ">> end feed watchdog status" << std::endl;
}

// void Phoenix6Driver::configure_motors_cb(units::time::second_t timeout)
// {
//     this->op_pub->publish(StringMsg{}.set__data("configure motors"));

//     auto a = system_clock::now();
//     auto s1 = this->motors[2].applyConfig();

//     auto b = system_clock::now();
//     auto s2 = this->motors[3].applyConfig();

//     auto c = system_clock::now();
//     auto s3 = this->motors[0].applyConfig();

//     auto d = system_clock::now();
//     auto s4 = this->motors[1].applyConfig();

//     auto e = system_clock::now();
//     for (size_t i = 0; i < NUM_MOTORS; i++)
//     {
//         this->motors[i].get().SetPosition(this->cached_motor_positions[i]);
//     }
//     auto f = system_clock::now();

//     std::cout << "Reconfigured Motors:\n"
//               << "\tTrencher took "
//               << std::chrono::duration<float>(b - a).count()
//               << "s and returned " << s1 << "\n\tHopper belt took "
//               << std::chrono::duration<float>(c - b).count()
//               << "s and returned " << s2 << "\n\tRight track took "
//               << std::chrono::duration<float>(d - c).count()
//               << "s and returned " << s3 << "\n\tLeft track took "
//               << std::chrono::duration<float>(e - d).count()
//               << "s and returned " << s3 << "\n\tApplying positions took "
//               << std::chrono::duration<float>(f - e).count() << "s"
//               << std::endl;

//     // RCLCPP_INFO(this->get_logger(), "Reconfigured motors.");
// }

void Phoenix6Driver::pub_motor_info_cb()
{
    //     this->op_pub->publish(StringMsg{}.set__data("pub motor info"));
    //     // std::cout << ">> begin pub motor info" << std::endl;
    //     TalonInfoMsg talon_info_msg{};
    //     talon_info_msg.header.stamp = this->get_clock()->now();

    //     bool any_disabled = false;
    //     for (size_t i = 0; i < NUM_MOTORS; i++)
    //     {
    //         TalonFX& m = this->motors[i];
    //         this->motor_pub_subs[i].get().info_pub->publish((talon_info_msg << m));
    //         any_disabled |= (!talon_info_msg.enabled && m.IsConnected(100_ms));
    //     }

    //     if (any_disabled && !this->is_disabled &&
    //         (std::chrono::system_clock::now() - this->last_enable_beg_time) >
    //             MOTOR_RESTART_DT_THRESH)
    //     {
    //         this->op_pub->publish(StringMsg{}.set__data("trigger motor restart"));
    //         // std::cout << "Restarting motors... (dt: "
    //         //           << std::chrono::duration<double>(
    //         //                  std::chrono::system_clock::now() -
    //         //                  this->last_enable_beg_time)
    //         //                  .count()
    //         //           << ")" << std::endl;

    //         this->sendSerialPowerDown();
    //         std::this_thread::sleep_for(TALONFX_POWER_CYCLE_DELAY);
    //         this->sendSerialPowerUp();
    //         this->is_disabled = true;
    //     }
    //     // std::cout << "<< end pub motor info" << std::endl;

    TalonInfoMsg buff;
    for (auto& m : this->motors)
    {
        m.pubInfo(buff);
    }
}

void Phoenix6Driver::pub_motor_fault_cb()
{
    //     this->op_pub->publish(StringMsg{}.set__data("publish faults"));
    //     // std::cout << "begin pub motor faults -- ";
    //     TalonFaultsMsg talon_faults_msg{};
    //     talon_faults_msg.header.stamp = this->get_clock()->now();

    //     for (size_t i = 0; i < NUM_MOTORS; i++)
    //     {
    //         this->motor_pub_subs[i].get().faults_pub->publish(
    //             (talon_faults_msg << this->motors[i]));
    //     }
    //     // std::cout << "end pub motor faults" << std::endl;
    TalonFaultsMsg buff;
    for (auto& m : this->motors)
    {
        m.pubFaults(buff);
    }
}



int main(int argc, char** argv)
{
    // ctre::phoenix::unmanaged::LoadPhoenix();
    // std::cout << "Loaded Phoenix 6 Unmanaged" << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Phoenix6Driver>();
    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) has started");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) shutting down...");
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
