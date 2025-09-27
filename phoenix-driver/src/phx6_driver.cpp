#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>

#define Phoenix_No_WPI  // remove WPI dependencies
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include <phoenix_ros_driver/msg/talon_ctrl.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>
#include <phoenix_ros_driver/msg/talon_faults.hpp>


#define phx6 ctre::phoenix6
using namespace std::chrono_literals;

using phoenix_ros_driver::msg::TalonCtrl;
using phoenix_ros_driver::msg::TalonInfo;
using phoenix_ros_driver::msg::TalonFaults;

using phx6::hardware::TalonFX;
using phx6::configs::TalonFXConfiguration;
using phx6::configs::Slot0Configs;
using phx6::configs::MotorOutputConfigs;
using phx6::configs::FeedbackConfigs;
using phx6::configs::CurrentLimitsConfigs;
using phx6::configs::VoltageConfigs;
using phx6::signals::NeutralModeValue;
using phx6::signals::InvertedValue;
using phx6::signals::FeedbackSensorSourceValue;


// --- util --------------------------------------------------------------------

template<typename T>
inline void declare_param(
    rclcpp::Node* node,
    const std::string param_name,
    T& param,
    const T& default_value)
{
    node->declare_parameter(param_name, default_value);
    node->get_parameter(param_name, param);
}


// --- Ros message serialization helpers ---------------------------------------

TalonInfo& operator<<(TalonInfo& info, TalonFX& m)
{
    info.position = m.GetPosition().GetValueAsDouble();
    info.velocity = m.GetVelocity().GetValueAsDouble();
    info.acceleration = m.GetAcceleration().GetValueAsDouble();

    info.device_temp = m.GetDeviceTemp().GetValueAsDouble();
    info.processor_temp = m.GetProcessorTemp().GetValueAsDouble();
    info.bus_voltage = m.GetSupplyVoltage().GetValueAsDouble();
    info.supply_current = m.GetSupplyCurrent().GetValueAsDouble();

    info.output_percent = m.GetDutyCycle().GetValueAsDouble();
    info.output_voltage = m.GetMotorVoltage().GetValueAsDouble();
    info.output_current = m.GetStatorCurrent().GetValueAsDouble();

    info.motor_state =
        static_cast<uint8_t>(m.GetMotorOutputStatus().GetValue().value);
    info.bridge_mode =
        static_cast<uint8_t>(m.GetBridgeOutput().GetValue().value);
    info.control_mode =
        static_cast<uint8_t>(m.GetControlMode().GetValue().value);
    info.enabled = static_cast<bool>(m.GetDeviceEnable().GetValue().value);

    return info;
}

TalonFaults& operator<<(TalonFaults& faults, TalonFX& m)
{
    faults.faults = m.GetFaultField().GetValue();
    faults.sticky_faults = m.GetStickyFaultField().GetValue();

    faults.hardware_fault = m.GetFault_Hardware().GetValue();
    faults.proc_temp_fault = m.GetFault_ProcTemp().GetValue();
    faults.device_temp_fault = m.GetFault_DeviceTemp().GetValue();
    faults.undervoltage_fault = m.GetFault_Undervoltage().GetValue();
    faults.boot_fault = m.GetFault_BootDuringEnable().GetValue();
    faults.unliscensed_fault = m.GetFault_UnlicensedFeatureInUse().GetValue();
    faults.bridge_brownout_fault = m.GetFault_BridgeBrownout().GetValue();
    faults.overvoltage_fault = m.GetFault_OverSupplyV().GetValue();
    faults.unstable_voltage_fault = m.GetFault_UnstableSupplyV().GetValue();
    faults.stator_current_limit_fault = m.GetFault_StatorCurrLimit().GetValue();
    faults.supply_current_limit_fault = m.GetFault_SupplyCurrLimit().GetValue();
    faults.static_brake_disabled_fault =
        m.GetFault_StaticBrakeDisabled().GetValue();

    faults.sticky_hardware_fault = m.GetStickyFault_Hardware().GetValue();
    faults.sticky_proc_temp_fault = m.GetStickyFault_ProcTemp().GetValue();
    faults.sticky_device_temp_fault = m.GetStickyFault_DeviceTemp().GetValue();
    faults.sticky_undervoltage_fault =
        m.GetStickyFault_Undervoltage().GetValue();
    faults.sticky_boot_fault = m.GetStickyFault_BootDuringEnable().GetValue();
    faults.sticky_unliscensed_fault =
        m.GetStickyFault_UnlicensedFeatureInUse().GetValue();
    faults.sticky_bridge_brownout_fault =
        m.GetStickyFault_BridgeBrownout().GetValue();
    faults.sticky_overvoltage_fault = m.GetStickyFault_OverSupplyV().GetValue();
    faults.sticky_unstable_voltage_fault =
        m.GetStickyFault_UnstableSupplyV().GetValue();
    faults.sticky_stator_current_limit_fault =
        m.GetStickyFault_StatorCurrLimit().GetValue();
    faults.sticky_supply_current_limit_fault =
        m.GetStickyFault_SupplyCurrLimit().GetValue();
    faults.sticky_static_brake_disabled_fault =
        m.GetStickyFault_StaticBrakeDisabled().GetValue();

    return faults;
}


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

inline TalonFXConfiguration buildTalonConfig(
    double kP,
    double kI,
    double kD,
    double kV,
    double neutral_deadband,
    int neutral_mode,
    int invert_mode,
    double stator_current_limit = 0.,
    double supply_current_limit = 0.,
    double voltage_limit = 0.)
{
    return TalonFXConfiguration{}
        .WithSlot0(Slot0Configs{}.WithKP(kP).WithKI(kI).WithKD(kD).WithKV(kV))
        .WithMotorOutput(
            MotorOutputConfigs{}
                .WithDutyCycleNeutralDeadband(neutral_deadband)
                .WithNeutralMode(neutral_mode)
                .WithInverted(invert_mode))
        .WithFeedback(
            FeedbackConfigs{}.WithFeedbackSensorSource(
                FeedbackSensorSourceValue::RotorSensor))
        .WithCurrentLimits(
            CurrentLimitsConfigs{}
                .WithStatorCurrentLimit(
                    units::current::ampere_t{stator_current_limit})
                .WithStatorCurrentLimitEnable(stator_current_limit > 0.)
                .WithSupplyCurrentLimit(
                    units::current::ampere_t{supply_current_limit})
                .WithSupplyCurrentLimitEnable(supply_current_limit >= 0.))
        .WithVoltage(
            (voltage_limit >= 0.)
                ? VoltageConfigs{}
                      .WithPeakForwardVoltage(
                          units::voltage::volt_t{voltage_limit})
                      .WithPeakReverseVoltage(
                          units::voltage::volt_t{-voltage_limit})
                : VoltageConfigs{});
}


// --- Program defaults --------------------------------------------------------

#define DEFAULT_ARDUINO_DEVICE "/dev/ttyACM0"
#define CAN_INTERFACE          "can_phx6"
#define RIGHT_TRACK_CANID      0
#define LEFT_TRACK_CANID       1
#define TRENCHER_CANID         2
#define HOPPER_BELT_CANID      3
#define NUM_MOTORS             4

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_SUB_QOS    10
#define ROBOT_CTRL_SUB_QOS    10

#define MOTOR_STATUS_PUB_DT       50ms
#define TALONFX_MAX_BOOTUP_DELAY  3s
#define TALONFX_POWER_CYCLE_DELAY 0.5s
#define MOTOR_RESTART_DT_THRESH   1.5s

#define DISABLE_WATCHDOG 0

#define SUPPLY_CURRENT_FAULT_MASK  0x100
#define STATOR_CURRENT_FAULT_MASK  0x200
#define OVERVOLTAGE_FAULT_MASK     0x40000
#define BRIDGE_BROWNOUT_FAULT_MASK 0x800000

#define FAULTS_IGNORE_MASK                                   \
    (SUPPLY_CURRENT_FAULT_MASK | STATOR_CURRENT_FAULT_MASK | \
     OVERVOLTAGE_FAULT_MASK | BRIDGE_BROWNOUT_FAULT_MASK)


// --- Driver node -------------------------------------------------------------

class Phoenix6Driver : public rclcpp::Node
{
public:
    Phoenix6Driver();
    ~Phoenix6Driver();

private:
    inline void neutralAll()
    {
        for (TalonFX& m : this->motors)
        {
            m.SetControl(phx6::controls::NeutralOut{});
        }
    }

    void parameterizeMotorConfigs();

    void initSerial(const char*);
    void sendSerialPowerDown();
    void sendSerialPowerUp();
    inline void closeSerial() { close(this->serial_port); }

    void feed_watchdog_status(int32_t status);
    // Function to setup motors for the robot
    void configure_motors_cb(units::time::second_t timeout = 0.1_s);
    // Periodic function for motor information updates
    void pub_motor_info_cb();
    void pub_motor_fault_cb();
    // Function to execute control commands on a motor
    void execute_ctrl_cb(TalonFX& motor, const TalonCtrl& msg);

private:
    struct TalonFXPubSub
    {
        rclcpp::Publisher<TalonInfo>::SharedPtr info_pub;
        rclcpp::Publisher<TalonFaults>::SharedPtr faults_pub;

        rclcpp::Subscription<TalonCtrl>::SharedPtr ctrl_sub;
    };

private:
    TalonFX track_right;
    TalonFX track_left;
    TalonFX trencher;
    TalonFX hopper_belt;

    TalonFXConfiguration track_right_config;
    TalonFXConfiguration track_left_config;
    TalonFXConfiguration trencher_config;
    TalonFXConfiguration hopper_belt_config;

    TalonFXPubSub track_right_pub_sub;
    TalonFXPubSub track_left_pub_sub;
    TalonFXPubSub trencher_pub_sub;
    TalonFXPubSub hopper_belt_pub_sub;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr relay_state_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr watchdog_status_sub;

    rclcpp::TimerBase::SharedPtr info_pub_timer;
    rclcpp::TimerBase::SharedPtr fault_pub_timer;

    std::array<std::reference_wrapper<TalonFX>, NUM_MOTORS> motors{
        {track_right, track_left, trencher, hopper_belt}
    };
    std::array<std::reference_wrapper<TalonFXPubSub>, NUM_MOTORS>
        motor_pub_subs{
            {track_right_pub_sub,
             track_left_pub_sub, trencher_pub_sub,
             hopper_belt_pub_sub}
    };

    std::array<units::angle::turn_t, NUM_MOTORS> cached_motor_positions{
        {0_tr, 0_tr, 0_tr, 0_tr}
    };

    int serial_port;
    bool is_disabled = true;
    std::chrono::system_clock::time_point last_enable_beg_time;
};



// clang-format off
#define INIT_TALON_PUB_SUB(device)                         \
    device##_pub_sub                                       \
    {                                                      \
        this->create_publisher<TalonInfo>(                 \
            ROBOT_TOPIC(#device "/info"),                  \
            rclcpp::SensorDataQoS{}),                      \
        this->create_publisher<TalonFaults>(               \
            ROBOT_TOPIC(#device "/faults"),                \
            rclcpp::SensorDataQoS{}),                      \
        this->create_subscription<TalonCtrl>(              \
            ROBOT_TOPIC(#device "/ctrl"),                  \
            TALON_CTRL_SUB_QOS,                            \
            [this](const TalonCtrl& msg)                   \
            { this->execute_ctrl_cb(this->device, msg); }) \
}
// clang-format on

Phoenix6Driver::Phoenix6Driver() :
    Node{"phoenix6_driver"},

    track_right{RIGHT_TRACK_CANID, CAN_INTERFACE},
    track_left{LEFT_TRACK_CANID, CAN_INTERFACE},
    trencher{TRENCHER_CANID, CAN_INTERFACE},
    hopper_belt{HOPPER_BELT_CANID, CAN_INTERFACE},

    INIT_TALON_PUB_SUB(track_right),
    INIT_TALON_PUB_SUB(track_left),
    INIT_TALON_PUB_SUB(trencher),
    INIT_TALON_PUB_SUB(hopper_belt),

    relay_state_pub{this->create_publisher<std_msgs::msg::Int8>(
        ROBOT_TOPIC("relay_status"),
        rclcpp::SensorDataQoS{})},
#if !DISABLE_WATCHDOG
    watchdog_status_sub{this->create_subscription<std_msgs::msg::Int32>(
        ROBOT_TOPIC("watchdog_status"),
        rclcpp::SensorDataQoS{},
        [this](const std_msgs::msg::Int32& msg)
        { this->feed_watchdog_status(msg.data); })},
#endif
    info_pub_timer{this->create_wall_timer(
        MOTOR_STATUS_PUB_DT,
        [this]()
        {
            this->pub_motor_info_cb();
#if DISABLE_WATCHDOG
            this->feed_watchdog_status(250);
#endif
        })},
    fault_pub_timer{this->create_wall_timer(
        250ms,
        [this]() { this->pub_motor_fault_cb(); })}
{
    this->parameterizeMotorConfigs();

    std::string arduino_device;
    declare_param<std::string>(
        this,
        "arduino_device",
        arduino_device,
        DEFAULT_ARDUINO_DEVICE);

    RCLCPP_INFO(
        this->get_logger(),
        "Using arduino device path : %s",
        arduino_device.c_str());

    this->initSerial(arduino_device.c_str());
    this->sendSerialPowerUp();

    RCLCPP_DEBUG(
        this->get_logger(),
        "Completed Phoenix6 Driver Node Initialization");
}

#undef INIT_TALON_PUB_SUB

Phoenix6Driver::~Phoenix6Driver()
{
    this->sendSerialPowerDown();
    // maybe need to pause here?
    this->closeSerial();
}


void Phoenix6Driver::parameterizeMotorConfigs()
{
    double kP;
    double kI;
    double kD;
    double kV;
    double neutral_deadband;
    double stator_current_limit;
    double supply_current_limit;
    double voltage_limit;
    bool neutral_brake = false;

    declare_param(this, "common.kP", kP, TFX_COMMON_KP);
    declare_param(this, "common.kI", kI, TFX_COMMON_KI);
    declare_param(this, "common.kD", kD, TFX_COMMON_KD);
    declare_param(this, "common.kV", kV, TFX_COMMON_KV);
    declare_param(
        this,
        "common.neutral_deadband",
        neutral_deadband,
        TFX_COMMON_NEUTRAL_DEADBAND);
    declare_param(this, "common.neutral_brake", neutral_brake, false);
    declare_param(
        this,
        "common.stator_current_limit",
        stator_current_limit,
        TFX_COMMON_STATOR_CURRENT_LIMIT);
    declare_param(
        this,
        "common.supply_current_limit",
        supply_current_limit,
        TFX_COMMON_SUPPLY_CURRENT_LIMIT);
    declare_param(
        this,
        "common.voltage_limit",
        voltage_limit,
        TFX_COMMON_PEAK_VOLTAGE);

    // std::cout << kP << ", " << kI << ", " << kD << ", " << kV << ", "
    //           << neutral_deadband << ", " << neutral_brake << ", "
    //           << stator_current_limit << ", " << supply_current_limit << ", "
    //           << voltage_limit << std::endl;

    this->track_left_config = buildTalonConfig(
        kP,
        kI,
        kD,
        kV,
        neutral_deadband,
        neutral_brake ? NeutralModeValue::Brake : NeutralModeValue::Coast,
        InvertedValue::Clockwise_Positive,
        stator_current_limit,
        supply_current_limit,
        voltage_limit);
    this->track_right_config = buildTalonConfig(
        kP,
        kI,
        kD,
        kV,
        neutral_deadband,
        neutral_brake ? NeutralModeValue::Brake : NeutralModeValue::Coast,
        InvertedValue::CounterClockwise_Positive,
        stator_current_limit,
        supply_current_limit,
        voltage_limit);
    this->trencher_config = buildTalonConfig(
        kP,
        kI,
        kD,
        kV,
        neutral_deadband,
        neutral_brake ? NeutralModeValue::Brake : NeutralModeValue::Coast,
        InvertedValue::Clockwise_Positive,
        stator_current_limit,
        supply_current_limit,
        voltage_limit);
    this->hopper_belt_config = buildTalonConfig(
        kP,
        kI,
        kD,
        kV,
        neutral_deadband,
        neutral_brake ? NeutralModeValue::Brake : NeutralModeValue::Coast,
        InvertedValue::Clockwise_Positive,
        stator_current_limit,
        supply_current_limit,
        voltage_limit);
}

void Phoenix6Driver::initSerial(const char* port)
{
    this->serial_port = open(port, O_RDWR | O_NOCTTY | O_SYNC);

    if (this->serial_port < 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to open serial port %s for motor resets!",
            port);
        return;
    }

    // Configure port
    termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(this->serial_port, &tty) != 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error configuring (tcgetattr) serial port %s for motor resets!",
            port);
        return;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag |=
        (CLOCAL | CREAD);  // enable receiver, ignore modem ctrl lines
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 bits
    tty.c_cflag &= ~PARENB;   // no parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;  // no flow control

    tty.c_lflag = 0;      // no signaling chars, no echo
    tty.c_oflag = 0;      // no remapping, no delays
    tty.c_cc[VMIN] = 1;   // read doesn't return until at least 1 char
    tty.c_cc[VTIME] = 5;  // 0.5 seconds read timeout

    if (tcsetattr(this->serial_port, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error configuring (tcsetattr) serial port %s for motor resets!",
            port);
        return;
    }
}

void Phoenix6Driver::sendSerialPowerDown()
{
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        TalonFX& m = this->motors[i].get();
        m.SetControl(phx6::controls::NeutralOut());
        this->cached_motor_positions[i] = m.GetPosition().GetValue();
    }

    RCLCPP_INFO(this->get_logger(), "Sending serial power down command...");
    (void)write(this->serial_port, "0", 1);
    this->relay_state_pub->publish(std_msgs::msg::Int8{}.set__data(0));
}

void Phoenix6Driver::sendSerialPowerUp()
{
    RCLCPP_INFO(this->get_logger(), "Sending serial power up command...");
    (void)write(this->serial_port, "1", 1);
    this->relay_state_pub->publish(std_msgs::msg::Int8{}.set__data(1));

    this->configure_motors_cb(TALONFX_MAX_BOOTUP_DELAY);
    this->neutralAll();
}

void Phoenix6Driver::feed_watchdog_status(int32_t status)
{
    /* Watchdog feed decoding:
     * POSTIVE feed time --> enabled
     * ZERO feed time --> disabled
     * NEGATIVE feed time --> autonomous */
    if (!status)
    {
        this->neutralAll();
        this->is_disabled = true;
    }
    else
    {
        if (this->is_disabled)
        {
            this->last_enable_beg_time = std::chrono::system_clock::now();
            // std::cout << "Applied new last enabled state change time "
            //           << this->last_enable_beg_time.time_since_epoch().count()
            //           << std::endl;
        }
        ctre::phoenix::unmanaged::FeedEnable(std::abs(status));
        this->is_disabled = false;
        // std::cout << "watchdog set disabled to false" << std::endl;
    }
}

void Phoenix6Driver::configure_motors_cb(units::time::second_t timeout)
{
    trencher.GetConfigurator().Apply(this->trencher_config, timeout);
    trencher.ClearStickyFaults();

    hopper_belt.GetConfigurator().Apply(this->hopper_belt_config, timeout);
    hopper_belt.ClearStickyFaults();

    track_right.GetConfigurator().Apply(this->track_right_config, timeout);
    track_right.ClearStickyFaults();

    track_left.GetConfigurator().Apply(this->track_left_config, timeout);
    track_left.ClearStickyFaults();

    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        this->motors[i].get().SetPosition(this->cached_motor_positions[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Reconfigured motors.");
}

void Phoenix6Driver::pub_motor_info_cb()
{
    TalonInfo talon_info_msg{};
    talon_info_msg.header.stamp = this->get_clock()->now();

    bool any_disabled = false;
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        this->motor_pub_subs[i].get().info_pub->publish(
            (talon_info_msg << this->motors[i]));
        any_disabled |= !talon_info_msg.enabled;
    }

    if (any_disabled && !this->is_disabled &&
        (std::chrono::system_clock::now() - this->last_enable_beg_time) >
            MOTOR_RESTART_DT_THRESH)
    {
        // std::cout << "Restarting motors... (dt: "
        //           << std::chrono::duration<double>(
        //                  std::chrono::system_clock::now() -
        //                  this->last_enable_beg_time)
        //                  .count()
        //           << ")" << std::endl;

        this->sendSerialPowerDown();
        std::this_thread::sleep_for(TALONFX_POWER_CYCLE_DELAY);
        this->sendSerialPowerUp();
        this->is_disabled = true;
    }
}

void Phoenix6Driver::pub_motor_fault_cb()
{
    TalonFaults talon_faults_msg{};
    talon_faults_msg.header.stamp = this->get_clock()->now();

    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        this->motor_pub_subs[i].get().faults_pub->publish(
            (talon_faults_msg << this->motors[i]));
    }
}

void Phoenix6Driver::execute_ctrl_cb(TalonFX& motor, const TalonCtrl& msg)
{
    if (this->is_disabled)
    {
        motor.SetControl(phx6::controls::NeutralOut());
    }
    else
    {
        switch (msg.mode)
        {
            case TalonCtrl::PERCENT_OUTPUT:
            {
                motor.SetControl(phx6::controls::DutyCycleOut{msg.value});
                break;
            }
            case TalonCtrl::POSITION:
            {
                motor.SetControl(
                    phx6::controls::PositionVoltage{
                        units::angle::turn_t{msg.value}});
                break;
            }
            case TalonCtrl::VELOCITY:
            {
                motor.SetControl(
                    phx6::controls::VelocityVoltage{
                        units::angular_velocity::turns_per_second_t{
                            msg.value}});
                break;
            }
            case TalonCtrl::CURRENT:
            {
                // torque/current control requires phoenix pro
                break;
            }
            case TalonCtrl::VOLTAGE:
            {
                motor.SetControl(
                    phx6::controls::VoltageOut{
                        units::voltage::volt_t{msg.value}});
                break;
            }
            case TalonCtrl::DISABLED:
            {
                motor.SetControl(phx6::controls::NeutralOut());
                break;
            }
            case TalonCtrl::FOLLOWER:
            case TalonCtrl::MOTION_MAGIC:
            case TalonCtrl::MOTION_PROFILE:
            case TalonCtrl::MOTION_PROFILE_ARC:
            {
                break;  // idk
            }
            case TalonCtrl::MUSIC_TONE:
            {
                motor.SetControl(
                    phx6::controls::MusicTone{
                        units::frequency::hertz_t{msg.value}});
                break;
            }
        }
    }
}



int main(int argc, char** argv)
{
    ctre::phoenix::unmanaged::LoadPhoenix();
    std::cout << "Loaded Phoenix 6 Unmanaged" << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Phoenix6Driver>();
    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) has started");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) shutting down...");
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
