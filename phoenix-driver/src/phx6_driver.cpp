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
using phx6::signals::NeutralModeValue;
using phx6::signals::InvertedValue;
using phx6::signals::FeedbackSensorSourceValue;


// --- Ros message serialization helpser ---------------------------------------

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
static constexpr double TFX_COMMON_KI = 0.;
// ^ 0.2 volts added for every rotation integrated error
static constexpr double TFX_COMMON_KD = 0.005;
// ^ 0.0001 volts added for every rotation per second^2 change in error [per second]
static constexpr double TFX_COMMON_KV = 0.12;
// ^ Falcon 500 is a 500kV motor, 500rpm / 1V = 8.333 rps / 1V --> 1/8.33 = 0.12 volts / rps

static constexpr double TFX_COMMON_NETRUAL_DEADBAND = 0.05;

static constexpr auto TFX_COMMON_STATOR_CURRENT_LIMIT = 20_A;
static constexpr auto TFX_COMMON_SUPPLY_CURRENT_LIMIT = 20_A;

static const TalonFXConfiguration LFET_TRACK_CONFIG =
    TalonFXConfiguration{}
        .WithSlot0(
            Slot0Configs{}
                .WithKP(TFX_COMMON_KP)
                .WithKI(TFX_COMMON_KI)
                .WithKD(TFX_COMMON_KD)
                .WithKV(TFX_COMMON_KV))
        .WithMotorOutput(
            MotorOutputConfigs{}
                .WithDutyCycleNeutralDeadband(TFX_COMMON_NETRUAL_DEADBAND)
                .WithNeutralMode(NeutralModeValue::Coast)
                .WithInverted(InvertedValue::CounterClockwise_Positive))
        .WithFeedback(
            FeedbackConfigs{}.WithFeedbackSensorSource(
                FeedbackSensorSourceValue::RotorSensor))
        .WithCurrentLimits(
            CurrentLimitsConfigs{}
                .WithStatorCurrentLimit(TFX_COMMON_STATOR_CURRENT_LIMIT)
                .WithStatorCurrentLimitEnable(true)
                .WithSupplyCurrentLimit(TFX_COMMON_SUPPLY_CURRENT_LIMIT)
                .WithSupplyCurrentLimitEnable(true));

static const TalonFXConfiguration RIGHT_TRACK_CONFIG =
    TalonFXConfiguration{}
        .WithSlot0(
            Slot0Configs{}
                .WithKP(TFX_COMMON_KP)
                .WithKI(TFX_COMMON_KI)
                .WithKD(TFX_COMMON_KD)
                .WithKV(TFX_COMMON_KV))
        .WithMotorOutput(
            MotorOutputConfigs{}
                .WithDutyCycleNeutralDeadband(TFX_COMMON_NETRUAL_DEADBAND)
                .WithNeutralMode(NeutralModeValue::Coast)
                .WithInverted(InvertedValue::Clockwise_Positive))
        .WithFeedback(
            FeedbackConfigs{}.WithFeedbackSensorSource(
                FeedbackSensorSourceValue::RotorSensor))
        .WithCurrentLimits(
            CurrentLimitsConfigs{}
                .WithStatorCurrentLimit(TFX_COMMON_STATOR_CURRENT_LIMIT)
                .WithStatorCurrentLimitEnable(true)
                .WithSupplyCurrentLimit(TFX_COMMON_SUPPLY_CURRENT_LIMIT)
                .WithSupplyCurrentLimitEnable(true));

static const TalonFXConfiguration TRENCHER_CONFIG =
    TalonFXConfiguration{}
        .WithSlot0(
            Slot0Configs{}
                .WithKP(TFX_COMMON_KP)
                .WithKI(TFX_COMMON_KI)
                .WithKD(TFX_COMMON_KD)
                .WithKV(TFX_COMMON_KV))
        .WithMotorOutput(
            MotorOutputConfigs{}
                .WithDutyCycleNeutralDeadband(TFX_COMMON_NETRUAL_DEADBAND)
                .WithNeutralMode(NeutralModeValue::Coast)
                .WithInverted(InvertedValue::Clockwise_Positive))
        // ^ trencher positive direction should result in digging
        .WithFeedback(
            FeedbackConfigs{}.WithFeedbackSensorSource(
                FeedbackSensorSourceValue::RotorSensor))
        .WithCurrentLimits(
            CurrentLimitsConfigs{}
                .WithStatorCurrentLimit(TFX_COMMON_STATOR_CURRENT_LIMIT)
                .WithStatorCurrentLimitEnable(true)
                .WithSupplyCurrentLimit(TFX_COMMON_SUPPLY_CURRENT_LIMIT)
                .WithSupplyCurrentLimitEnable(true));

static const TalonFXConfiguration HOPPER_BELT_CONFIG =
    TalonFXConfiguration{}
        .WithSlot0(
            Slot0Configs{}
                .WithKP(TFX_COMMON_KP)
                .WithKI(TFX_COMMON_KI)
                .WithKD(TFX_COMMON_KD)
                .WithKV(TFX_COMMON_KV))
        .WithMotorOutput(
            MotorOutputConfigs{}
                .WithDutyCycleNeutralDeadband(TFX_COMMON_NETRUAL_DEADBAND)
                .WithNeutralMode(NeutralModeValue::Coast)
                .WithInverted(InvertedValue::Clockwise_Positive))
        // ^ hopper belt positive direction should result in dumping
        .WithFeedback(
            FeedbackConfigs{}.WithFeedbackSensorSource(
                FeedbackSensorSourceValue::RotorSensor));
        // .WithCurrentLimits(
        //     CurrentLimitsConfigs{}
        //         .WithStatorCurrentLimit(TFX_COMMON_STATOR_CURRENT_LIMIT)
        //         .WithStatorCurrentLimitEnable(false)
        //         .WithSupplyCurrentLimit(TFX_COMMON_SUPPLY_CURRENT_LIMIT)
        //         .WithSupplyCurrentLimitEnable(false));


// --- Program defaults --------------------------------------------------------

#define DEFAULT_ARDUINO_DEVICE "/dev/ttyACM0"
#define CAN_INTERFACE          "can0"
#define RIGHT_TRACK_CANID      0
#define LEFT_TRACK_CANID       1
#define TRENCHER_CANID         2
#define HOPPER_BELT_CANID      3
#define NUM_MOTORS             4

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_SUB_QOS    10
#define ROBOT_CTRL_SUB_QOS    10

#define MOTOR_STATUS_PUB_DT       50ms
#define TALONFX_BOOTUP_DELAY      3s
#define TALONFX_POWER_CYCLE_DELAY 1s

#define DISABLE_WATCHDOG 0


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

    void initSerial(const char*);
    void sendSerialPowerDown();
    void sendSerialPowerUp();
    inline void closeSerial() { close(this->serial_port); }

    void feed_watchdog_status(int32_t status);
    // Function to setup motors for the robot
    void configure_motors_cb();
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

    TalonFXPubSub track_right_pub_sub;
    TalonFXPubSub track_left_pub_sub;
    TalonFXPubSub trencher_pub_sub;
    TalonFXPubSub hopper_belt_pub_sub;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr watchdog_status_sub;

    rclcpp::TimerBase::SharedPtr info_pub_timer;
    rclcpp::TimerBase::SharedPtr fault_pub_timer;

    std::array<std::reference_wrapper<TalonFX>, NUM_MOTORS> motors{
        {track_right, track_left, trencher, hopper_belt}
    };
    std::array<std::reference_wrapper<TalonFXPubSub>, NUM_MOTORS> motor_pub_subs{
        {track_right_pub_sub,
         track_left_pub_sub, trencher_pub_sub,
         hopper_belt_pub_sub}
    };

    std::array<units::angle::turn_t, NUM_MOTORS> cached_motor_positions{
        {0_tr, 0_tr, 0_tr, 0_tr}
    };

    int serial_port;
    bool is_disabled = true;
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
    std::string arduino_device;
    this->declare_parameter("arduino_device", DEFAULT_ARDUINO_DEVICE);
    this->get_parameter("arduino_device", arduino_device);

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
        this->cached_motor_positions[i] =
            this->motors[i].get().GetPosition().GetValue();
    }

    RCLCPP_INFO(this->get_logger(), "Sending serial power down command...");
    (void)write(this->serial_port, "0", 1);
}

void Phoenix6Driver::sendSerialPowerUp()
{
    RCLCPP_INFO(this->get_logger(), "Sending serial power up command...");
    (void)write(this->serial_port, "1", 1);

    std::this_thread::sleep_for(TALONFX_BOOTUP_DELAY);

    this->configure_motors_cb();
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
        ctre::phoenix::unmanaged::FeedEnable(std::abs(status));
        this->is_disabled = false;
    }
}

void Phoenix6Driver::configure_motors_cb()
{
    trencher.GetConfigurator().Apply(TRENCHER_CONFIG);
    trencher.ClearStickyFaults();

    hopper_belt.GetConfigurator().Apply(HOPPER_BELT_CONFIG);
    hopper_belt.ClearStickyFaults();

    track_right.GetConfigurator().Apply(RIGHT_TRACK_CONFIG);
    track_right.ClearStickyFaults();

    track_left.GetConfigurator().Apply(LFET_TRACK_CONFIG);
    track_left.ClearStickyFaults();

    for(size_t i = 0; i < NUM_MOTORS; i++)
    {
        this->motors[i].get().SetPosition(this->cached_motor_positions[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Reconfigured motors.");
}

void Phoenix6Driver::pub_motor_info_cb()
{
    TalonInfo talon_info_msg{};
    talon_info_msg.header.stamp = this->get_clock()->now();

    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        this->motor_pub_subs[i].get().info_pub->publish(
            (talon_info_msg << this->motors[i]));
    }
}

void Phoenix6Driver::pub_motor_fault_cb()
{
    TalonFaults talon_faults_msg{};
    talon_faults_msg.header.stamp = this->get_clock()->now();

    constexpr uint32_t SUPPLY_CURRENT_FAULT_MASK = 0x100;
    constexpr uint32_t STATOR_CURRENT_FAULT_MASK = 0x200;
    constexpr uint32_t OVERVOLTAGE_FAULT_MASK = 0x40000;
    constexpr uint32_t BRIDGE_BROWNOUT_FAULT_MASK = 0x800000;

    constexpr uint32_t FAULTS_IGNORE_MASK =
        (SUPPLY_CURRENT_FAULT_MASK | STATOR_CURRENT_FAULT_MASK |
         OVERVOLTAGE_FAULT_MASK | BRIDGE_BROWNOUT_FAULT_MASK);

    bool any_faults = false;
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        this->motor_pub_subs[i].get().faults_pub->publish(
            (talon_faults_msg << this->motors[i]));
        any_faults |= (talon_faults_msg.faults & (~FAULTS_IGNORE_MASK));
    }

    if (any_faults)
    {
        this->sendSerialPowerDown();
        std::this_thread::sleep_for(TALONFX_POWER_CYCLE_DELAY);
        this->sendSerialPowerUp();
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
                // torque/current control required phoenix pro
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
