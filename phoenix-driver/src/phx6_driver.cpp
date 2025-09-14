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

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include <phoenix_ros_driver/msg/talon_ctrl.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>
#include <phoenix_ros_driver/msg/talon_faults.hpp>


#define HARD_ENABLE 0

#define phx6 ctre::phoenix6
using namespace std::chrono_literals;

using phoenix_ros_driver::msg::TalonCtrl;
using phoenix_ros_driver::msg::TalonInfo;
using phoenix_ros_driver::msg::TalonFaults;
using phx6::hardware::TalonFX;


namespace TalonStaticConfig
{
    static constexpr char const* INTERFACE = "can0";

    static constexpr double kP = 0.11;
    static constexpr double kI = 0.5;
    static constexpr double kD = 0.0001;
    static constexpr double kV = 0.12;

    static constexpr double DUTY_CYCLE_DEADBAND = 0.05;
    static constexpr int NEUTRAL_MODE = ctre::phoenix6::signals::NeutralModeValue::Coast;

    static constexpr phx6::configs::Slot0Configs
        SLOT0_CONFIG =
            phx6::configs::Slot0Configs{}
                .WithKP(kP)
                .WithKI(kI)
                .WithKD(kD)
                .WithKV(kV);

    static constexpr phx6::configs::MotorOutputConfigs
        MOTOR_OUTPUT_CONFIG =
            phx6::configs::MotorOutputConfigs{}
                .WithDutyCycleNeutralDeadband(DUTY_CYCLE_DEADBAND)  // <-- deadband which applies neutral mode!
                .WithNeutralMode(NEUTRAL_MODE);

    static constexpr phx6::configs::FeedbackConfigs
        FEEDBACK_CONFIGS =
            phx6::configs::FeedbackConfigs{}
                .WithFeedbackSensorSource(phx6::signals::FeedbackSensorSourceValue::RotorSensor);

    static const phx6::configs::CurrentLimitsConfigs    // TODO -- analysis
        CURRENT_LIMIT_CONFIG =
            phx6::configs::CurrentLimitsConfigs{}
                .WithStatorCurrentLimitEnable(true)
                .WithStatorCurrentLimit(100_A)
                .WithSupplyCurrentLowerLimit(50_A)
                .WithSupplyCurrentLowerTime(5_s);
}
namespace TalonRuntimeConfig
{
    static constexpr auto MOTOR_VELOCITY_SETPOINT_ACC = 5_tr_per_s_sq;
};

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_SUB_QOS 10
#define ROBOT_CTRL_SUB_QOS 10

#define MOTOR_STATUS_PUB_DT 50ms
#define TALONFX_BOOTUP_DELAY 3s
#define TALONFX_POWER_CYCLE_DELAY 1s


class Phoenix6Driver :
    public rclcpp::Node
{
protected:
    struct TalonFXPubSub
    {
        rclcpp::Publisher<TalonInfo>::SharedPtr info_pub;
        rclcpp::Publisher<TalonFaults>::SharedPtr faults_pub;

        rclcpp::Subscription<TalonCtrl>::SharedPtr ctrl_sub;
    };

public:
    #define INIT_TALON_PUB_SUB(device) \
        device##_pub_sub \
        { \
            this->create_publisher<TalonInfo>(ROBOT_TOPIC(#device"/info"), rclcpp::SensorDataQoS{}), \
            this->create_publisher<TalonFaults>(ROBOT_TOPIC(#device"/faults"), rclcpp::SensorDataQoS{}), \
            this->create_subscription<TalonCtrl>( \
                ROBOT_TOPIC(#device"/ctrl"), \
                TALON_CTRL_SUB_QOS, \
                [this](const TalonCtrl& msg){ this->execute_ctrl_cb(this->device, msg); } ) \
        }

    Phoenix6Driver() :
        Node{ "phoenix6_driver" },

        track_right{ 0, TalonStaticConfig::INTERFACE },
        track_left{ 1, TalonStaticConfig::INTERFACE },
        trencher{ 2, TalonStaticConfig::INTERFACE },
        hopper_belt{ 3, TalonStaticConfig::INTERFACE },

        INIT_TALON_PUB_SUB(track_right),
        INIT_TALON_PUB_SUB(track_left),
        INIT_TALON_PUB_SUB(trencher),
        INIT_TALON_PUB_SUB(hopper_belt),

        watchdog_status_sub
        {
            this->create_subscription<std_msgs::msg::Int32>(
                ROBOT_TOPIC("watchdog_status"),
                rclcpp::SensorDataQoS{},
                [this](const std_msgs::msg::Int32& msg){ this->feed_watchdog_status(msg.data); } )
        },
        info_pub_timer
        {
            this->create_wall_timer(
                MOTOR_STATUS_PUB_DT,
                [this](){
                    this->pub_motor_info_cb();
                #if HARD_ENABLE
                    this->feed_watchdog_status(250);
                #endif
                }) },
        fault_pub_timer{ this->create_wall_timer(250ms, [this](){ this->pub_motor_fault_cb(); }) }
    {
        std::string arduino_device;
        this->declare_parameter("arduino_device", "/dev/ttyACM0");
        this->get_parameter("arduino_device", arduino_device);

        RCLCPP_INFO(this->get_logger(), "Using arduino device path : %s", arduino_device.c_str());

        this->initSerial(arduino_device.c_str());
        this->sendSerialPowerUp();

        RCLCPP_DEBUG(this->get_logger(), "Completed Phoenix6 Driver Node Initialization");
    }

    inline ~Phoenix6Driver()
    {
        this->sendSerialPowerDown();
        // maybe need to pause here?
        this->closeSerial();
    }

    #undef INIT_TALON_PUB_SUB

private:
    inline void neutralAll()
    {
        for(TalonFX& m : this->motors)
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
    TalonFX
        track_right,
        track_left,
        trencher,
        hopper_belt;

    TalonFXPubSub
        track_right_pub_sub,
        track_left_pub_sub,
        trencher_pub_sub,
        hopper_belt_pub_sub;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
        watchdog_status_sub;
    rclcpp::TimerBase::SharedPtr
        info_pub_timer,
        fault_pub_timer;

    std::array<std::reference_wrapper<TalonFX>, 4>
        motors{ { track_right, track_left, trencher, hopper_belt } };
    std::array<std::reference_wrapper<TalonFXPubSub>, 4>
        motor_pub_subs{ { track_right_pub_sub, track_left_pub_sub, trencher_pub_sub, hopper_belt_pub_sub } };

    int serial_port;
    bool is_disabled = true;

};


TalonInfo& operator<<(TalonInfo& info, TalonFX& m)
{
    info.position       = m.GetPosition().GetValueAsDouble();
    info.velocity       = m.GetVelocity().GetValueAsDouble();
    info.acceleration   = m.GetAcceleration().GetValueAsDouble();

    info.device_temp    = m.GetDeviceTemp().GetValueAsDouble();
    info.processor_temp = m.GetProcessorTemp().GetValueAsDouble();
    info.bus_voltage    = m.GetSupplyVoltage().GetValueAsDouble();
    info.supply_current = m.GetSupplyCurrent().GetValueAsDouble();

    info.output_percent = m.GetDutyCycle().GetValueAsDouble();
    info.output_voltage = m.GetMotorVoltage().GetValueAsDouble();
    info.output_current = m.GetStatorCurrent().GetValueAsDouble();

    info.motor_state    = static_cast<uint8_t>(m.GetMotorOutputStatus().GetValue().value);
    info.bridge_mode    = static_cast<uint8_t>(m.GetBridgeOutput().GetValue().value);
    info.control_mode   = static_cast<uint8_t>(m.GetControlMode().GetValue().value);
    info.enabled        = static_cast<bool>(m.GetDeviceEnable().GetValue().value);

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
    faults.static_brake_disabled_fault = m.GetFault_StaticBrakeDisabled().GetValue();

    faults.sticky_hardware_fault = m.GetStickyFault_Hardware().GetValue();
    faults.sticky_proc_temp_fault = m.GetStickyFault_ProcTemp().GetValue();
    faults.sticky_device_temp_fault = m.GetStickyFault_DeviceTemp().GetValue();
    faults.sticky_undervoltage_fault = m.GetStickyFault_Undervoltage().GetValue();
    faults.sticky_boot_fault = m.GetStickyFault_BootDuringEnable().GetValue();
    faults.sticky_unliscensed_fault = m.GetStickyFault_UnlicensedFeatureInUse().GetValue();
    faults.sticky_bridge_brownout_fault = m.GetStickyFault_BridgeBrownout().GetValue();
    faults.sticky_overvoltage_fault = m.GetStickyFault_OverSupplyV().GetValue();
    faults.sticky_unstable_voltage_fault = m.GetStickyFault_UnstableSupplyV().GetValue();
    faults.sticky_stator_current_limit_fault = m.GetStickyFault_StatorCurrLimit().GetValue();
    faults.sticky_supply_current_limit_fault = m.GetStickyFault_SupplyCurrLimit().GetValue();
    faults.sticky_static_brake_disabled_fault = m.GetStickyFault_StaticBrakeDisabled().GetValue();

    return faults;
}


void Phoenix6Driver::initSerial(const char* port)
{
    this->serial_port = open(port, O_RDWR | O_NOCTTY | O_SYNC);

    if(this->serial_port < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s for motor resets!", port);
        return;
    }

    // Configure port
    termios tty;
    memset(&tty, 0, sizeof(tty));
    if(tcgetattr(this->serial_port, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error configuring (tcgetattr) serial port %s for motor resets!", port);
        return;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);    // enable receiver, ignore modem ctrl lines
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8 bits
    tty.c_cflag &= ~PARENB;             // no parity
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;            // no flow control

    tty.c_lflag = 0;                    // no signaling chars, no echo
    tty.c_oflag = 0;                    // no remapping, no delays
    tty.c_cc[VMIN]  = 1;                // read doesn't return until at least 1 char
    tty.c_cc[VTIME] = 5;                // 0.5 seconds read timeout

    if(tcsetattr(this->serial_port, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error configuring (tcsetattr) serial port %s for motor resets!", port);
        return;
    }
}

void Phoenix6Driver::sendSerialPowerDown()
{
    RCLCPP_INFO(this->get_logger(), "Sending serial power down command...");
    write(this->serial_port, "0", 1);
}
void Phoenix6Driver::sendSerialPowerUp()
{
    RCLCPP_INFO(this->get_logger(), "Sending serial power up command...");
    write(this->serial_port, "1", 1);

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
    if(!status)
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
    phx6::configs::TalonFXConfiguration config =
        phx6::configs::TalonFXConfiguration{}
            .WithSlot0(TalonStaticConfig::SLOT0_CONFIG)
            .WithMotorOutput(TalonStaticConfig::MOTOR_OUTPUT_CONFIG)
            .WithFeedback(TalonStaticConfig::FEEDBACK_CONFIGS);
            // .WithCurrentLimits(TalonStaticConfig::CURRENT_LIMIT_CONFIG);
    
    config.CurrentLimits.StatorCurrentLimit = units::current::ampere_t(10);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t(12);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.Inverted = phx6::signals::InvertedValue::Clockwise_Positive;     // trencher positive should result in digging
    trencher.GetConfigurator().Apply(config);
    trencher.ClearStickyFaults();

    config.MotorOutput.Inverted = phx6::signals::InvertedValue::Clockwise_Positive;     // hopper positive should result in dumping
    hopper_belt.GetConfigurator().Apply(config);
    hopper_belt.ClearStickyFaults();

    config.MotorOutput.Inverted = phx6::signals::InvertedValue::Clockwise_Positive;
    track_right.GetConfigurator().Apply(config);
    track_right.ClearStickyFaults();

    config.MotorOutput.Inverted = phx6::signals::InvertedValue::CounterClockwise_Positive;
    track_left.GetConfigurator().Apply(config);
    track_left.ClearStickyFaults();

    RCLCPP_INFO(this->get_logger(), "Reconfigured motors.");
}

void Phoenix6Driver::pub_motor_info_cb()
{
    TalonInfo talon_info_msg{};
    talon_info_msg.header.stamp = this->get_clock()->now();

    for(size_t i = 0; i < 4; i++)
    {
        this->motor_pub_subs[i].get().info_pub->publish( (talon_info_msg << this->motors[i]) );
    }
}

void Phoenix6Driver::pub_motor_fault_cb()
{
    TalonFaults talon_faults_msg{};
    talon_faults_msg.header.stamp = this->get_clock()->now();

    constexpr uint32_t SUPPLY_CURRENT_FAULT_MASK = 0x100;
    constexpr uint32_t STATOR_CURRENT_FAULT_MASK = 0x200;
    constexpr uint32_t OVERVOLTAGE_FAULT_MASK = 0x40000;
    constexpr uint32_t FAULTS_IGNORE_MASK = (
        SUPPLY_CURRENT_FAULT_MASK |
        STATOR_CURRENT_FAULT_MASK |
        OVERVOLTAGE_FAULT_MASK);

    bool any_faults = false;
    for(size_t i = 0; i < 4; i++)
    {
        this->motor_pub_subs[i].get().faults_pub->publish( (talon_faults_msg << this->motors[i]) );
        any_faults |= (talon_faults_msg.faults & (~FAULTS_IGNORE_MASK));
    }

    if(any_faults)
    {
        this->sendSerialPowerDown();
        std::this_thread::sleep_for(TALONFX_POWER_CYCLE_DELAY);
        this->sendSerialPowerUp();
    }
}

void Phoenix6Driver::execute_ctrl_cb(TalonFX &motor, const TalonCtrl &msg)
{
    if(this->is_disabled)
    {
        motor.SetControl(phx6::controls::NeutralOut());
    }
    else
    {
        switch(msg.mode)
        {
            case TalonCtrl::PERCENT_OUTPUT:
            {
                motor.SetControl(phx6::controls::DutyCycleOut{ msg.value });
                break;
            }
            case TalonCtrl::POSITION:
            {
                motor.SetControl(phx6::controls::PositionVoltage{ units::angle::turn_t{ msg.value } });
                break;
            }
            case TalonCtrl::VELOCITY:
            {
                motor.SetControl(
                    phx6::controls::VelocityVoltage{ units::angular_velocity::turns_per_second_t{ msg.value } }
                        .WithAcceleration(TalonRuntimeConfig::MOTOR_VELOCITY_SETPOINT_ACC) );
                break;
            }
            case TalonCtrl::CURRENT:
            {
                // torque/current control required phoenix pro
                break;
            }
            case TalonCtrl::VOLTAGE:
            {
                motor.SetControl(phx6::controls::VoltageOut{ units::voltage::volt_t{ msg.value } });
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
                motor.SetControl(phx6::controls::MusicTone{ units::frequency::hertz_t{ msg.value } });
                break;
            }
        }
    }
}



int main(int argc, char **argv)
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
