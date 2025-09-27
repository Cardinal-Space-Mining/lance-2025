#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>

#define Phoenix_No_WPI
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include <phoenix_ros_driver/msg/talon_info.hpp>
#include <phoenix_ros_driver/msg/talon_faults.hpp>


using namespace std::chrono_literals;

using Float64Msg = std_msgs::msg::Float64;
using JoyMsg = sensor_msgs::msg::Joy;

#define phx6 ctre::phoenix6
using phoenix_ros_driver::msg::TalonInfo;
using phoenix_ros_driver::msg::TalonFaults;
using phx6::hardware::TalonFX;


#define INTF_NAME "can0"
#define CAN_ID    4


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


class FalconTestNode : public rclcpp::Node
{
public:
    FalconTestNode();

protected:
    void config_motor();

    void joy_cb(const JoyMsg& msg);

protected:
    TalonFX motor;

    rclcpp::Publisher<TalonInfo>::SharedPtr info_pub;
    rclcpp::Publisher<TalonFaults>::SharedPtr faults_pub;
    rclcpp::Publisher<Float64Msg>::SharedPtr target_pub;

    rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub;
};



FalconTestNode::FalconTestNode() :
    Node("falcon_test_node"),
    motor{CAN_ID, INTF_NAME},
    info_pub{this->create_publisher<TalonInfo>(
        "/test_falcon/info",
        rclcpp::SensorDataQoS{})},
    faults_pub{this->create_publisher<TalonFaults>(
        "/test_falcon/faults",
        rclcpp::SensorDataQoS{})},
    target_pub{this->create_publisher<Float64Msg>(
        "/test_falcon/target",
        rclcpp::SensorDataQoS{})},
    joy_sub{this->create_subscription<JoyMsg>(
        "/joy",
        rclcpp::SensorDataQoS{},
        [this](const JoyMsg& msg) { this->joy_cb(msg); })}
{
    this->config_motor();
}

void FalconTestNode::config_motor()
{
    static constexpr double kP = 0.5;
    static constexpr double kI = 0.2;
    static constexpr double kD = 0.0001;
    static constexpr double kV = 0.115;

    static constexpr double DUTY_CYCLE_DEADBAND = 0.05;
    static constexpr int NEUTRAL_MODE =
        ctre::phoenix6::signals::NeutralModeValue::Coast;

    static constexpr phx6::configs::Slot0Configs SLOT0_CONFIG =
        phx6::configs::Slot0Configs{}.WithKP(kP).WithKI(kI).WithKD(kD).WithKV(
            kV);
    static constexpr phx6::configs::MotorOutputConfigs MOTOR_OUTPUT_CONFIG =
        phx6::configs::MotorOutputConfigs{}
            .WithDutyCycleNeutralDeadband(DUTY_CYCLE_DEADBAND)
            .WithNeutralMode(NEUTRAL_MODE);
    static constexpr phx6::configs::FeedbackConfigs FEEDBACK_CONFIGS =
        phx6::configs::FeedbackConfigs{}.WithFeedbackSensorSource(
            phx6::signals::FeedbackSensorSourceValue::RotorSensor);
    static const phx6::configs::CurrentLimitsConfigs CURRENT_LIMIT_CONFIG =
        phx6::configs::CurrentLimitsConfigs{}
            .WithStatorCurrentLimitEnable(true)
            .WithStatorCurrentLimit(3_A)
            .WithSupplyCurrentLimitEnable(true)
            .WithSupplyCurrentLimit(3_A);

    phx6::configs::TalonFXConfiguration config =
        phx6::configs::TalonFXConfiguration{}
            .WithSlot0(SLOT0_CONFIG)
            .WithMotorOutput(MOTOR_OUTPUT_CONFIG)
            .WithFeedback(FEEDBACK_CONFIGS)
            .WithCurrentLimits(CURRENT_LIMIT_CONFIG);
    config.MotorOutput.Inverted =
        phx6::signals::InvertedValue::CounterClockwise_Positive;

    this->motor.GetConfigurator().Apply(config);
    this->motor.ClearStickyFaults();
}

void FalconTestNode::joy_cb(const JoyMsg& msg)
{
    ctre::phoenix::unmanaged::FeedEnable(100);

    double target_rps = 20. * msg.axes[1] * ((1. - msg.axes[2]) * 2. + 1.);

    this->motor.SetControl(
        phx6::controls::VelocityVoltage{
            units::angular_velocity::turns_per_second_t{target_rps}});

    this->target_pub->publish(Float64Msg{}.set__data(target_rps));

    TalonInfo info_msg{};
    this->info_pub->publish((info_msg << this->motor));
    TalonFaults faults_msg{};
    this->faults_pub->publish((faults_msg << this->motor));
}



int main(int argc, char** argv)
{
    ctre::phoenix::unmanaged::LoadPhoenix();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FalconTestNode>());
    rclcpp::shutdown();
}
