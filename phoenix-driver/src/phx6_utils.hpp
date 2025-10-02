#pragma once

#define Phoenix_No_WPI
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>

#include <phoenix_ros_driver/msg/talon_ctrl.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>
#include <phoenix_ros_driver/msg/talon_faults.hpp>


#define phx_ ctre::phoenix
#define phx6 ctre::phoenix6

using TalonCtrlMsg = phoenix_ros_driver::msg::TalonCtrl;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;
using TalonFaultsMsg = phoenix_ros_driver::msg::TalonFaults;

using phx6::hardware::Pigeon2;
using phx6::hardware::TalonFX;
using phx6::hardware::TalonFXS;

using phx6::configs::TalonFXConfiguration;
using phx6::configs::TalonFXSConfiguration;
using phx6::configs::Slot0Configs;
using phx6::configs::MotorOutputConfigs;
using phx6::configs::FeedbackConfigs;
using phx6::configs::CurrentLimitsConfigs;
using phx6::configs::VoltageConfigs;

using phx6::signals::NeutralModeValue;
using phx6::signals::InvertedValue;
using phx6::signals::FeedbackSensorSourceValue;


// --- Config Helpers ----------------------------------------------------------

inline TalonFXConfiguration buildFXConfig(
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

// --- Message Serializers -----------------------------------------------------

inline TalonInfoMsg& operator<<(TalonInfoMsg& info, TalonFX& m)
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
    info.status =
        (static_cast<uint8_t>(m.GetDeviceEnable().GetValue().value) << 1) |
        (static_cast<uint8_t>(m.IsConnected()) << 2) |
        (static_cast<uint8_t>(m.HasResetOccurred()) << 3);

    return info;
}

inline TalonFaultsMsg& operator<<(TalonFaultsMsg& faults, TalonFX& m)
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

inline TalonInfoMsg& operator<<(TalonInfoMsg& info, TalonFXS& m)
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
    info.status =
        (static_cast<uint8_t>(m.GetDeviceEnable().GetValue().value) << 1) |
        (static_cast<uint8_t>(m.IsConnected()) << 2) |
        (static_cast<uint8_t>(m.HasResetOccurred()) << 3);

    return info;
}

inline TalonFaultsMsg& operator<<(TalonFaultsMsg& faults, TalonFXS& m)
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

// ---

inline phx_::StatusCode operator<<(TalonFX& motor, const TalonCtrlMsg& msg)
{
    switch (msg.mode)
    {
        case TalonCtrlMsg::PERCENT_OUTPUT:
        {
            return motor.SetControl(phx6::controls::DutyCycleOut{msg.value});
        }
        case TalonCtrlMsg::POSITION:
        {
            return motor.SetControl(
                phx6::controls::PositionVoltage{
                    units::angle::turn_t{msg.value}});
        }
        case TalonCtrlMsg::VELOCITY:
        {
            return motor.SetControl(
                phx6::controls::VelocityVoltage{
                    units::angular_velocity::turns_per_second_t{msg.value}});
        }
        case TalonCtrlMsg::VOLTAGE:
        {
            return motor.SetControl(
                phx6::controls::VoltageOut{units::voltage::volt_t{msg.value}});
        }
        case TalonCtrlMsg::DISABLED:
        {
            return motor.SetControl(phx6::controls::NeutralOut());
        }
        case TalonCtrlMsg::MUSIC_TONE:
        {
            return motor.SetControl(
                phx6::controls::MusicTone{
                    units::frequency::hertz_t{msg.value}});
        }
        case TalonCtrlMsg::CURRENT:
        case TalonCtrlMsg::FOLLOWER:
        case TalonCtrlMsg::MOTION_MAGIC:
        case TalonCtrlMsg::MOTION_PROFILE:
        case TalonCtrlMsg::MOTION_PROFILE_ARC:
        default:
        {
            return phx_::StatusCode{};
        }
    }
}
