#pragma once

#ifndef Phoenix_No_WPI
#define Phoenix_No_WPI
#endif
#include <ctre/Phoenix.h>

#include <phoenix_ros_driver/msg/talon_ctrl.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>
#include <phoenix_ros_driver/msg/talon_faults.hpp>


using TalonCtrlMsg = phoenix_ros_driver::msg::TalonCtrl;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;
using TalonFaultsMsg = phoenix_ros_driver::msg::TalonFaults;
using ctre::phoenix::motorcontrol::can::TalonSRX;


// --- Message Serializers -----------------------------------------------------

inline TalonInfoMsg& operator<<(TalonInfoMsg& info, TalonSRX& m)
{
    info.position = -m.GetSelectedSensorPosition();
    info.velocity = m.GetSelectedSensorVelocity();

    info.device_temp = m.GetTemperature();
    info.bus_voltage = m.GetBusVoltage();
    info.supply_current = m.GetSupplyCurrent();

    info.output_percent = m.GetMotorOutputPercent();
    info.output_voltage = m.GetMotorOutputVoltage();
    info.output_current = m.GetOutputCurrent();

    info.control_mode = static_cast<uint8_t>(m.GetControlMode());
    info.status = static_cast<uint8_t>(m.HasResetOccurred()) << 3;

    return info;
}

inline TalonFaultsMsg& operator<<(TalonFaultsMsg& faults, TalonSRX& m)
{
    Faults f;
    m.GetFaults(f);

    faults.faults = f.ToBitfield();

    faults.hardware_fault = f.HardwareFailure;
    faults.undervoltage_fault = f.UnderVoltage;
    faults.boot_fault = f.ResetDuringEn;
    faults.overvoltage_fault = f.SupplyOverV;
    faults.unstable_voltage_fault = f.SupplyUnstable;

    StickyFaults sf;
    m.GetStickyFaults(sf);

    faults.sticky_faults = sf.ToBitfield();

    faults.sticky_hardware_fault = sf.HardwareESDReset;
    faults.sticky_undervoltage_fault = sf.UnderVoltage;
    faults.sticky_boot_fault = sf.ResetDuringEn;
    faults.sticky_overvoltage_fault = sf.SupplyOverV;
    faults.sticky_unstable_voltage_fault = sf.SupplyUnstable;

    return faults;
}
