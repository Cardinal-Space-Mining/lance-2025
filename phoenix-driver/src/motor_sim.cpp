// --- made by chatgpt ---

// Phoenix Physical Simulator with Falcon500 + Linear Actuator
// Modes: PERCENT_OUTPUT, VOLTAGE, VELOCITY, DISABLED
// Shared battery with sag across all motors

#include <chrono>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "phoenix_ros_driver/msg/talon_ctrl.hpp"
#include "phoenix_ros_driver/msg/talon_info.hpp"
#include "phoenix_ros_driver/msg/talon_faults.hpp"

using namespace std::chrono_literals;

using phoenix_ros_driver::msg::TalonCtrl;
using phoenix_ros_driver::msg::TalonInfo;
using phoenix_ros_driver::msg::TalonFaults;

static double clamp_double(double x, double a, double b)
{
    return (x < a ? a : (x > b ? b : x));
}

// -----------------------------
// Battery Model
// -----------------------------
class Battery
{
public:
    Battery(double nominal_voltage = 16.0, double internal_resistance = 0.02) :
        nominal_voltage_(nominal_voltage),
        internal_resistance_(internal_resistance)
    {
    }

    double compute_voltage(double total_current)
    {
        double sag = total_current * internal_resistance_;
        return std::max(0.0, nominal_voltage_ - sag);
    }

private:
    double nominal_voltage_;
    double internal_resistance_;
};

// -----------------------------
// Falcon 500 Motor Model
// -----------------------------
class FalconMotorSim
{
public:
    FalconMotorSim(
        const std::string& name,
        double inertia = 5e-5,
        double kv_rpm_per_volt = 531.7,
        double kt_nm_per_amp = 0.0184,
        double resistance_ohm = 12.0 / 257.0,
        double damping = 0.002) :
        name_(name),
        kv_rpm_per_volt_(kv_rpm_per_volt),
        kt_nm_per_amp_(kt_nm_per_amp),
        resistance_(resistance_ohm),
        inertia_(inertia),
        damping_(damping),
        position_(0.0),
        velocity_(0.0),
        acceleration_(0.0),
        output_voltage_(0.0),
        output_current_(0.0),
        output_percent_(0.0),
        enabled_(true),
        control_mode_(TalonCtrl::PERCENT_OUTPUT),
        setpoint_(0.0)
    {
    }

    void set_control(int mode, double value)
    {
        control_mode_ = mode;
        setpoint_ = value;
    }

    void set_enabled(bool e) { enabled_ = e; }

    void step(double dt, double bus_voltage)
    {
        double applied_voltage = 0.0;
        double effort = 0.0;

        if (!enabled_ || control_mode_ == TalonCtrl::DISABLED)
        {
            applied_voltage = 0.0;
            effort = 0.0;
        }
        else
        {
            switch (control_mode_)
            {
                case TalonCtrl::PERCENT_OUTPUT:
                    effort = clamp_double(setpoint_, -1.0, 1.0);
                    applied_voltage = effort * bus_voltage;
                    break;
                case TalonCtrl::VOLTAGE:
                    applied_voltage =
                        clamp_double(setpoint_, -bus_voltage, bus_voltage);
                    effort = applied_voltage / bus_voltage;
                    break;
                case TalonCtrl::VELOCITY:
                {
                    double vel_sp_rad_s =
                        setpoint_ * 2.0 * M_PI;  // turns/s -> rad/s
                    double kv_rad_s_per_volt =
                        kv_rpm_per_volt_ * (2.0 * M_PI / 60.0);
                    double vel_error = vel_sp_rad_s - velocity_;
                    double kf_volts = vel_sp_rad_s / kv_rad_s_per_volt;
                    double kp_volts = vel_error * 0.2;  // tune
                    applied_voltage = clamp_double(
                        kf_volts + kp_volts,
                        -bus_voltage,
                        bus_voltage);
                    effort = applied_voltage / bus_voltage;
                    break;
                }
                default:
                    applied_voltage = 0.0;
                    effort = 0.0;
                    break;
            }
        }

        // Back-EMF
        double kv_rad_s_per_volt = kv_rpm_per_volt_ * (2.0 * M_PI / 60.0);
        double back_emf = velocity_ / kv_rad_s_per_volt;
        double voltage_diff = applied_voltage - back_emf;

        // Current
        double current =
            (std::abs(resistance_) > 1e-9) ? voltage_diff / resistance_ : 0.0;
        double stall_current = 257.0;
        current = clamp_double(current, -stall_current, stall_current);

        // Torque
        double torque = kt_nm_per_amp_ * current;
        double torque_net = torque - damping_ * velocity_;

        // Dynamics
        acceleration_ = torque_net / inertia_;
        velocity_ += acceleration_ * dt;
        position_ += velocity_ * dt;

        // Outputs
        output_voltage_ = applied_voltage;
        output_current_ = current;
        output_percent_ = effort;
    }

    void fill_talon_info(TalonInfo& info, double bus_voltage)
    {
        info.position = position_ / (2.0 * M_PI);          // turns
        info.velocity = velocity_ / (2.0 * M_PI);          // turns/s
        info.acceleration = acceleration_ / (2.0 * M_PI);  // turns/s^2
        info.device_temp = 30.0f;
        info.processor_temp = 30.0f;
        info.bus_voltage = static_cast<float>(bus_voltage);
        info.supply_current = static_cast<float>(std::abs(output_current_));
        info.output_percent = static_cast<float>(output_percent_);
        info.output_voltage = static_cast<float>(output_voltage_);
        info.output_current = static_cast<float>(output_current_);
        info.status = enabled_ ? 0b11 : 0;
        info.control_mode = static_cast<uint8_t>(control_mode_);
    }

    std::string name_;
    double kv_rpm_per_volt_;
    double kt_nm_per_amp_;
    double resistance_;
    double inertia_;
    double damping_;

    double position_;
    double velocity_;
    double acceleration_;

    double output_voltage_;
    double output_current_;
    double output_percent_;

    bool enabled_;
    int control_mode_;
    double setpoint_;
};

// -----------------------------
// Linear Actuator Model (simplified)
// -----------------------------
class LinearActuatorSim
{
public:
    LinearActuatorSim(
        const std::string& name,
        double max_speed = 150.0)  // counts/s
        :
        name_(name),
        position_(0.0),
        velocity_(0.0),
        output_percent_(0.0),
        enabled_(true),
        control_mode_(TalonCtrl::PERCENT_OUTPUT),
        setpoint_(0.0),
        max_speed_(max_speed)
    {
    }

    void set_control(int mode, double value)
    {
        control_mode_ = mode;
        setpoint_ = value;
    }

    void set_enabled(bool e) { enabled_ = e; }

    void step(double dt)
    {
        if (!enabled_ || control_mode_ == TalonCtrl::DISABLED)
        {
            velocity_ = 0.0;
            output_percent_ = 0.0;
        }
        else if (
            control_mode_ == TalonCtrl::PERCENT_OUTPUT ||
            control_mode_ == TalonCtrl::VOLTAGE)
        {
            output_percent_ = clamp_double(setpoint_, -1.0, 1.0);
            velocity_ = output_percent_ * max_speed_;
        }
        else if (control_mode_ == TalonCtrl::VELOCITY)
        {
            velocity_ = clamp_double(setpoint_, -max_speed_, max_speed_);
            output_percent_ = velocity_ / max_speed_;
        }

        position_ += velocity_ * dt;
        position_ = clamp_double(position_, 0.0, 1000.0);  // clamp travel
    }

    void fill_talon_info(TalonInfo& info, double bus_voltage)
    {
        info.position = position_;
        info.velocity = velocity_;
        info.acceleration = 0.0;
        info.device_temp = 30.0f;
        info.processor_temp = 30.0f;
        info.bus_voltage = static_cast<float>(bus_voltage);
        info.supply_current = 0.0f;
        info.output_percent = static_cast<float>(output_percent_);
        info.output_voltage = static_cast<float>(output_percent_ * bus_voltage);
        info.output_current = 0.0f;
        info.status = enabled_ ? 0b11 : 0;
        info.control_mode = static_cast<uint8_t>(control_mode_);
    }

    std::string name_;
    double position_;
    double velocity_;
    double output_percent_;

    bool enabled_;
    int control_mode_;
    double setpoint_;
    double max_speed_;
};

// -----------------------------
// Simulator Node
// -----------------------------
class PhoenixPhysicalSimulator : public rclcpp::Node
{
public:
    PhoenixPhysicalSimulator() :
        Node("phoenix_physical_simulator"),
        qos_{rclcpp::SystemDefaultsQoS()},
        battery_(16.0, 0.01)  // 20mΩ internal resistance
    {
        motor_names_ = {"track_right", "track_left", "trencher", "hopper_belt"};
        for (const auto& name : motor_names_)
        {
            motors_[name] = std::make_shared<FalconMotorSim>(name);
        }
        linear_act_ = std::make_shared<LinearActuatorSim>("hopper_act");

        for (const auto& name : motor_names_)
        {
            setup_io(name);
        }
        setup_io("hopper_act");

        watchdog_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/lance/watchdog_status",
            qos_,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            { on_watchdog(msg->data); });

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states",
            qos_);

        timer_ = this->create_wall_timer(
            1ms,
            std::bind(&PhoenixPhysicalSimulator::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Motor sim started!");
    }

private:
    void setup_io(const std::string& name)
    {
        publisher_info_[name] =
            this->create_publisher<TalonInfo>("/lance/" + name + "/info", qos_);
        publisher_faults_[name] = this->create_publisher<TalonFaults>(
            "/lance/" + name + "/faults",
            qos_);
        subscription_ctrl_[name] = this->create_subscription<TalonCtrl>(
            "/lance/" + name + "/ctrl",
            qos_,
            [this, name](const TalonCtrl::SharedPtr msg)
            { on_ctrl(name, *msg); });
    }

    void on_ctrl(const std::string& name, const TalonCtrl& msg)
    {
        if (name == "hopper_act")
        {
            linear_act_->set_control(msg.mode, msg.value);
        }
        else
        {
            auto it = motors_.find(name);
            if (it != motors_.end())
            {
                it->second->set_control(msg.mode, msg.value);
            }
        }
    }

    void on_watchdog(int32_t status)
    {
        bool enable = (status != 0);
        for (auto& kv : motors_)
        {
            kv.second->set_enabled(enable);
        }
        linear_act_->set_enabled(enable);
    }

    void timer_callback()
    {
        double dt = 0.001;
        double total_current = 0.0;

        for (auto& kv : motors_)
        {
            kv.second->step(dt, last_bus_voltage_);
            total_current += std::abs(kv.second->output_current_);
        }
        linear_act_->step(dt);

        last_bus_voltage_ = battery_.compute_voltage(total_current);

        auto now = this->get_clock()->now();
        for (auto& kv : motors_)
        {
            TalonInfo info;
            info.header.stamp = now;
            kv.second->fill_talon_info(info, last_bus_voltage_);
            publisher_info_[kv.first]->publish(info);
            TalonFaults faults;
            faults.header.stamp = now;
            publisher_faults_[kv.first]->publish(faults);
        }
        TalonInfo act_info;
        act_info.header.stamp = now;
        linear_act_->fill_talon_info(act_info, last_bus_voltage_);
        publisher_info_["hopper_act"]->publish(act_info);
        TalonFaults act_faults;
        act_faults.header.stamp = now;
        publisher_faults_["hopper_act"]->publish(act_faults);

        // sensor_msgs::msg::JointState js;
        // js.header.stamp = now;
        // for (auto& kv : motors_)
        // {
        //     js.name.push_back(kv.first + "_joint");
        //     js.position.push_back(kv.second->position_);
        //     js.velocity.push_back(kv.second->velocity_);
        //     js.effort.push_back(kv.second->output_percent_ * 4.69);
        // }
        // js.name.push_back("hopper_act_joint");
        // js.position.push_back(linear_act_->position_);
        // js.velocity.push_back(linear_act_->velocity_);
        // js.effort.push_back(linear_act_->output_percent_);
        // joint_pub_->publish(js);
    }

    rclcpp::QoS qos_;
    Battery battery_;
    double last_bus_voltage_ = 16.0;

    std::vector<std::string> motor_names_;
    std::unordered_map<std::string, std::shared_ptr<FalconMotorSim>> motors_;
    std::shared_ptr<LinearActuatorSim> linear_act_;

    std::unordered_map<std::string, rclcpp::Publisher<TalonInfo>::SharedPtr>
        publisher_info_;
    std::unordered_map<std::string, rclcpp::Publisher<TalonFaults>::SharedPtr>
        publisher_faults_;
    std::unordered_map<std::string, rclcpp::Subscription<TalonCtrl>::SharedPtr>
        subscription_ctrl_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr watchdog_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PhoenixPhysicalSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
