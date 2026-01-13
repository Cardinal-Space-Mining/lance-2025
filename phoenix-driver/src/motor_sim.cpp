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
#include <numbers>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "phoenix_ros_driver/msg/talon_ctrl.hpp"
#include "phoenix_ros_driver/msg/talon_info.hpp"
#include "phoenix_ros_driver/msg/talon_faults.hpp"

#include "ros_utils.hpp"

using namespace util::ros_aliases;
using namespace std::chrono_literals;

using TalonCtrlMsg = phoenix_ros_driver::msg::TalonCtrl;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;
using TalonFaultsMsg = phoenix_ros_driver::msg::TalonFaults;

using OdometryMsg = nav_msgs::msg::Odometry;
using Int32Msg = std_msgs::msg::Int32;
using Float64Msg = std_msgs::msg::Float64;
using JointStateMsg = sensor_msgs::msg::JointState;
using TwistMsg = geometry_msgs::msg::Twist;

static constexpr double TRACK_WIDTH_M = 0.636;
static constexpr double TRACK_EFFECTIVE_OUTPUT_RADIUS_M = 0.07032851;
static constexpr double TRACK_GEARING = 64.;

// velocities in radians per second
static void motor_vels_to_robot_twist(double l, double r, TwistMsg& twist)
{
    constexpr double RATIO =
        (1. / TRACK_GEARING) * (TRACK_EFFECTIVE_OUTPUT_RADIUS_M);

    double v_l = l * RATIO;
    double v_r = r * RATIO;

    twist.linear.x = (v_r + v_l) / 2.0;
    twist.angular.z = (v_r - v_l) / TRACK_WIDTH_M;
}
static double track_m_to_motor_rot(double track_mps)
{
    return track_mps *
           (1. / (TRACK_EFFECTIVE_OUTPUT_RADIUS_M * 2 * std::numbers::pi) * TRACK_GEARING);
}

static double act_val_to_gz_joint_target(double act_val)
{
    double angle = (std::numbers::pi / 180.) * (15. + (act_val / 1000.) * -30.);
    return angle > 0.1 ? 0.1 : angle;

    // L2:
    // return (std::numbers::pi / 180.) * (10. + (act_val / 1000.) * -20.);
}

#define SIM_STEP_DT_MS 1

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
        setpoint_(0.0),
        enabled_(true),
        control_mode_(TalonCtrlMsg::PERCENT_OUTPUT)
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

        if (!enabled_ || control_mode_ == TalonCtrlMsg::DISABLED)
        {
            applied_voltage = 0.0;
            effort = 0.0;
        }
        else
        {
            switch (control_mode_)
            {
                case TalonCtrlMsg::PERCENT_OUTPUT:
                    effort = std::clamp(setpoint_, -1.0, 1.0);
                    applied_voltage = effort * bus_voltage;
                    break;
                case TalonCtrlMsg::VOLTAGE:
                    applied_voltage =
                        std::clamp(setpoint_, -bus_voltage, bus_voltage);
                    effort = applied_voltage / bus_voltage;
                    break;
                case TalonCtrlMsg::VELOCITY:
                {
                    double vel_sp_rad_s =
                        setpoint_ * 2.0 * std::numbers::pi;  // turns/s -> rad/s
                    double kv_rad_s_per_volt =
                        kv_rpm_per_volt_ * (2.0 * std::numbers::pi / 60.0);
                    double vel_error = vel_sp_rad_s - velocity_;
                    double kf_volts = vel_sp_rad_s / kv_rad_s_per_volt;
                    double kp_volts = vel_error * 0.2;  // tune
                    applied_voltage = std::clamp(
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
        double kv_rad_s_per_volt = kv_rpm_per_volt_ * (2.0 * std::numbers::pi / 60.0);
        double back_emf = velocity_ / kv_rad_s_per_volt;
        double voltage_diff = applied_voltage - back_emf;

        // Current
        double current =
            (std::abs(resistance_) > 1e-9) ? voltage_diff / resistance_ : 0.0;
        double stall_current = 257.0;
        current = std::clamp(current, -stall_current, stall_current);

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

    void fill_talon_info(TalonInfoMsg& info, double bus_voltage)
    {
        info.position = position_ / (2.0 * std::numbers::pi);          // turns
        info.velocity = velocity_ / (2.0 * std::numbers::pi);          // turns/s
        info.acceleration = acceleration_ / (2.0 * std::numbers::pi);  // turns/s^2
        info.device_temp = 30.0f;
        info.processor_temp = 30.0f;
        info.bus_voltage = static_cast<float>(bus_voltage);
        info.supply_current = static_cast<float>(std::abs(output_current_));
        info.output_percent = static_cast<float>(output_percent_);
        info.output_voltage = static_cast<float>(output_voltage_);
        info.output_current = static_cast<float>(output_current_);
        info.status = enabled_ ? 0b11 : 0b10;
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

    double setpoint_;
    bool enabled_;
    int control_mode_;
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
        setpoint_(0.0),
        max_speed_(max_speed),
        control_mode_(TalonCtrlMsg::PERCENT_OUTPUT),
        enabled_(true)
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
        if (!enabled_ || control_mode_ == TalonCtrlMsg::DISABLED)
        {
            velocity_ = 0.0;
            output_percent_ = 0.0;
        }
        else if (
            control_mode_ == TalonCtrlMsg::PERCENT_OUTPUT ||
            control_mode_ == TalonCtrlMsg::VOLTAGE)
        {
            output_percent_ = std::clamp(setpoint_, -1.0, 1.0);
            velocity_ = output_percent_ * max_speed_;
        }
        else if (control_mode_ == TalonCtrlMsg::VELOCITY)
        {
            velocity_ = std::clamp(setpoint_, -max_speed_, max_speed_);
            output_percent_ = velocity_ / max_speed_;
        }

        position_ += velocity_ * dt;
        position_ = std::clamp(position_, 0.0, 1000.0);  // clamp travel
    }

    void fill_talon_info(TalonInfoMsg& info, double bus_voltage)
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
        info.status = enabled_ ? 0b11 : 0b10;
        info.control_mode = static_cast<uint8_t>(control_mode_);
    }

    std::string name_;
    double position_;
    double velocity_;
    double output_percent_;

    double setpoint_;
    double max_speed_;
    int control_mode_;
    bool enabled_;
};

// -----------------------------
// Simulator Node
// -----------------------------
class PhoenixPhysicalSimulator : public RclNode
{
public:
    PhoenixPhysicalSimulator() :
        RclNode("phoenix_physical_simulator"),
        battery_(16.0, 0.01),  // 10mΩ internal resistance
        use_gz_track_feedback(util::declare_and_get_param(*this, "use_gz_track_feedback", false))
    {
        std::array<std::string,4> motor_names_{"track_right", "track_left", "trencher", "hopper_belt"};
        for (const auto& name : motor_names_)
        {
            motors_[name] = std::make_shared<FalconMotorSim>(name);
            setup_io(name);
        }
        linear_act_ = std::make_shared<LinearActuatorSim>("hopper_act");
        setup_io("hopper_act");

        watchdog_sub_ = this->create_subscription<Int32Msg>(
            "/lance/watchdog_status",
            rclcpp::SystemDefaultsQoS(),
            [this](const Int32Msg::SharedPtr msg) { on_watchdog(msg->data); });

        // joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        //     "joint_states",
        //     rclcpp::SystemDefaultsQoS());

        sim_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(SIM_STEP_DT_MS),
            std::bind(&PhoenixPhysicalSimulator::sim_callback, this));
        io_timer_ = this->create_wall_timer(
            20ms,
            std::bind(&PhoenixPhysicalSimulator::io_callback, this));

        gz_joint_sub = this->create_subscription<JointStateMsg>(
            "/gz_joint_states",
            rclcpp::SystemDefaultsQoS(),
            [this](const JointStateMsg& msg)
            {
                // std::cout << "Received GZ Joint State Msg" << std::endl;
                for (size_t i = 0; i < msg.name.size(); i++)
                {
                    if (msg.name[i] == "dump_joint")    // L2: "hopper_joint"
                    {
                        double target = act_val_to_gz_joint_target(
                            this->linear_act_->position_);
                        double error = target - msg.position[i];
                        // std::cout << "Actuator target: " << target
                        //           << ", GZ pos: " << msg.position[i]
                        //           << ", error: " << error << std::endl;
                        if (std::abs(error) > 0.02)
                        {
                            this->act_vel_pub->publish(
                                Float64Msg{}.set__data(
                                    std::signbit(error) ? -0.5 : 0.5));
                        }
                        else
                        {
                            this->act_vel_pub->publish(
                                Float64Msg{}.set__data(0.0));
                        }
                        break;
                    }
                }
            });
        left_track_odom_sub = this->create_subscription<OdometryMsg>(
            "/left_track_odom",
            rclcpp::SystemDefaultsQoS(),
            [this](const OdometryMsg& msg)
            {
                this->gz_left_track_pos =
                    track_m_to_motor_rot(msg.pose.pose.position.x);
                this->gz_left_track_vel =
                    track_m_to_motor_rot(msg.twist.twist.linear.x);
            });
        right_track_odom_sub = this->create_subscription<OdometryMsg>(
            "/right_track_odom",
            rclcpp::SystemDefaultsQoS(),
            [this](const OdometryMsg& msg)
            {
                this->gz_right_track_pos =
                    track_m_to_motor_rot(msg.pose.pose.position.x);
                this->gz_right_track_vel =
                    track_m_to_motor_rot(msg.twist.twist.linear.x);
            });
        act_vel_pub = this->create_publisher<Float64Msg>("/dump_cmd_vel", rclcpp::SystemDefaultsQoS());
        track_twist_pub = this->create_publisher<TwistMsg>("/cmd_vel", rclcpp::SystemDefaultsQoS());

        RCLCPP_INFO(this->get_logger(), "Motor sim started!");
    }

private:
    void setup_io(const std::string& name)
    {
        publisher_info_[name] = this->create_publisher<TalonInfoMsg>(
            "/lance/" + name + "/info",
            rclcpp::SystemDefaultsQoS());
        publisher_faults_[name] = this->create_publisher<TalonFaultsMsg>(
            "/lance/" + name + "/faults",
            rclcpp::SystemDefaultsQoS());
        subscription_ctrl_[name] = this->create_subscription<TalonCtrlMsg>(
            "/lance/" + name + "/ctrl",
            rclcpp::SystemDefaultsQoS(),
            [this, name](const TalonCtrlMsg::SharedPtr msg)
            { on_ctrl(name, *msg); });
    }

    void on_ctrl(const std::string& name, const TalonCtrlMsg& msg)
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

    void sim_callback()
    {
        double dt = (SIM_STEP_DT_MS / 1000.);
        double total_current = 0.0;

        for (auto& kv : motors_)
        {
            kv.second->step(dt, last_bus_voltage_);
            total_current += std::abs(kv.second->output_current_);
        }
        linear_act_->step(dt);

        last_bus_voltage_ = battery_.compute_voltage(total_current);
    }

    void io_callback()
    {
        TwistMsg twist_msg;
        motor_vels_to_robot_twist(
            motors_["track_left"]->velocity_,
            motors_["track_right"]->velocity_,
            twist_msg);
        this->track_twist_pub->publish(twist_msg);

        auto now = this->get_clock()->now();

        TalonInfoMsg info_msg;
        TalonFaultsMsg faults_msg;
        info_msg.header.stamp = now;
        faults_msg.header.stamp = now;

        motors_["track_right"]->fill_talon_info(info_msg, last_bus_voltage_);
        if(this->use_gz_track_feedback)
        {
            info_msg.position = gz_right_track_pos;
            info_msg.velocity = gz_right_track_vel;
        }
        publisher_info_["track_right"]->publish(info_msg);
        publisher_faults_["track_right"]->publish(faults_msg);

        motors_["track_left"]->fill_talon_info(info_msg, last_bus_voltage_);
        if(this->use_gz_track_feedback)
        {
            info_msg.position = gz_left_track_pos;
            info_msg.velocity = gz_left_track_vel;
        }
        publisher_info_["track_left"]->publish(info_msg);
        publisher_faults_["track_left"]->publish(faults_msg);

        motors_["trencher"]->fill_talon_info(info_msg, last_bus_voltage_);
        publisher_info_["trencher"]->publish(info_msg);
        publisher_faults_["trencher"]->publish(faults_msg);

        motors_["hopper_belt"]->fill_talon_info(info_msg, last_bus_voltage_);
        publisher_info_["hopper_belt"]->publish(info_msg);
        publisher_faults_["hopper_belt"]->publish(faults_msg);

        linear_act_->fill_talon_info(info_msg, last_bus_voltage_);
        publisher_info_["hopper_act"]->publish(info_msg);
        publisher_faults_["hopper_act"]->publish(faults_msg);
    }

private:
    Battery battery_;
    double last_bus_voltage_ = 16.0;

    double gz_left_track_pos = 0.0;
    double gz_left_track_vel = 0.0;
    double gz_right_track_pos = 0.0;
    double gz_right_track_vel = 0.0;
    bool use_gz_track_feedback = false;

    std::unordered_map<std::string, std::shared_ptr<FalconMotorSim>> motors_;
    std::shared_ptr<LinearActuatorSim> linear_act_;

    std::unordered_map<std::string, SharedPub<TalonInfoMsg>> publisher_info_;
    std::unordered_map<std::string, SharedPub<TalonFaultsMsg>>
        publisher_faults_;
    std::unordered_map<std::string, SharedSub<TalonCtrlMsg>> subscription_ctrl_;

    SharedSub<Int32Msg> watchdog_sub_;

    SharedSub<JointStateMsg> gz_joint_sub;
    SharedSub<OdometryMsg> left_track_odom_sub;
    SharedSub<OdometryMsg> right_track_odom_sub;
    SharedPub<Float64Msg> act_vel_pub;
    SharedPub<TwistMsg> track_twist_pub;

    RclTimer sim_timer_;
    RclTimer io_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PhoenixPhysicalSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
