#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#define Phoenix_No_WPI
#include <ctre/phoenix/cci/Unmanaged_CCI.h>
#include <ctre/phoenix/cci/Diagnostics_CCI.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>

#include "ros_utils.hpp"
#include "phx5_utils.hpp"


using namespace util;
using namespace std::chrono_literals;

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_SUB_QOS    10

#define DEFAULT_CAN_INTERFACE    "can_phx5"
#define DEFAULT_MOTOR_CAN_ID     4
#define DEFAULT_DIAG_SERVER_PORT 1250


class Phoenix5Driver : public rclcpp::Node
{
    using Int32Msg = std_msgs::msg::Int32;

    struct RclTalonSRX
    {
        TalonSRX motor;
        RclPubPtr<TalonInfoMsg> info_pub;
        RclPubPtr<TalonFaultsMsg> faults_pub;
        RclSubPtr<TalonCtrlMsg> ctrl_sub;
    };

    struct TempParams
    {
        int diagnostics_server_port;
    };

public:
    Phoenix5Driver();
    ~Phoenix5Driver();

private:
    void getParams(TempParams& params);
    void initMotor(const TempParams& params);
    void startDiagnostics(const TempParams& params);

    void feed_watchdog_status(int32_t status);

    void pub_motor_info_cb();
    void pub_motor_fault_cb();

    void execute_ctrl(TalonSRX& motor, const TalonCtrlMsg& msg);

private:
    RclTalonSRX hopper_act;

    RclSubPtr<Int32Msg> watchdog_status_sub;

    RclTimerPtr info_pub_timer;
    RclTimerPtr fault_pub_timer;

    bool is_disabled = false;
};


Phoenix5Driver::Phoenix5Driver() :
    Node{ "phoenix5_driver" },
    hopper_act{
        .motor{DEFAULT_MOTOR_CAN_ID, DEFAULT_CAN_INTERFACE},
        .info_pub{this->create_publisher<TalonInfoMsg>(
            ROBOT_TOPIC("hopper_act/info"),
            rclcpp::SensorDataQoS{})},
        .faults_pub{this->create_publisher<TalonFaultsMsg>(
            ROBOT_TOPIC("hopper_act/faults"),
            rclcpp::SensorDataQoS{})},
        .ctrl_sub{this->create_subscription<TalonCtrlMsg>(
            ROBOT_TOPIC("hopper_act/ctrl"),
            TALON_CTRL_SUB_QOS,
            [this](const TalonCtrlMsg& msg)
            { this->execute_ctrl(this->hopper_act.motor, msg); })}},
    watchdog_status_sub{this->create_subscription<Int32Msg>(
        ROBOT_TOPIC("watchdog_status"),
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& msg) { this->feed_watchdog_status(msg.data); })},
    info_pub_timer{this->create_wall_timer(
        100ms,
        [this]() { this->pub_motor_info_cb(); })},
    fault_pub_timer{this->create_wall_timer(
        250ms,
        [this]() { this->pub_motor_fault_cb(); })}
{
    TempParams params;
    this->getParams(params);
    this->initMotor(params);
    this->startDiagnostics(params);

    RCLCPP_DEBUG(
        this->get_logger(),
        "Completed Phoenix5 Driver Node Initialization");
}

Phoenix5Driver::~Phoenix5Driver() { c_Phoenix_Diagnostics_Dispose(); }


void Phoenix5Driver::getParams(TempParams& params)
{
    // declare_param<std::string>(
    //     this,
    //     "canbus",
    //     params.canbus,
    //     DEFAULT_CAN_INTERFACE);
    // declare_param(this, "motor_id", params.can_id, DEFAULT_MOTOR_CAN_ID);
    declare_param(
        this,
        "diagnostics_server_port",
        params.diagnostics_server_port,
        DEFAULT_DIAG_SERVER_PORT);
}

void Phoenix5Driver::initMotor(const TempParams& params)
{
    (void)params;

    // this->hopper_act.motor =
    //     std::make_unique<TalonSRX>(params.can_id, params.canbus);

    this->hopper_act.motor.ConfigFactoryDefault();
    this->hopper_act.motor.ConfigSelectedFeedbackSensor(
        TalonSRXFeedbackDevice::Analog);
    this->hopper_act.motor.SetInverted(true);
    this->hopper_act.motor.SetNeutralMode(NeutralMode::Brake);
    this->hopper_act.motor.ConfigNeutralDeadband(5.);  // <-- percent
    this->hopper_act.motor.ClearStickyFaults();

    // this->hopper_act.info_pub = this->create_publisher<TalonInfoMsg>(
    //     ROBOT_TOPIC("hopper_act/info"),
    //     rclcpp::SensorDataQoS{});
    // this->hopper_act.faults_pub = this->create_publisher<TalonFaultsMsg>(
    //     ROBOT_TOPIC("hopper_act/faults"),
    //     rclcpp::SensorDataQoS{});
    // this->hopper_act.ctrl_sub = this->create_subscription<TalonCtrlMsg>(
    //     ROBOT_TOPIC("hopper_act/ctrl"),
    //     TALON_CTRL_SUB_QOS,
    //     [this](const TalonCtrlMsg& msg)
    //     { this->execute_ctrl(*this->hopper_act.motor, msg); });
}

void Phoenix5Driver::startDiagnostics(const TempParams& params)
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

void Phoenix5Driver::feed_watchdog_status(int32_t status)
{
    /* Watchdog feed decoding:
     * POSTIVE feed time --> enabled
     * ZERO feed time --> disabled
     * NEGATIVE feed time --> autonomous */
    if (!status)
    {
        this->hopper_act.motor.Set(ControlMode::Disabled, 0.);
        this->is_disabled = true;
    }
    else
    {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(std::abs(status));
        this->is_disabled = false;
    }
}

void Phoenix5Driver::pub_motor_info_cb()
{
    TalonInfoMsg info_msg{};
    info_msg.header.stamp = this->get_clock()->now();
    info_msg.enabled = !this->is_disabled;

    this->hopper_act.info_pub->publish((info_msg << this->hopper_act.motor));
}

void Phoenix5Driver::pub_motor_fault_cb()
{
    TalonFaultsMsg faults_msg{};
    faults_msg.header.stamp = this->get_clock()->now();

    this->hopper_act.faults_pub->publish(
        (faults_msg << this->hopper_act.motor));
}

void Phoenix5Driver::execute_ctrl(TalonSRX& motor, const TalonCtrlMsg& msg)
{
    if (this->is_disabled)
    {
        motor.Set(ControlMode::Disabled, 0.);
    }
    else
    {
        switch (msg.mode)
        {
            case TalonCtrlMsg::PERCENT_OUTPUT:
            {
                motor.Set(ControlMode::PercentOutput, msg.value);
                break;
            }
            case TalonCtrlMsg::POSITION:
            {
                motor.Set(ControlMode::Position, msg.value);
                break;
            }
            case TalonCtrlMsg::VELOCITY:
            {
                motor.Set(ControlMode::Velocity, msg.value);
                break;
            }
            case TalonCtrlMsg::CURRENT:
            {
                motor.Set(ControlMode::Current, msg.value);
                break;
            }
            case TalonCtrlMsg::DISABLED:
            {
                motor.Set(ControlMode::Disabled, 0.);
                return;
            }
            case TalonCtrlMsg::FOLLOWER:
            {
                motor.Set(ControlMode::Follower, msg.value);
                break;
            }
            case TalonCtrlMsg::MUSIC_TONE:
            {
                motor.Set(ControlMode::MusicTone, msg.value);
                break;
            }
            case TalonCtrlMsg::VOLTAGE:
            case TalonCtrlMsg::MOTION_MAGIC:
            case TalonCtrlMsg::MOTION_PROFILE:
            case TalonCtrlMsg::MOTION_PROFILE_ARC:
            default:
            {
                break;
            }
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "Set motor mode to %d and output to: %f",
            static_cast<int>(msg.mode),
            msg.value);
    }
}



int main(int argc, char** argv)
{
    ctre::phoenix::unmanaged::Unmanaged::LoadPhoenix();
    std::cout << "Loaded Phoenix 5 Unmanaged" << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Phoenix5Driver>();
    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix5) has started");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix5) shutting down...");
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
