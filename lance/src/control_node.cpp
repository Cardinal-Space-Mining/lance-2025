/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robot/motor_interface.hpp"
#include "robot/controller.hpp"
#include "robot/robot_math.hpp"

// #include "util/pub_map.hpp"


using namespace std::chrono_literals;

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_PUB_QOS    10
#define MOTOR_UPDATE_DT       50ms
#define HOPPER_JOINT_NAME     "dump_joint"


using BoolMsg = std_msgs::msg::Bool;
using Int32Msg = std_msgs::msg::Int32;
using StringMsg = std_msgs::msg::String;
using Float64Msg = std_msgs::msg::Float64;
using JointStateMsg = sensor_msgs::msg::JointState;

template<typename T>
using RclPubPtr = typename rclcpp::Publisher<T>::SharedPtr;
template<typename T>
using RclSubPtr = typename rclcpp::Subscription<T>::SharedPtr;
using RclTimerPtr = rclcpp::TimerBase::SharedPtr;


class CollectionStatePublisher
{
public:
    inline CollectionStatePublisher(rclcpp::Node& n) :
        is_vol_cap_pub{n.create_publisher<BoolMsg>(
            "/collection_state/is_full_volume",
            rclcpp::SensorDataQoS{})},
        is_full_occ_pub{n.create_publisher<BoolMsg>(
            "/collection_state/is_full_occ",
            rclcpp::SensorDataQoS{})},
        vol_pub{n.create_publisher<Float64Msg>(
            "/collection_state/volume",
            rclcpp::SensorDataQoS{})},
        mining_target_pub{n.create_publisher<Float64Msg>(
            "/collection_state/mining_target",
            rclcpp::SensorDataQoS{})},
        offload_target_pub{n.create_publisher<Float64Msg>(
            "/collection_state/offload_target",
            rclcpp::SensorDataQoS{})},
        belt_pos_pub{n.create_publisher<Float64Msg>(
            "/collection_state/belt_pos_m",
            rclcpp::SensorDataQoS{})},
        high_pos_pub{n.create_publisher<Float64Msg>(
            "/collection_state/high_pos_m",
            rclcpp::SensorDataQoS{})},
        low_pos_pub{n.create_publisher<Float64Msg>(
            "/collection_state/low_pos_m",
            rclcpp::SensorDataQoS{})},
        belt_usage_pub{n.create_publisher<Float64Msg>(
            "/collection_state/belt_usage_m",
            rclcpp::SensorDataQoS{})}
    {
    }

public:
    inline void publish(const HopperState& hopper_state)
    {
        this->is_vol_cap_pub->publish(
            BoolMsg{}.set__data(hopper_state.isVolCapacity()));
        this->is_full_occ_pub->publish(
            BoolMsg{}.set__data(hopper_state.isBeltCapacity()));
        this->vol_pub->publish(Float64Msg{}.set__data(hopper_state.volume()));
        this->mining_target_pub->publish(
            Float64Msg{}.set__data(hopper_state.miningTargetMotorPosition()));
        this->offload_target_pub->publish(
            Float64Msg{}.set__data(hopper_state.offloadTargetMotorPosition()));
        this->belt_pos_pub->publish(
            Float64Msg{}.set__data(hopper_state.beltPosMeters()));
        this->high_pos_pub->publish(
            Float64Msg{}.set__data(hopper_state.startPosMeters()));
        this->low_pos_pub->publish(
            Float64Msg{}.set__data(hopper_state.endPosMeters()));
        this->belt_usage_pub->publish(
            Float64Msg{}.set__data(hopper_state.beltUsageMeters()));
    }

protected:
    RclPubPtr<BoolMsg> is_vol_cap_pub, is_full_occ_pub;
    RclPubPtr<Float64Msg> vol_pub, mining_target_pub, offload_target_pub,
        belt_pos_pub, high_pos_pub, low_pos_pub, belt_usage_pub;
};


class RobotControlNode : public rclcpp::Node
{
protected:
    struct TalonPubSub
    {
        RclPubPtr<TalonCtrl> ctrl_pub;
        RclSubPtr<TalonInfo> info_sub;
    };

public:
#define INIT_TALON_PUB_SUB(device_topic, device_var)         \
    device_var##_pub_sub{                                    \
        this->create_publisher<TalonCtrl>(                   \
            ROBOT_TOPIC(#device_topic "/ctrl"),              \
            TALON_CTRL_PUB_QOS),                             \
        this->create_subscription<TalonInfo>(                \
            ROBOT_TOPIC(#device_topic "/info"),              \
            rclcpp::SensorDataQoS{},                         \
            [this](const TalonInfo& msg)                     \
            { this->robot_motor_status.device_var = msg; })}

    RobotControlNode() :
        Node{"robot_control"},

        INIT_TALON_PUB_SUB(track_right, track_right),
        INIT_TALON_PUB_SUB(track_left, track_left),
        INIT_TALON_PUB_SUB(trencher, trencher),
        INIT_TALON_PUB_SUB(hopper_belt, hopper_belt),
        INIT_TALON_PUB_SUB(hopper_act, hopper_actuator),

        control_level_pub{this->create_publisher<StringMsg>(
            ROBOT_TOPIC("control_level"),
            rclcpp::SensorDataQoS{})},
        mining_status_pub{this->create_publisher<StringMsg>(
            ROBOT_TOPIC("mining_status"),
            rclcpp::SensorDataQoS{})},
        offload_status_pub{this->create_publisher<StringMsg>(
            ROBOT_TOPIC("offload_status"),
            rclcpp::SensorDataQoS{})},

        joint_pub{this->create_publisher<JointStateMsg>(
            "joint_states",
            rclcpp::SensorDataQoS{})},

        collection_state_pub{*this},

        joy_sub{this->create_subscription<JoyMsg>(
            "/joy",
            rclcpp::SensorDataQoS{},
            [this](const JoyMsg& joy) { this->joystick_values = joy; })},
        watchdog_sub{this->create_subscription<Int32Msg>(
            ROBOT_TOPIC("watchdog_status"),
            rclcpp::SensorDataQoS{},
            [this](const Int32Msg& status)
            { this->watchdog_status = status.data; })},

        control_iteration_timer{this->create_wall_timer(
            MOTOR_UPDATE_DT,
            [this]()
            {
                const RobotMotorCommands& mc = this->robot_controller.update(
                    this->watchdog_status,
                    this->joystick_values,
                    this->robot_motor_status);

                this->track_right_pub_sub.ctrl_pub->publish(mc.track_right);
                this->track_left_pub_sub.ctrl_pub->publish(mc.track_left);
                this->trencher_pub_sub.ctrl_pub->publish(mc.trencher);
                this->hopper_belt_pub_sub.ctrl_pub->publish(mc.hopper_belt);
                this->hopper_actuator_pub_sub.ctrl_pub->publish(
                    mc.hopper_actuator);

                this->publishHopperJoint();
                this->collection_state_pub.publish(
                    this->robot_controller.getHopperState());

                static StringMsg control_level_msg;
                static StringMsg mining_status_msg;
                static StringMsg offload_status_msg;

                this->robot_controller.getStatusStrings(
                    control_level_msg.data,
                    mining_status_msg.data,
                    offload_status_msg.data);

                this->control_level_pub->publish(control_level_msg);
                this->mining_status_pub->publish(mining_status_msg);
                this->offload_status_pub->publish(offload_status_msg);
            })}
    {
    }

private:
    void publishHopperJoint()
    {
        JointStateMsg joint_msg;
        joint_msg.header = this->robot_motor_status.hopper_actuator.header;
        joint_msg.name.push_back(HOPPER_JOINT_NAME);
        joint_msg.position.push_back(linear_actuator_to_joint_angle(
            this->robot_motor_status.hopper_actuator.position / 1000.));
        this->joint_pub->publish(joint_msg);
    }

private:
    TalonPubSub track_right_pub_sub;
    TalonPubSub track_left_pub_sub;
    TalonPubSub trencher_pub_sub;
    TalonPubSub hopper_belt_pub_sub;
    TalonPubSub hopper_actuator_pub_sub;

    RclPubPtr<StringMsg> control_level_pub;
    RclPubPtr<StringMsg> mining_status_pub;
    RclPubPtr<StringMsg> offload_status_pub;

    RclPubPtr<JointStateMsg> joint_pub;
    CollectionStatePublisher collection_state_pub;

    RclSubPtr<JoyMsg> joy_sub;
    RclSubPtr<Int32Msg> watchdog_sub;
    RclTimerPtr control_iteration_timer;

    RobotControl robot_controller;

    RobotMotorStatus robot_motor_status;
    JoyMsg joystick_values;
    int32_t watchdog_status{0};
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControlNode>());
    rclcpp::shutdown();

    return 0;
}
