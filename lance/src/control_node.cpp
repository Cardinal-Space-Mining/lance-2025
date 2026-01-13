/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
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

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <csm_metrics/profiling.hpp>

#include "util/pub_map.hpp"
#include "util/joy_utils.hpp"
#include "util/ros_utils.hpp"

#include "robot/robot_math.hpp"
#include "robot/motor_interface.hpp"
#include "robot/robot_controller.hpp"


#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_PUB_QOS    10
#define HOPPER_JOINT_NAME     "dump_joint"  // L2: "hopper_joint"

using namespace std::chrono_literals;
using namespace util::ros_aliases;

using util::JoyState;
using util::GenericPubMap;

using BoolMsg = std_msgs::msg::Bool;
using Int32Msg = std_msgs::msg::Int32;
using Float64Msg = std_msgs::msg::Float64;
using JoyMsg = sensor_msgs::msg::Joy;
using JointStateMsg = sensor_msgs::msg::JointState;


class RobotControlNode : public rclcpp::Node
{
protected:
    struct TalonPubSub
    {
        SharedPub<TalonCtrlMsg> ctrl_pub;
        SharedSub<TalonInfoMsg> info_sub;
    };

public:
#define INIT_TALON_PUB_SUB(device_topic, device_var)         \
    device_var##_pub_sub{                                    \
        this->create_publisher<TalonCtrlMsg>(                \
            ROBOT_TOPIC(#device_topic "/ctrl"),              \
            TALON_CTRL_PUB_QOS),                             \
        this->create_subscription<TalonInfoMsg>(             \
            ROBOT_TOPIC(#device_topic "/info"),              \
            rclcpp::SensorDataQoS{},                         \
            [this](const TalonInfoMsg& msg)                  \
            { this->robot_motor_status.device_var = msg; })}

    RobotControlNode() :
        Node{"robot_control"},
        pub_map{*this, "", rclcpp::SensorDataQoS{}},
        robot_controller{*this, this->pub_map},

        INIT_TALON_PUB_SUB(track_right, track_right),
        INIT_TALON_PUB_SUB(track_left, track_left),
        INIT_TALON_PUB_SUB(trencher, trencher),
        INIT_TALON_PUB_SUB(hopper_belt, hopper_belt),
        INIT_TALON_PUB_SUB(hopper_act, hopper_actuator),

        joy_sub{this->create_subscription<JoyMsg>(
            "/joy",
            rclcpp::SensorDataQoS{},
            [this](const JoyMsg::ConstSharedPtr& msg)
            { this->last_joy_msg = msg; })},
        watchdog_sub{this->create_subscription<Int32Msg>(
            ROBOT_TOPIC("watchdog_status"),
            rclcpp::SensorDataQoS{},
            [this](const Int32Msg& status)
            { this->watchdog_status = status.data; })},

        control_iteration_timer{this->create_wall_timer(
            std::chrono::duration<float>(
                this->robot_controller.getParams().iteration_period_seconds),
            [this]()
            {
                PROFILING_SYNC();
                PROFILING_NOTIFY_ALWAYS(iterate_control);

                if (this->last_joy_msg)
                {
                    this->joy_state.update(*this->last_joy_msg);
                    this->last_joy_msg = nullptr;
                }

                RobotMotorCommands commands;
                this->robot_controller.iterate(
                    this->watchdog_status,
                    this->joy_state,
                    this->robot_motor_status,
                    commands);

                this->track_right_pub_sub.ctrl_pub->publish(
                    commands.track_right);
                this->track_left_pub_sub.ctrl_pub->publish(commands.track_left);
                this->trencher_pub_sub.ctrl_pub->publish(commands.trencher);
                this->hopper_belt_pub_sub.ctrl_pub->publish(
                    commands.hopper_belt);
                this->hopper_actuator_pub_sub.ctrl_pub->publish(
                    commands.hopper_actuator);

                this->publishHopperJoint();
                this->publishCollectionState();

                PROFILING_NOTIFY_ALWAYS(iterate_control);
                PROFILING_FLUSH();
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
            this->robot_motor_status.getHopperActNormalizedValue()));
        this->pub_map.publish("joint_states", joint_msg);
    }
    void publishCollectionState()
    {
        const HopperState& hopper_state = this->robot_controller.hopperState();

        this->pub_map.publish(
            "/collection_state/is_full_volume",
            BoolMsg{}.set__data(hopper_state.isVolCapacity()));
        this->pub_map.publish(
            "/collection_state/is_full_occ",
            BoolMsg{}.set__data(hopper_state.isBeltCapacity()));
        this->pub_map.publish(
            "/collection_state/volume",
            Float64Msg{}.set__data(hopper_state.volume()));
        this->pub_map.publish(
            "/collection_state/mining_target",
            Float64Msg{}.set__data(hopper_state.miningTargetMotorPosition()));
        this->pub_map.publish(
            "/collection_state/offload_target",
            Float64Msg{}.set__data(hopper_state.offloadTargetMotorPosition()));
        this->pub_map.publish(
            "/collection_state/belt_pos_m",
            Float64Msg{}.set__data(hopper_state.beltPosMeters()));
        this->pub_map.publish(
            "/collection_state/high_pos_m",
            Float64Msg{}.set__data(hopper_state.startPosMeters()));
        this->pub_map.publish(
            "/collection_state/low_pos_m",
            Float64Msg{}.set__data(hopper_state.endPosMeters()));
        this->pub_map.publish(
            "/collection_state/belt_usage_m",
            Float64Msg{}.set__data(hopper_state.beltUsageMeters()));
    }

private:
    GenericPubMap pub_map;
    RobotController robot_controller;

    TalonPubSub track_right_pub_sub;
    TalonPubSub track_left_pub_sub;
    TalonPubSub trencher_pub_sub;
    TalonPubSub hopper_belt_pub_sub;
    TalonPubSub hopper_actuator_pub_sub;

    SharedSub<JoyMsg> joy_sub;
    SharedSub<Int32Msg> watchdog_sub;
    RclTimer control_iteration_timer;

    RobotMotorStatus robot_motor_status;
    JoyMsg::ConstSharedPtr last_joy_msg{nullptr};
    JoyState joy_state;
    int32_t watchdog_status{0};
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlNode>();
    PROFILING_INIT(*node, PROFILING_DEFAULT_TOPIC, PROFILING_DEFAULT_QOS);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
