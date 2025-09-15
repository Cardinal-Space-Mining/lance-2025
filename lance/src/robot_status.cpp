#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "lance/srv/set_robot_mode.hpp"


using namespace std::chrono_literals;
using namespace std::chrono;
using lance::srv::SetRobotMode;

#define WATCHDOG_PUB_DT           100ms
#define WATCHDOG_TELEOP_FEED_TIME 250ms
#define WATCHDOG_AUTO_FEED_TIME   10000ms

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic

class RobotStatusServer : public rclcpp::Node
{
public:
    RobotStatusServer() :
        Node("robot_status"),

        watchdog_status_pub{this->create_publisher<std_msgs::msg::Int32>(
            ROBOT_TOPIC("watchdog_status"),
            rclcpp::SensorDataQoS{})},
        robot_state_service{this->create_service<SetRobotMode>(
            ROBOT_TOPIC("set_robot_mode"),
            [this](
                SetRobotMode::Request::SharedPtr req,
                SetRobotMode::Response::SharedPtr resp)
            {
                this->robot_mode = req->mode;
                RCLCPP_INFO(
                    this->get_logger(),
                    "SET ROBOT MODE : %d",
                    this->robot_mode);
                resp->success = true;
            })},
        watchdog_timer{this->create_wall_timer(
            WATCHDOG_PUB_DT,
            [this]()
            {
                this->watchdog_status_pub->publish(
                    std_msgs::msg::Int32{}.set__data(this->getFeedTime()));
            })}
    {
    }

protected:
    inline int32_t getFeedTime()
    {
        return (
            this->robot_mode > 0
                ? (duration_cast<milliseconds>(WATCHDOG_TELEOP_FEED_TIME)
                       .count())
                : (this->robot_mode < 0
                       ? -duration_cast<milliseconds>(WATCHDOG_AUTO_FEED_TIME)
                              .count()
                       : 0));
    }

protected:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr watchdog_status_pub;
    rclcpp::Service<SetRobotMode>::SharedPtr robot_state_service;
    rclcpp::TimerBase::SharedPtr watchdog_timer;

    int robot_mode{0};
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusServer>());
    rclcpp::shutdown();

    return 0;
}
