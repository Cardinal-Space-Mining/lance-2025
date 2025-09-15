#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <phoenix_ros_driver/msg/talon_ctrl.hpp>

using phoenix_ros_driver::msg::TalonCtrl;

class TwistToTalon : public rclcpp::Node
{
public:
    TwistToTalon() :
        Node("twistToTalon"),
        drive_right_pub(
            this->create_publisher<TalonCtrl>("track_right_ctrl_auto", 10)),
        drive_left_pub(
            this->create_publisher<TalonCtrl>("track_left_ctrl_auto", 10)),
        drive_sub(this->create_subscription<geometry_msgs::msg::Twist>(
            "drive_twist",
            10,
            [this](const geometry_msgs::msg::Twist& msg)
            { this->translate(msg); }))
    {
    }

    void translate(const geometry_msgs::msg::Twist& msg)
    {
        auto linear = msg.linear.x;
        auto angular = msg.angular.z;

        auto left = linear - (angular * TRACK_WIDTH / 2.0);
        auto right = linear + (angular * TRACK_WIDTH / 2.0);

        TalonCtrl track_right;
        track_right.value = right;
        drive_right_pub->publish(track_right);

        TalonCtrl track_left;
        track_left.value = left;
        drive_left_pub->publish(track_left);
    }

private:
    rclcpp::Publisher<TalonCtrl>::SharedPtr drive_right_pub;
    rclcpp::Publisher<TalonCtrl>::SharedPtr drive_left_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_sub;

    const float TRACK_WIDTH = 0.5334;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TwistToTalon>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
