#include "util.hpp"
#include "imu_transform.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>


class ImuTf : public rclcpp::Node
{
public:
    ImuTf() :
        rclcpp::Node{ "imu_transformer" },
        tf_buffer{ std::make_shared<rclcpp::Clock>(rclcpp::Clock{ RCL_ROS_TIME }) },
        tf_listener{ tf_buffer }
    {
        util::declare_param(this, "target_frame", this->target_frame, "base_link");
        util::declare_param(this, "override_frame", this->override_frame, "");

        this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("transformed_imu", rclcpp::SensorDataQoS{});
        this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "input_imu", rclcpp::SensorDataQoS{},
            [this](const typename sensor_msgs::msg::Imu::SharedPtr msg)
            {
                try
                {
                    sensor_msgs::msg::Imu tf_scan;
                    auto tf = this->tf_buffer.lookupTransform(
                        this->target_frame,
                        msg->header.frame_id,
                        util::toTf2TimePoint(msg->header.stamp));

                    tf2::doTransform(*msg, tf_scan, tf);

                    if(!this->override_frame.empty())
                    {
                        tf_scan.header.frame_id = this->override_frame;
                    }
                    this->imu_pub->publish(tf_scan);
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "[IMU TF]: Failed to transform - what():\n\t%s", e.what());
                }
            }
        );
    }

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    std::string target_frame, override_frame;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuTf>());
    rclcpp::shutdown();

    return 0;
}
