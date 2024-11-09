#include "util.hpp"
// #include "imu_transform.hpp"
#include "geometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


class ImuViz : public rclcpp::Node
{
public:
    ImuViz() :
        rclcpp::Node{ "imu_visualizer" },
        tf_buffer{ std::make_shared<rclcpp::Clock>(rclcpp::Clock{ RCL_ROS_TIME }) },
        tf_listener{ tf_buffer }
    {
        std::string topic;
        util::declare_param(this, "imu_topic", topic, "/imu");

        this->orient_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic + "/orientation", rclcpp::SensorDataQoS{});
        this->angular_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic + "/angular_velocity", rclcpp::SensorDataQoS{});
        this->linear_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic + "/linear_acceleration", rclcpp::SensorDataQoS{});
        this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            topic, rclcpp::SensorDataQoS{},
            [this](const typename sensor_msgs::msg::Imu::SharedPtr msg)
            {
                using namespace util::geom::cvt::ops;
                geometry_msgs::msg::PoseStamped _orient, _angular, _linear;

                Eigen::Quaterniond q;
                _orient.pose.orientation << (msg->orientation >> q).inverse();
                _orient.header = msg->header;

                Eigen::Vector3d _av, _la;
                _av << msg->angular_velocity;
                _la << msg->linear_acceleration;

                Eigen::Quaterniond _avq, _laq;
                _avq.setFromTwoVectors(Eigen::Vector3d::Zero(), _av);
                _laq.setFromTwoVectors(Eigen::Vector3d::Zero(), _la);

                _angular.pose.orientation << _avq;
                _angular.header = msg->header;

                _linear.pose.orientation << _laq;
                _linear.header = msg->header;

                this->orient_pose_pub->publish(_orient);
                this->angular_pose_pub->publish(_angular);
                this->linear_pose_pub->publish(_linear);
            }
        );
    }

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr orient_pose_pub, angular_pose_pub, linear_pose_pub;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuViz>());
    rclcpp::shutdown();

    return 0;
}
