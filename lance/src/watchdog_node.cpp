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
#include <std_msgs/msg/int32.hpp>

#include "lance/srv/set_robot_mode.hpp"

#include "util/ros_utils.hpp"


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace util::ros_aliases;


#define WATCHDOG_PUB_DT           100ms
#define WATCHDOG_TELEOP_FEED_TIME 250ms
#define WATCHDOG_AUTO_FEED_TIME   10000ms

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic


class RobotStatusServer : public rclcpp::Node
{
    using Int32Msg = std_msgs::msg::Int32;
    using SetRobotModeSrv = lance::srv::SetRobotMode;

public:
    RobotStatusServer() :
        Node("robot_status"),

        watchdog_status_pub{this->create_publisher<Int32Msg>(
            ROBOT_TOPIC("watchdog_status"),
            rclcpp::SensorDataQoS{})},
        robot_state_service{this->create_service<SetRobotModeSrv>(
            ROBOT_TOPIC("set_robot_mode"),
            [this](
                SetRobotModeSrv::Request::SharedPtr req,
                SetRobotModeSrv::Response::SharedPtr resp)
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
                    Int32Msg{}.set__data(this->getFeedTime()));
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
    SharedPub<Int32Msg> watchdog_status_pub;
    SharedSrv<SetRobotModeSrv> robot_state_service;
    RclTimer watchdog_timer;

    int robot_mode{0};
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusServer>());
    rclcpp::shutdown();

    return 0;
}
