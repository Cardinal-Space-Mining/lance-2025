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

#include "localization_controller.hpp"

#include <cmath>
// #include <iostream>

#include <Eigen/Core>


#define PERCEPTION_REFLECTOR_HINT_TOPIC "/cardinal_perception/reflector_hint"
#define PERCEPTION_LFD_CONTROL_TOPIC    "/cardinal_perception/set_global_alignment"

using Vec2f = Eigen::Vector2f;


LocalizationController::LocalizationController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    const Tf2Buffer& tf_buffer) :
    pub_map{pub_map},
    params{params},
    tf_buffer{tf_buffer},
    hint_sub{node.create_subscription<ReflectorHintMsg>(
        PERCEPTION_REFLECTOR_HINT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const ReflectorHintMsg::ConstSharedPtr& msg)
        { this->last_hint = msg; })},
    lfd_control_client{
        node.create_client<SetBoolSrv>(PERCEPTION_LFD_CONTROL_TOPIC)}
{
}

void LocalizationController::initialize()
{
    this->stage = Stage::INITIALIZATION;
    this->last_hint = nullptr;
}

bool LocalizationController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void LocalizationController::setCancelled()
{
    this->stage = Stage::FINISHED;
    this->setLfdControl(false);
}

void LocalizationController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    // if at any point the full localization transform is established,
    // the command is finished
    // TODO: parameterize frame ids
    if (this->tf_buffer.canTransform("base_link", "map", rclcpp::Time{}))
    {
        this->stage = Stage::FINISHED;
    }

    commands.disableAll();

    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            this->setLfdControl(true);
            this->stage = Stage::SEARCHING;
            [[fallthrough]];
        }
        case Stage::SEARCHING:
        {
            constexpr double SEARCHING_TRACKS_VEL = 20.;

            if (!this->last_hint || this->last_hint->samples < 100U)
            {
                commands.setTracksVelocity(
                    -SEARCHING_TRACKS_VEL,
                    SEARCHING_TRACKS_VEL);
                break;
            }

            this->stage = Stage::TARGETTING;
            [[fallthrough]];
        }
        case Stage::TARGETTING:
        {
            constexpr double TARGETTING_TRACKS_VEL = 10.;
            constexpr float TARGETTING_HEADING_THRESH_DEG = 2.f;

            const auto& pt = this->last_hint->centroid;
            const Vec2f rel{
                static_cast<float>(pt.point.x),
                static_cast<float>(pt.point.y)};
            const Vec2f ref{0.f, 1.f};

            const float sin_heading = rel.dot(ref.normalized());
            if (std::abs(sin_heading) > std::sin(
                                            std::numbers::pi_v<float> / 180.f *
                                            TARGETTING_HEADING_THRESH_DEG))
            {
                if (sin_heading > 0.f)
                {
                    commands.setTracksVelocity(
                        -TARGETTING_TRACKS_VEL,
                        TARGETTING_TRACKS_VEL);
                }
                else
                {
                    commands.setTracksVelocity(
                        TARGETTING_TRACKS_VEL,
                        -TARGETTING_TRACKS_VEL);
                }
            }

            break;
        }
        case Stage::FINISHED:
        {
            this->setLfdControl(false);
        }
    }

    // constexpr char const* STRS[] = {"init", "searching", "targetting", "finished"};
    // std::cout << STRS[static_cast<size_t>(this->stage)] << std::endl;
}

void LocalizationController::setLfdControl(bool enabled)
{
    auto req = std::make_shared<SetBoolSrv::Request>();
    req->data = enabled;
    this->lfd_control_client->async_send_request(
        req,
        [](rclcpp::Client<SetBoolSrv>::SharedFuture) {});
}
