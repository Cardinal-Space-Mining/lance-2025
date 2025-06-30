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

#pragma once

#include <csm_metrics/profiling_config.hpp>

#include <rclcpp/rclcpp.hpp>


#ifndef PROFILING_MODE
    #define PROFILING_MODE PROFILING_MODE_ALL
#endif
#ifndef PROFILING_DEFAULT_BUFFER_SIZE
    #define PROFILING_DEFAULT_BUFFER_SIZE 20
#endif

namespace csm
{
namespace metrics
{
namespace profiling
{

void init(
    rclcpp::Node& node,
    const char* topic = PROFILING_DEFAULT_TOPIC,
    const rclcpp::QoS& qos = PROFILING_DEFAULT_QOS);
void deinit();
void flush();
void flush_local();
void notify_unbuffered(const char* label);
void notify2_unbuffered(const char* label1, const char* label2);
void notify(
    const char* label,
    size_t buffering = PROFILING_DEFAULT_BUFFER_SIZE);
void notify2(
    const char* label1,
    const char* label2,
    size_t buffering = PROFILING_DEFAULT_BUFFER_SIZE);

};  // namespace profiling
};  // namespace metrics
};  // namespace csm

#if PROFILING_MODE >= PROFILING_MODE_ALL
    #define PROFILING_NOTIFY(label, ...)                                   \
        csm::metrics::profiling::notify(#label __VA_OPT__(, ) __VA_ARGS__)
    #define PROFILING_NOTIFY2(label1, label2, ...) \
        csm::metrics::profiling::notify2(          \
            #label1,                               \
            #label2 __VA_OPT__(, ) __VA_ARGS__)
    #define PROFILING_NOTIFY_UNBUFFERED(label)             \
        csm::metrics::profiling::notify_unbuffered(#label)
    #define PROFILING_NOTIFY2_UNBUFFERED(label1, label2)              \
        csm::metrics::profiling::notify2_unbuffered(#label1, #label2)
#else
    #define PROFILING_NOTIFY(...)
    #define PROFILING_NOTIFY2(...)
    #define PROFILING_NOTIFY_UNBUFFERED(...)
    #define PROFILING_NOTIFY2_UNBUFFERED(...)
#endif
#if PROFILING_MODE >= PROFILING_MODE_LIMITED
    #define PROFILING_INIT(node, ...)                                  \
        csm::metrics::profiling::init(node __VA_OPT__(, ) __VA_ARGS__)
    #define PROFILING_DEINIT()      csm::metrics::profiling::deinit()
    #define PROFILING_FLUSH()       csm::metrics::profiling::flush()
    #define PROFILING_FLUSH_LOCAL() csm::metrics::profiling::flush_local()
    #define PROFILING_NOTIFY_ALWAYS(label, ...)                            \
        csm::metrics::profiling::notify(#label __VA_OPT__(, ) __VA_ARGS__)
    #define PROFILING_NOTIFY2_ALWAYS(label1, label2, ...) \
        csm::metrics::profiling::notify2(                 \
            #label1,                                      \
            #label2 __VA_OPT__(, ) __VA_ARGS__)
    #define PROFILING_NOTIFY_UNBUFFERED_ALWAYS(label)      \
        csm::metrics::profiling::notify_unbuffered(#label)
    #define PROFILING_NOTIFY2_UNBUFFERED_ALWAYS(label1, label2)       \
        csm::metrics::profiling::notify2_unbuffered(#label1, #label2)
#else
    #define PROFILING_INIT(...)
    #define PROFILING_DEINIT(...)
    #define PROFILING_FLUSH(...)
    #define PROFILING_FLUSH_LOCAL(...)
    #define PROFILING_NOTIFY_ALWAYS(...)
    #define PROFILING_NOTIFY2_ALWAYS(...)
    #define PROFILING_NOTIFY_UNBUFFERED_ALWAYS(...)
    #define PROFILING_NOTIFY2_UNBUFFERED_ALWAYS(...)
#endif
