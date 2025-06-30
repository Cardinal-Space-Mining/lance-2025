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

#include <csm_metrics/profiling.hpp>

#include <mutex>
#include <chrono>
#include <thread>
#include <vector>

#include <csm_metrics/msg/trace_notification.hpp>
#include <csm_metrics/msg/trace_notifications.hpp>


#ifndef BUFFERING_USE_LOCAL
    #define BUFFERING_USE_LOCAL 1
#endif
#ifndef FORCE_DISABLE_BUFFERING
    #define FORCE_DISABLE_BUFFERING 0
#endif

#define USING_LOCAL_BUFFER ((BUFFERING_USE_LOCAL) && !(FORCE_DISABLE_BUFFERING))
#define USING_SHARED_BUFFER                                \
    (!(BUFFERING_USE_LOCAL) && !(FORCE_DISABLE_BUFFERING))

namespace csm
{
namespace metrics
{
namespace profiling
{

using TraceNotificationMsg = csm_metrics::msg::TraceNotification;
using TraceNotificationsMsg = csm_metrics::msg::TraceNotifications;

static rclcpp::Publisher<TraceNotificationsMsg>::SharedPtr notif_pub{nullptr};
static std::hash<std::thread::id> thread_id_hash;
#if USING_LOCAL_BUFFER
thread_local TraceNotificationsMsg notif_buffer;
#elif USING_SHARED_BUFFER
static TraceNotificationsMsg notif_buffer;
static std::mutex buff_mtx;
#endif


inline uint64_t getNs()
{
    return (std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count());
}
inline uint64_t getId() { return thread_id_hash(std::this_thread::get_id()); }

inline void fillMsg(
    TraceNotificationMsg& msg,
    const char* label,
    uint64_t ns = getNs(),
    uint64_t id = getId())
{
    msg.label = label;
    msg.ns_since_epoch = ns;
    msg.thread_id = id;
}


void init(rclcpp::Node& node, const char* topic, const rclcpp::QoS& qos)
{
    notif_pub = node.create_publisher<TraceNotificationsMsg>(topic, qos);
}
void deinit() { notif_pub.reset(); }

void flush()
{
#if USING_SHARED_BUFFER
    std::unique_lock<std::mutex> lock{buff_mtx};
#endif
#if USING_SHARED_BUFFER || USING_LOCAL_BUFFER
    notif_pub->publish(notif_buffer);
    notif_buffer.notifications.clear();
#endif
}
void flush_local()
{
#if USING_LOCAL_BUFFER
    flush();
#endif
}

void notify_unbuffered(const char* label)
{
    TraceNotificationsMsg msg;
    fillMsg(msg.notifications.emplace_back(), label);
    notif_pub->publish(msg);
}
void notify2_unbuffered(const char* label1, const char* label2)
{
    TraceNotificationsMsg msg;
    msg.notifications.resize(2);
    const uint64_t ns = getNs();
    const uint64_t id = getId();

    fillMsg(msg.notifications[0], label1, ns, id);
    fillMsg(msg.notifications[1], label2, ns, id);
    notif_pub->publish(msg);
}

void notify(const char* label, size_t buffering)
{
#if USING_SHARED_BUFFER
    std::unique_lock<std::mutex> lock{buff_mtx};
#endif
#if USING_SHARED_BUFFER || USING_LOCAL_BUFFER
    fillMsg(notif_buffer.notifications.emplace_back(), label);
    if (notif_buffer.notifications.size() >= buffering)
    {
        notif_pub->publish(notif_buffer);
        notif_buffer.notifications.clear();
    }
#else
    notify_unbuffered(label);
#endif
}
void notify2(const char* label1, const char* label2, size_t buffering)
{
#if USING_SHARED_BUFFER
    std::unique_lock<std::mutex> lock{buff_mtx};
#endif
#if USING_SHARED_BUFFER || USING_LOCAL_BUFFER
    const uint64_t ns = getNs();
    const uint64_t id = getId();
    fillMsg(notif_buffer.notifications.emplace_back(), label1, ns, id);
    fillMsg(notif_buffer.notifications.emplace_back(), label2, ns, id);
    if (notif_buffer.notifications.size() >= buffering)
    {
        notif_pub->publish(notif_buffer);
        notif_buffer.notifications.clear();
    }
#else
    notify2_unbuffered(label1, label2);
#endif
}

};  // namespace profiling
};  // namespace metrics
};  // namespace csm
