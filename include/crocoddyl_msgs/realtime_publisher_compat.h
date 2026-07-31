///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Compatibility adapter for realtime_tools 5.x (ROS 2 Lyrical and newer).
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSGS__REALTIME_PUBLISHER_COMPAT_H_
#define CROCODDYL_MSGS__REALTIME_PUBLISHER_COMPAT_H_

#ifdef ROS2

#include <memory>
#include <mutex>

#include <realtime_tools/realtime_publisher.hpp>

namespace crocoddyl_msgs {

// crocoddyl_msgs historically used the ROS 1-style RealtimePublisher API
// (msg_, trylock(), unlockAndPublish()).  realtime_tools 5.x deliberately
// replaced that API with try_publish(message).  Keep the public
// crocoddyl_msgs API stable while delegating publication to the new
// non-realtime publishing thread.
template <class MessageT>
class RealtimePublisherCompat {
public:
  using PublisherSharedPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;

  explicit RealtimePublisherCompat(PublisherSharedPtr publisher)
      : publisher_(std::move(publisher)) {}

  bool trylock() { return message_mutex_.try_lock(); }

  void unlockAndPublish() {
    publisher_.try_publish(msg_);
    message_mutex_.unlock();
  }

  MessageT msg_;

private:
  realtime_tools::RealtimePublisher<MessageT> publisher_;
  std::mutex message_mutex_;
};

} // namespace crocoddyl_msgs

#else

#include <realtime_tools/realtime_publisher.h>

namespace crocoddyl_msgs {

template <class MessageT>
using RealtimePublisherCompat = realtime_tools::RealtimePublisher<MessageT>;

} // namespace crocoddyl_msgs

#endif

#endif // CROCODDYL_MSGS__REALTIME_PUBLISHER_COMPAT_H_
