///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2024, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_SUBSCRIBER_H_
#define CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_SUBSCRIBER_H_

#include "crocoddyl_msgs/conversions.h"

#include <mutex>
#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#else
#include <ros/node_handle.h>
#endif

namespace crocoddyl_msgs {

#ifdef ROS2
typedef const crocoddyl_msgs::msg::MultibodyInertialParameters::SharedPtr
    MultibodyInertialParametersSharedPtr;
#else
typedef const crocoddyl_msgs::MultibodyInertialParameters::ConstPtr
    &MultibodyInertialParametersSharedPtr;
#endif

class MultibodyInertialParametersRosSubscriber {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialize the multi-body inertial parameters subscriber.
   *
   * @param[in] topic  Topic name (default: "/crocoddyl/inertial_parameters")
   */
  MultibodyInertialParametersRosSubscriber(
      const std::string &topic = "/crocoddyl/inertial_parameters")
#ifdef ROS2
      : node_(rclcpp::Node::make_shared("inertia_parameters_subscriber")),
        sub_(node_->create_subscription<MultibodyInertialParameters>(
            topic, 1,
            std::bind(&MultibodyInertialParametersRosSubscriber::callback, this,
                      std::placeholders::_1))),
        has_new_msg_(false), is_processing_msg_(false), last_msg_time_(0.) {
    spinner_.add_node(node_);
    thread_ = std::thread([this]() { this->spin(); });
    thread_.detach();
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Subscribing MultibodyInertialParameters messages on "
                           << topic);
#else
      : spinner_(2), has_new_msg_(false), is_processing_msg_(false),
        last_msg_time_(0.) {
    ros::NodeHandle n;
    sub_ = n.subscribe<MultibodyInertialParameters>(
        topic, 1, &MultibodyInertialParametersRosSubscriber::callback, this,
        ros::TransportHints().tcpNoDelay());
    spinner_.start();
    ROS_INFO_STREAM("Subscribing MultibodyInertialParameters messages on "
                    << topic);
#endif
  }

  ~MultibodyInertialParametersRosSubscriber() = default;

  /**
   * @brief Get the latest inertial parameters of all bodies
   *
   * The inertial parameters vector is defined as [m, h_x, h_y, h_z,
   * I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T, where h=mc is
   * the first moment of inertial (mass * barycenter) and the rotational
   * inertia I = I_C + mS^T(c)S(c) where I_C has its origin at the
   * barycenter.
   * 
   * @return  A map from body names to inertial parameters.   *
   */
  std::map<std::string, Vector10d> get_parameters() {
    // start processing the message
    is_processing_msg_ = true;
    std::lock_guard<std::mutex> guard(mutex_);

    const std::size_t n_bodies = msg_.parameters.size();
    for (std::size_t i = 0; i < n_bodies; ++i) {
      const std::string &body_name = msg_.parameters[i].name;
      fromMsg(msg_.parameters[i], psi_tmp_);
      parameters_[body_name] = psi_tmp_;
    }
    // finish processing the message
    is_processing_msg_ = false;
    has_new_msg_ = false;
    return parameters_;
  }

  /**
   * @brief Indicate whether we have received a new message
   */
  bool has_new_msg() const { return has_new_msg_; }

private:
#ifdef ROS2
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor spinner_;
  std::thread thread_;
  void spin() { spinner_.spin(); }
  rclcpp::Subscription<MultibodyInertialParameters>::SharedPtr
      sub_; //!< ROS subscriber
#else
  ros::NodeHandle node_;
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_; //!< ROS subscriber
#endif
  std::mutex mutex_; ///< Mutex to prevent race condition on callback
  MultibodyInertialParameters msg_; //!< ROS message

  bool has_new_msg_;       //!< Indcate when a new message has been received
  bool is_processing_msg_; //!< Indicate when we are processing the message
  double last_msg_time_;

  Vector10d psi_tmp_;
  std::map<std::string, Vector10d> parameters_;

  void callback(MultibodyInertialParametersSharedPtr msg) {
    if (!is_processing_msg_) {
#ifdef ROS2
      double t = rclcpp::Time(msg->header.stamp).seconds();
#else
      double t = msg->header.stamp.toNSec();
#endif
      // Avoid out of order arrival and ensure each message is newer (or equal
      // to) than the preceeding:
      if (last_msg_time_ <= t) {
        std::lock_guard<std::mutex> guard(mutex_);
        msg_ = *msg;
        has_new_msg_ = true;
        last_msg_time_ = t;
      } else {
#ifdef ROS2
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "Out of order message. Last timestamp: "
                               << std::fixed << last_msg_time_
                               << ", current timestamp: " << t);
#else
        ROS_WARN_STREAM("Out of order message. Last timestamp: "
                        << std::fixed << last_msg_time_
                        << ", current timestamp: " << t);
#endif
      }
    }
  }
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_SUBSCRIBER_H_
