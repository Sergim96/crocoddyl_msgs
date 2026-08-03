///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2024, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_WHOLE_BODY_STATE_PUBLISHER_H_
#define CROCODDYL_MSG_WHOLE_BODY_STATE_PUBLISHER_H_

#include "crocoddyl_msgs/conversions.h"

#include "crocoddyl_msgs/realtime_publisher_compat.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#else
#include <ros/node_handle.h>
#endif

namespace crocoddyl_msgs {

static std::map<std::string, pinocchio::SE3> DEFAULT_SE3;

static std::map<std::string, pinocchio::Motion> DEFAULT_MOTION;

static std::map<std::string,
                std::tuple<pinocchio::Force, ContactType, ContactStatus>>
    DEFAULT_FORCE;

static std::map<std::string, std::pair<Eigen::Vector3d, double>>
    DEFAULT_FRICTION;

class WholeBodyStateRosPublisher {
public:
  /**
   * @brief Initialize the whole-body state publisher.
   *
   * @param[in] model  Pinocchio model
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
  WholeBodyStateRosPublisher(
      pinocchio::Model &model,
      const std::string &topic = "/crocoddyl/whole_body_state",
      const std::string &frame = "odom")
#ifdef ROS2
      : node_("whole_body_state_publisher"),
        pub_(node_.create_publisher<WholeBodyState>(topic, 1)), model_(model),
        data_(model), odom_frame_(frame), a_(model.nv),
        is_reduced_model_(false) {
    RCLCPP_INFO_STREAM(node_.get_logger(),
                       "Publishing WholeBodyState messages on "
                           << topic << " (frame: " << frame << ")");
#else
      : model_(model), data_(model), odom_frame_(frame), a_(model.nv),
        is_reduced_model_(false) {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
    ROS_INFO_STREAM("Publishing WholeBodyState messages on "
                    << topic << " (frame: " << frame << ")");
#endif
    pub_.msg_.header.frame_id = frame;
    init();
  }

  /**
   * @brief Initialize the whole-body state publisher for rigid-body system
   * with locked joints.
   *
   * @param[in] model          Pinocchio model
   * @param[in] locked_joints  List of joints to be locked
   * @param[in] qref           Reference configuration
   * @param[in] topic          Topic name
   * @param[in] frame          Odometry frame
   */
  WholeBodyStateRosPublisher(
      pinocchio::Model &model, const std::vector<std::string> &locked_joints,
      const Eigen::Ref<const Eigen::VectorXd> &qref,
      const std::string &topic = "/crocoddyl/whole_body_state",
      const std::string &frame = "odom")
#ifdef ROS2
      : node_("whole_body_state_publisher"),
        pub_(node_.create_publisher<WholeBodyState>(topic, 1)), model_(model),
        data_(model), odom_frame_(frame), a_(model.nv), qref_(qref),
        is_reduced_model_(true) {
    RCLCPP_INFO_STREAM(node_.get_logger(),
                       "Publishing WholeBodyState messages on "
                           << topic << " (frame: " << frame << ")");
#else
      : model_(model), data_(model_), odom_frame_(frame), a_(model.nv),
        qref_(qref), is_reduced_model_(true) {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
    ROS_INFO_STREAM("Publishing WholeBodyState messages on "
                    << topic << " (frame: " << frame << ")");
#endif
    pub_.msg_.header.frame_id = frame;
    init(locked_joints);
  }
  ~WholeBodyStateRosPublisher() {
#ifdef ROS2
    stop_async_publishing();
#endif
  }

#ifdef ROS2
  /**
   * @brief Move Pinocchio conversion and ROS-message construction off the
   * caller thread using a bounded, preallocated SPSC queue.
   *
   * The producer still snapshots every input synchronously. If the worker
   * falls a complete queue behind, the newest sample is dropped, matching the
   * non-blocking behavior of RealtimePublisher::try_publish().
   */
  void enable_async_publishing(const std::vector<std::string> &contact_names,
                               const std::size_t capacity = 64) {
    if (async_enabled_.load(std::memory_order_acquire)) {
      return;
    }
    if (is_reduced_model_) {
      throw std::logic_error(
          "Asynchronous whole-body publication does not support reduced models");
    }
    if (capacity < 2) {
      throw std::invalid_argument(
          "Asynchronous whole-body publication capacity must be at least two");
    }

    AsyncSnapshot prototype;
    prototype.q = Eigen::VectorXd::Zero(model_.nq);
    prototype.v = Eigen::VectorXd::Zero(model_.nv);
    prototype.a = Eigen::VectorXd::Zero(model_.nv);
    prototype.tau =
        Eigen::VectorXd::Zero(model_.nv - getRootNv(model_));
    for (const std::string &name : contact_names) {
      prototype.contact_position.emplace(name, pinocchio::SE3::Identity());
      prototype.contact_velocity.emplace(name, pinocchio::Motion::Zero());
      prototype.contact_force.emplace(
          name, std::make_tuple(pinocchio::Force::Zero(), LOCOMOTION,
                                SEPARATION));
      prototype.contact_surface.emplace(
          name, std::make_pair(Eigen::Vector3d::UnitZ(), 0.0));
    }
    async_queue_.assign(capacity, prototype);
    async_read_sequence_.store(0, std::memory_order_relaxed);
    async_write_sequence_.store(0, std::memory_order_relaxed);
    async_dropped_samples_.store(0, std::memory_order_relaxed);
    async_stop_.store(false, std::memory_order_relaxed);
    async_enabled_.store(true, std::memory_order_release);
    async_thread_ = std::thread([this]() { async_publish_loop(); });
  }

  std::uint64_t get_async_dropped_samples() const {
    return async_dropped_samples_.load(std::memory_order_relaxed);
  }
#endif

  /**
   * @brief Publish a whole-body state ROS message.
   * The dimension of the configuration, velocity and joint effort are defined
   * by the rigid-body system with locked joints.
   *
   * @param t[in]    Time in secs
   * @param q[in]    Configuration vector (dimension: model.nq)
   * @param v[in]    Generalized velocity (dimension: model.nv)
   * @param tau[in]  Joint effort (dimension: model.nv)
   * @param p[in]    Contact position
   * @param pd[in]   Contact velocity
   * @param f[in]    Contact force, type and status
   * @param s[in]    Contact surface and friction coefficient
   */
  void publish(
      const double t, const Eigen::Ref<const Eigen::VectorXd> &q,
      const Eigen::Ref<const Eigen::VectorXd> &v,
      const Eigen::Ref<const Eigen::VectorXd> &tau,
      const std::map<std::string, pinocchio::SE3> &p = DEFAULT_SE3,
      const std::map<std::string, pinocchio::Motion> &pd = DEFAULT_MOTION,
      const std::map<std::string, std::tuple<pinocchio::Force, ContactType,
                                             ContactStatus>> &f = DEFAULT_FORCE,
      const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s =
          DEFAULT_FRICTION) {
#ifdef ROS2
    if (async_enabled_.load(std::memory_order_acquire)) {
      enqueue_async(t, q, v, a_, tau, p, pd, f, s);
      return;
    }
#endif
    publish_synchronously_without_acceleration(t, q, v, tau, p, pd, f, s);
  }

  /**
   * @brief Publish a whole-body state ROS message
   *
   * @param t[in]    Time in secs
   * @param q[in]    Configuration vector (dimension: model.nq)
   * @param v[in]    Generalized velocity (dimension: model.nv)
   * @param a[in]    Generalized acceleration (dimension: model.nv)
   * @param tau[in]  Joint effort (dimension: model.nv)
   * @param p[in]    Contact position
   * @param pd[in]   Contact velocity
   * @param f[in]    Contact force, type and status
   * @param s[in]    Contact surface and friction coefficient
   */
  void publish(
      const double t, const Eigen::Ref<const Eigen::VectorXd> &q,
      const Eigen::Ref<const Eigen::VectorXd> &v,
      const Eigen::Ref<const Eigen::VectorXd> &a,
      const Eigen::Ref<const Eigen::VectorXd> &tau,
      const std::map<std::string, pinocchio::SE3> &p = DEFAULT_SE3,
      const std::map<std::string, pinocchio::Motion> &pd = DEFAULT_MOTION,
      const std::map<std::string, std::tuple<pinocchio::Force, ContactType,
                                             ContactStatus>> &f = DEFAULT_FORCE,
      const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s =
          DEFAULT_FRICTION) {
#ifdef ROS2
    if (async_enabled_.load(std::memory_order_acquire)) {
      enqueue_async(t, q, v, a, tau, p, pd, f, s);
      return;
    }
#endif
    publish_synchronously(t, q, v, a, tau, p, pd, f, s);
  }

  /**
   * @brief Update the Pinocchio model's inertial parameters of a given body
   * frame
   *
   * The inertial parameters vector is defined as [m, h_x, h_y, h_z,
   * I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T, where h=mc is
   * the first moment of inertial (mass * barycenter) and the rotational
   * inertia I = I_C + mS^T(c)S(c) where I_C has its origin at the
   * barycenter. Additionally, the type of frame supported are joints,
   * fixed joints, and bodies.
   *
   * @param model[in]      Pinocchio model
   * @param body_name[in] Body name
   * @param psi[in]        Inertial parameters
   */
  void update_body_inertial_parameters(const std::string &body_name,
                                       const Eigen::Ref<const Vector10d> &psi) {
#ifdef ROS2
    std::lock_guard<std::mutex> guard(model_mutex_);
#endif
    updateBodyInertialParameters(model_, body_name, psi);
    if (is_reduced_model_)
      updateBodyInertialParameters(reduced_model_, body_name, psi);
  }

  /**
   * @brief Return the Pinocchio model's inertial parameters of a given body
   * frame
   *
   * The inertial parameters vector is defined as [m, h_x, h_y, h_z,
   * I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T, where h=mc is
   * the first moment of inertial (mass * barycenter) and the rotational
   * inertia I = I_C + mS^T(c)S(c) where I_C has its origin at the
   * barycenter.
   *
   * @param body_name[in]  Body name
   */
  const Vector10d
  get_body_inertial_parameters(const std::string &body_name) const {
#ifdef ROS2
    std::lock_guard<std::mutex> guard(model_mutex_);
#endif
    return getBodyInertialParameters(model_, body_name);
  }

private:
#ifdef ROS2
  rclcpp::Node node_;
#endif
  RealtimePublisherCompat<WholeBodyState> pub_;
  pinocchio::Model model_;
  pinocchio::Model reduced_model_;
  pinocchio::Data data_;
  std::string odom_frame_;
  Eigen::VectorXd a_;
  std::vector<pinocchio::JointIndex> joint_ids_;
  Eigen::VectorXd qref_;
  Eigen::VectorXd qfull_;
  Eigen::VectorXd vfull_;
  Eigen::VectorXd afull_;
  Eigen::VectorXd ufull_;
  bool is_reduced_model_;
  pinocchio::Inertia inertia_tmp_;

#ifdef ROS2
  struct AsyncSnapshot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double time{0.0};
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::VectorXd a;
    Eigen::VectorXd tau;
    std::map<std::string, pinocchio::SE3> contact_position;
    std::map<std::string, pinocchio::Motion> contact_velocity;
    std::map<std::string,
             std::tuple<pinocchio::Force, ContactType, ContactStatus>>
        contact_force;
    std::map<std::string, std::pair<Eigen::Vector3d, double>> contact_surface;
  };

  std::vector<AsyncSnapshot, Eigen::aligned_allocator<AsyncSnapshot>>
      async_queue_;
  std::atomic<std::uint64_t> async_read_sequence_{0};
  std::atomic<std::uint64_t> async_write_sequence_{0};
  std::atomic<std::uint64_t> async_dropped_samples_{0};
  std::atomic_bool async_enabled_{false};
  std::atomic_bool async_stop_{false};
  std::thread async_thread_;
  mutable std::mutex model_mutex_;
#endif

  void publish_synchronously(
      const double t, const Eigen::Ref<const Eigen::VectorXd> &q,
      const Eigen::Ref<const Eigen::VectorXd> &v,
      const Eigen::Ref<const Eigen::VectorXd> &a,
      const Eigen::Ref<const Eigen::VectorXd> &tau,
      const std::map<std::string, pinocchio::SE3> &p,
      const std::map<std::string, pinocchio::Motion> &pd,
      const std::map<std::string,
                     std::tuple<pinocchio::Force, ContactType, ContactStatus>>
          &f,
      const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
#ifdef ROS2
    std::lock_guard<std::mutex> model_guard(model_mutex_);
#endif
    if (!pub_.trylock()) {
      return;
    }
    pub_.msg_.header.frame_id = odom_frame_;
    if (is_reduced_model_) {
      fromReduced(model_, reduced_model_, qfull_, vfull_, afull_, ufull_, q, v,
                  a, tau, qref_, joint_ids_);
      crocoddyl_msgs::toMsg(model_, data_, pub_.msg_, t, qfull_, vfull_,
                            afull_, ufull_, p, pd, f, s);
    } else {
      crocoddyl_msgs::toMsg(model_, data_, pub_.msg_, t, q, v, a, tau, p, pd,
                            f, s);
    }
    pub_.unlockAndPublish();
  }

  void publish_synchronously_without_acceleration(
      const double t, const Eigen::Ref<const Eigen::VectorXd> &q,
      const Eigen::Ref<const Eigen::VectorXd> &v,
      const Eigen::Ref<const Eigen::VectorXd> &tau,
      const std::map<std::string, pinocchio::SE3> &p,
      const std::map<std::string, pinocchio::Motion> &pd,
      const std::map<std::string,
                     std::tuple<pinocchio::Force, ContactType, ContactStatus>>
          &f,
      const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
#ifdef ROS2
    std::lock_guard<std::mutex> model_guard(model_mutex_);
#endif
    if (!pub_.trylock()) {
      return;
    }
    pub_.msg_.header.frame_id = odom_frame_;
    if (is_reduced_model_) {
      fromReduced(model_, reduced_model_, qfull_, vfull_, ufull_, q, v, tau,
                  qref_, joint_ids_);
      crocoddyl_msgs::toMsg(model_, data_, pub_.msg_, t, qfull_, vfull_, a_,
                            ufull_, p, pd, f, s);
    } else {
      crocoddyl_msgs::toMsg(model_, data_, pub_.msg_, t, q, v, a_, tau, p, pd,
                            f, s);
    }
    pub_.unlockAndPublish();
  }

#ifdef ROS2
  template <typename MapT>
  static void copy_contact_values(MapT &destination, const MapT &source,
                                  const char *label) {
    if (destination.size() != source.size()) {
      throw std::invalid_argument(std::string("Unexpected ") + label +
                                  " contact count");
    }
    for (auto &item : destination) {
      const auto source_item = source.find(item.first);
      if (source_item == source.end()) {
        throw std::invalid_argument(std::string("Missing ") + label +
                                    " contact '" + item.first + "'");
      }
      item.second = source_item->second;
    }
  }

  void enqueue_async(
      const double t, const Eigen::Ref<const Eigen::VectorXd> &q,
      const Eigen::Ref<const Eigen::VectorXd> &v,
      const Eigen::Ref<const Eigen::VectorXd> &a,
      const Eigen::Ref<const Eigen::VectorXd> &tau,
      const std::map<std::string, pinocchio::SE3> &p,
      const std::map<std::string, pinocchio::Motion> &pd,
      const std::map<std::string,
                     std::tuple<pinocchio::Force, ContactType, ContactStatus>>
          &f,
      const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
    if (q.size() != model_.nq || v.size() != model_.nv ||
        a.size() != model_.nv ||
        tau.size() != model_.nv - static_cast<Eigen::Index>(getRootNv(model_))) {
      throw std::invalid_argument(
          "Asynchronous whole-body state dimensions do not match the model");
    }
    const std::uint64_t write =
        async_write_sequence_.load(std::memory_order_relaxed);
    const std::uint64_t read =
        async_read_sequence_.load(std::memory_order_acquire);
    if (write - read >= async_queue_.size()) {
      async_dropped_samples_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    AsyncSnapshot &snapshot = async_queue_[write % async_queue_.size()];
    snapshot.time = t;
    snapshot.q = q;
    snapshot.v = v;
    snapshot.a = a;
    snapshot.tau = tau;
    copy_contact_values(snapshot.contact_position, p, "position");
    copy_contact_values(snapshot.contact_velocity, pd, "velocity");
    copy_contact_values(snapshot.contact_force, f, "force");
    copy_contact_values(snapshot.contact_surface, s, "surface");
    async_write_sequence_.store(write + 1, std::memory_order_release);
  }

  void async_publish_loop() {
    while (true) {
      const std::uint64_t read =
          async_read_sequence_.load(std::memory_order_relaxed);
      const std::uint64_t write =
          async_write_sequence_.load(std::memory_order_acquire);
      if (read == write) {
        if (async_stop_.load(std::memory_order_acquire)) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        continue;
      }

      const AsyncSnapshot &snapshot =
          async_queue_[read % async_queue_.size()];
      try {
        publish_synchronously(
            snapshot.time, snapshot.q, snapshot.v, snapshot.a, snapshot.tau,
            snapshot.contact_position, snapshot.contact_velocity,
            snapshot.contact_force, snapshot.contact_surface);
      } catch (const std::exception &error) {
        RCLCPP_ERROR(node_.get_logger(),
                     "Asynchronous whole-body publication failed: %s",
                     error.what());
      }
      async_read_sequence_.store(read + 1, std::memory_order_release);
    }
  }

  void stop_async_publishing() {
    if (!async_enabled_.exchange(false, std::memory_order_acq_rel)) {
      return;
    }
    async_stop_.store(true, std::memory_order_release);
    if (async_thread_.joinable()) {
      async_thread_.join();
    }
  }
#endif

  void init(const std::vector<std::string> &locked_joints = DEFAULT_VECTOR) {
    a_.setZero();

    if (locked_joints.size() != 0) {
      // Check the size of the reference configuration
      if (qref_.size() != model_.nq) {
#ifdef ROS2
        RCLCPP_ERROR_STREAM(
            node_.get_logger(),
            "Invalid argument: qref has wrong dimension (it should be "
                << std::to_string(model_.nq) << ")");
#else
        ROS_ERROR_STREAM(
            "Invalid argument: qref has wrong dimension (it should be "
            << std::to_string(model_.nq) << ")");
#endif
      }
      // Build the reduced model
      for (std::string name : locked_joints) {
        if (model_.existJointName(name)) {
          joint_ids_.push_back(model_.getJointId(name));
        } else {
#ifdef ROS2
          RCLCPP_ERROR_STREAM(node_.get_logger(),
                              "Doesn't exist " << name << " joint");
#else
          ROS_ERROR_STREAM("Doesn't exist " << name << " joint");
#endif
        }
      }
      pinocchio::buildReducedModel(model_, joint_ids_, qref_, reduced_model_);
      // Initialize the vectors and dimensions
      const std::size_t nv_root = getRootNv(model_);
      qfull_ = Eigen::VectorXd::Zero(model_.nq);
      vfull_ = Eigen::VectorXd::Zero(model_.nv);
      afull_ = Eigen::VectorXd::Zero(model_.nv);
      ufull_ = Eigen::VectorXd::Zero(model_.nv - nv_root);
    } else {
      is_reduced_model_ = false;
    }
  }
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_WHOLE_BODY_STATE_PUBLISHER_H_
