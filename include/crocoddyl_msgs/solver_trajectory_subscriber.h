///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_
#define CROCODDYL_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_

#include "crocoddyl_msgs/conversions.h"
#include <deque>

#include <Eigen/Dense>
#include <mutex>
#ifdef ROS2
#include "crocoddyl_msgs/msg/solver_trajectory.hpp"
#include <rclcpp/rclcpp.hpp>
#else
#include "crocoddyl_msgs/SolverTrajectory.h"
#include <ros/node_handle.h>
#endif

namespace crocoddyl_msgs {

#ifdef ROS2
typedef msg::SolverTrajectory SolverTrajectory;
typedef const SolverTrajectory::SharedPtr SolverTrajectorySharedPtr;
#else
typedef SolverTrajectory SolverTrajectory;
typedef const SolverTrajectory::ConstPtr &SolverTrajectorySharedPtr;
#endif

class SolverTrajectoryRosSubscriber {
public:
  /**
   * @brief Initialize the solver trajectory subscriber
   *
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
  SolverTrajectoryRosSubscriber(
      const std::string &topic = "/crocoddyl/solver_trajectory",
      bool verbose = false)
#ifdef ROS2
      : node_(rclcpp::Node::make_shared("solver_trajectory_subscriber")),
        sub_(node_->create_subscription<SolverTrajectory>(
            topic, 1,
            std::bind(&SolverTrajectoryRosSubscriber::callback, this,
                      std::placeholders::_1))),
        has_new_msg_(false), is_processing_msg_(false), last_msg_time_(0.),
        communication_delay_(0.), verbose_(verbose), msg_counter_(0)  {
    spinner_.add_node(node_);
    init_time_ = node_->get_clock()->now().seconds();
    thread_ = std::thread([this]() { this->spin(); });
    thread_.detach();
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Subscribing SolverTrajectory messages on " << topic);
#else
      : node_(), spinner_(2),
        sub_(node_.subscribe<SolverTrajectory>(
            topic, 1, &SolverTrajectoryRosSubscriber::callback, this,
            ros::TransportHints().tcpNoDelay())),
        has_new_msg_(false), is_processing_msg_(false), last_msg_time_(0.),
        communication_delay_(0.), verbose_(verbose), msg_counter_(0)  {
    spinner_.start();
    init_time_ = ros::Time::now().toSec();
    if (verbose_) {
      ROS_INFO_STREAM("Verbose mode enabled.");
    }
    if (verbose_) {
      std::string filename = "/tmp/solver_trajectory_debug.csv";
      log_file_.open(filename);
      log_file_ << "t_rel,event,queue_size,queue_start,queue_end,t_now,t0,dt,msg_counter\n";
      ROS_INFO_STREAM("Logging debug info to: " << filename);
    }

    ROS_INFO_STREAM("Subscribing SolverTrajectory messages on " << topic);
#endif
  }
  ~SolverTrajectoryRosSubscriber() = default;

  /**
   * @brief Get the latest solver trajectory
   *
   * @return  A tuple with the vector of time at the beginning of the interval,
   * its durations, initial state, state's rate of change, feed-forward control,
   * feedback gain, type of control and control parametrization.
   */
  std::tuple<std::vector<double>, std::vector<double>,
             std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>,
             std::vector<Eigen::VectorXd>, std::vector<Eigen::MatrixXd>,
             std::vector<crocoddyl_msgs::ControlType>,
             std::vector<crocoddyl_msgs::ControlParametrization>>
  get_solver_trajectory() {
    // start processing the message
    is_processing_msg_ = true;
    std::lock_guard<std::mutex> guard(mutex_);
    const std::size_t N = msg_.intervals.size();
    if (msg_.state_trajectory.size() != N) {
      throw std::invalid_argument(
          "The size of the state trajectory vector needs to equal "
          "the size of the intervals vector.");
    }
    if (msg_.control_trajectory.size() != 0 &&
        msg_.control_trajectory.size() != N) {
      throw std::invalid_argument(
          "The size of the control trajectory vector needs to equal "
          "the size of the intervals vector.");
    }
    ts_.resize(N);
    dts_.resize(N);
    xs_.resize(N);
    dxs_.resize(N);
    us_.resize(N);
    Ks_.resize(N);
    types_.resize(N);
    params_.resize(N);
    for (std::size_t i = 0; i < N; ++i) {
      const TimeInterval &interval = msg_.intervals[i];
      const State &state = msg_.state_trajectory[i];
      const Control &control = msg_.control_trajectory[i];
      ts_[i] = interval.time;
      dts_[i] = interval.duration;
      xs_[i].resize(state.x.size());
      dxs_[i].resize(state.dx.size());
      us_[i].resize(control.u.size());
      Ks_[i].resize(control.gain.nu, control.gain.nx);
      crocoddyl_msgs::fromMsg(state, xs_[i], dxs_[i]);
      crocoddyl_msgs::fromMsg(control, us_[i], Ks_[i], types_[i], params_[i]);
    }
    if (verbose_) {
    #ifdef ROS2
      double now = node_->get_clock()->now().seconds();
    #else
      double now = ros::Time::now().toSec();
    #endif
      double t_rel = now - init_time_;
      ROS_INFO_STREAM("[get_solver_trajectory] t=" << t_rel
                      << "s | Parsed msg #" << msg_counter_
                      << " | N=" << N);
    }
    // finish processing the message
    is_processing_msg_ = false;
    has_new_msg_ = false;
    return {ts_, dts_, xs_, dxs_, us_, Ks_, types_, params_};
  }

  /**
   * @brief Retrieve the next solver trajectory reference point ready for
   * execution.
   *
   * This function returns the first trajectory point in the internal queue that
   * is ready to be executed, based on the current system time corrected by the
   * communication delay.
   *
   * The logic proceeds as follows:
   *
   * - If the queue is empty, an exception is thrown.
   * - If the queue has only one element and it is already expired (i.e., its
   * time interval has passed), the queue is cleared and an exception is thrown.
   * - If the timestamp of the front element is earlier than or equal to the
   * current time (minus communication delay), it is considered ready. Any
   * earlier entries that have also expired are dropped to ensure the returned
   * reference is the most recent one still valid.
   * - If no point is ready for execution, an exception is thrown with
   * diagnostic information.
   *
   * This function should only be called after confirming readiness via
   * `process_queue()`.
   *
   * @return A tuple containing:
   *   - `time` (start of the interval),
   *   - `duration`,
   *   - `state` (`x`),
   *   - `state derivative` (`dx`),
   *   - `feed-forward control` (`u`),
   *   - `feedback gain` (`K`),
   *   - `control type`,
   *   - `control parametrization`.
   *
   * @throws std::runtime_error if the queue is empty, or if no point is ready
   * for execution.
   */
  std::tuple<double, double, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd,
             Eigen::MatrixXd, crocoddyl_msgs::ControlType,
             crocoddyl_msgs::ControlParametrization>
  get_current_reference() {
#ifdef ROS2
    double now = node_->get_clock()->now().seconds();
#else
    double now = ros::Time::now().toSec();
#endif

    if (ts_queue_.empty()) {
      throw std::runtime_error("Reference queue is empty.");
    }

    double t0 = ts_queue_.front();
    double dt0 = dts_queue_.front();
    double t_now = now - communication_delay_;
    if (verbose_) {
      double t_rel = now - init_time_;
      ROS_INFO_STREAM("[get_current_reference] t=" << t_rel << "s");
      ROS_INFO_STREAM("  Queue size: " << ts_queue_.size());
      if (!ts_queue_.empty()) {
        ROS_INFO_STREAM("  Queue start: " << ts_queue_.front()
                        << " | end: " << (ts_queue_.back() + dts_queue_.back())
                        << " | now-delay: " << t_now);
      }
    }
    if (verbose_ && log_file_.is_open()) {
      log_file_ << std::fixed << std::setprecision(6)
                << (now - init_time_) << ",get_current_reference,"
                << ts_queue_.size() << ","
                << ts_queue_.front() << ","
                << (ts_queue_.back() + dts_queue_.back()) << ","
                << t_now << "," << ts_queue_.front() << "," << dts_queue_.front() << ","
                << msg_counter_ << "\n";
    }
    // Handle the case of only one element and it's too old
    if (ts_queue_.size() == 1 && t_now > t0 + dt0) {
      ts_queue_.clear();
      dts_queue_.clear();
      xs_queue_.clear();
      dxs_queue_.clear();
      us_queue_.clear();
      Ks_queue_.clear();
      types_queue_.clear();
      params_queue_.clear();
      if (verbose_) {
        ROS_WARN_STREAM("  -> Dropping stale final point. Queue cleared.");
      }
      throw std::runtime_error(
          "[SolverTrajectoryRosSubscriber::get_current_reference] "
          "Single remaining point is too old. Queue cleared.");
    }

    // Check if the first reference is ready
    if (t0 <= t_now) {
      // Check if the next one is also ready
      while (ts_queue_.size() >= 2 && ts_queue_[1] <= t_now) {
        ts_queue_.pop_front();
        dts_queue_.pop_front();
        xs_queue_.pop_front();
        dxs_queue_.pop_front();
        us_queue_.pop_front();
        Ks_queue_.pop_front();
        types_queue_.pop_front();
        params_queue_.pop_front();
      }
      if (verbose_) {
        ROS_INFO_STREAM("  -> Returning reference at t=" << ts_queue_.front() << " with dt="<< dts_queue_.front());
      }
      return {ts_queue_.front(),    dts_queue_.front(),   xs_queue_.front(),
              dxs_queue_.front(),   us_queue_.front(),    Ks_queue_.front(),
              types_queue_.front(), params_queue_.front()};
    }
    if (verbose_) {
      ROS_WARN_STREAM("  -> No valid reference. Next t=" << t0
                      << " | now-delay=" << t_now
                      << " | time to wait=" << (t0 - t_now) << "s");
    }
    // Nothing ready
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(5);
    oss << "[SolverTrajectoryRosSubscriber::get_current_reference] \n"
        << "No point ready. Waiting window not reached. \n"
        << "Next timestamp: " << t0 << "\n"
        << "Current time: " << now << "\n"
        << "Communication delay: " << communication_delay_ << "\n"
        << "Current time + delay: " << t_now << "\n"
        << "Time to wait: " << (t0 - t_now) << "s";
    throw std::runtime_error(oss.str());
  }

  /**
   * @brief Process any new incoming trajectory message and update the internal
   * execution queue.
   *
   * This method integrates new `SolverTrajectory` messages into the
   * subscriber's internal trajectory queue based on timing logic. It handles
   * message merging and replacement depending on the temporal relationship
   * between the new and current trajectories.
   *
   * Behavior falls into four categories:
   *
   * 1. **Reject Old Message:**
   *    If the new message starts significantly earlier than the current queue
   *    (i.e. `t0_new + delay < t0_current`), it is rejected. This guards
   * against receiving stale messages that are out of date.
   *
   * 2. **Replace (Same Start Time):**
   *    If the new message starts at approximately the same time as the current
   * queue (within `communication_delay_`), the entire queue is cleared and
   * replaced with the new one. This models re-planning from the same initial
   * state.
   *
   * 3. **Append (Starts After Current Ends):**
   *    If the new message starts after the current trajectory has finished
   * executing (i.e. `tN_current + delay < t0_new`), the new message is appended
   * to the end of the queue. This allows smooth extension of the current plan.
   *
   * 4. **Merge (Partial Overlap):**
   *    If the new message overlaps partially with the current queue, the prefix
   * of the current queue up to `t0_new + delay` is preserved and the new
   * message is appended after that. This allows partial updates to a trajectory
   * in progress.
   *
   * After any processing, the method checks whether the front of the queue is
   * ready to be executed by comparing its timestamp (adjusted for communication
   * delay) with the current time. A `true` return indicates that a valid
   * reference is available.
   *
   * @return True if a reference in the queue is ready to be executed; False
   * otherwise.
   */

  bool process_queue() {
#ifdef ROS2
    double now = node_->get_clock()->now().seconds();
#else
    double now = ros::Time::now().toSec();
#endif
    double t_now = now - communication_delay_;

    // STEP 1: Handle new message if available
    if (has_new_msg_) {
      const auto &[ts_new, dts_new, xs_new, dxs_new, us_new, Ks_new, types_new,
                   params_new] = get_solver_trajectory();

      if (ts_new.empty()) {
        throw std::runtime_error("[SolverTrajectoryRosSubscriber::process_"
                                 "queue] Received empty trajectory.");
      }

      double t0_new = ts_new.front();
      double t0_cur = ts_queue_.empty() ? t0_new : ts_queue_.front();
      double tN_cur =
          ts_queue_.empty() ? t0_new : ts_queue_.back() + dts_queue_.back();
      if (verbose_) {
      #ifdef ROS2
        double now = node_->get_clock()->now().seconds();
      #else
        double now = ros::Time::now().toSec();
      #endif
        double t_rel = now - init_time_;
        msg_counter_++;
        ROS_INFO_STREAM("[process_queue] t=" << t_rel << "s | msg #" << msg_counter_
                        << " | new start=" << t0_new << " | current start=" << t0_cur
                        << " | current end=" << tN_cur);
      }
      if (std::abs(t0_new - t0_cur) < communication_delay_) {
        if (verbose_) {
          ROS_INFO_STREAM("  -> Replacing full queue with new message");
        }
        ts_queue_.clear();
        dts_queue_.clear();
        xs_queue_.clear();
        dxs_queue_.clear();
        us_queue_.clear();
        Ks_queue_.clear();
        types_queue_.clear();
        params_queue_.clear();
        for (std::size_t i = 0; i < ts_new.size(); ++i) {
          ts_queue_.push_back(ts_new[i]);
          dts_queue_.push_back(dts_new[i]);
          xs_queue_.push_back(xs_new[i]);
          dxs_queue_.push_back(dxs_new[i]);
          us_queue_.push_back(us_new[i]);
          Ks_queue_.push_back(Ks_new[i]);
          types_queue_.push_back(types_new[i]);
          params_queue_.push_back(params_new[i]);
        }

      } else if (tN_cur + communication_delay_ < t0_new) {
        if (verbose_) {
          ROS_INFO_STREAM("  -> Appending new message after current queue");
        }
        for (std::size_t i = 0; i < ts_new.size(); ++i) {
          ts_queue_.push_back(ts_new[i]);
          dts_queue_.push_back(dts_new[i]);
          xs_queue_.push_back(xs_new[i]);
          dxs_queue_.push_back(dxs_new[i]);
          us_queue_.push_back(us_new[i]);
          Ks_queue_.push_back(Ks_new[i]);
          types_queue_.push_back(types_new[i]);
          params_queue_.push_back(params_new[i]);
        }
      } else {
        if (verbose_) {
          ROS_INFO_STREAM("  -> Merging new message after truncating current queue");
        }
        std::deque<double> ts_merged;
        std::deque<double> dts_merged;
        std::deque<Eigen::VectorXd> xs_merged;
        std::deque<Eigen::VectorXd> dxs_merged;
        std::deque<Eigen::VectorXd> us_merged;
        std::deque<Eigen::MatrixXd> Ks_merged;
        std::deque<crocoddyl_msgs::ControlType> types_merged;
        std::deque<crocoddyl_msgs::ControlParametrization> params_merged;

        // Preserve valid portion of current queue (before t0_new + delay)
        for (std::size_t i = 0; i < ts_queue_.size(); ++i) {
          if (ts_queue_[i] < t0_new + communication_delay_) {
            ts_merged.push_back(ts_queue_[i]);
            dts_merged.push_back(dts_queue_[i]);
            xs_merged.push_back(xs_queue_[i]);
            dxs_merged.push_back(dxs_queue_[i]);
            us_merged.push_back(us_queue_[i]);
            Ks_merged.push_back(Ks_queue_[i]);
            types_merged.push_back(types_queue_[i]);
            params_merged.push_back(params_queue_[i]);
          } else {
            break;
          }
        }

        // Append new message after preserved section
        for (std::size_t i = 0; i < ts_new.size(); ++i) {
          ts_merged.push_back(ts_new[i]);
          dts_merged.push_back(dts_new[i]);
          xs_merged.push_back(xs_new[i]);
          dxs_merged.push_back(dxs_new[i]);
          us_merged.push_back(us_new[i]);
          Ks_merged.push_back(Ks_new[i]);
          types_merged.push_back(types_new[i]);
          params_merged.push_back(params_new[i]);
        }

        std::swap(ts_queue_, ts_merged);
        std::swap(dts_queue_, dts_merged);
        std::swap(xs_queue_, xs_merged);
        std::swap(dxs_queue_, dxs_merged);
        std::swap(us_queue_, us_merged);
        std::swap(Ks_queue_, Ks_merged);
        std::swap(types_queue_, types_merged);
        std::swap(params_queue_, params_merged);
      }
      if (verbose_ && log_file_.is_open()) {
        log_file_ << std::fixed << std::setprecision(6)
                  << (now - init_time_) << ",process_queue,"
                  << ts_queue_.size() << ","
                  << (ts_queue_.empty() ? -1 : ts_queue_.front()) << ","
                  << (ts_queue_.empty() ? -1 : ts_queue_.back() + dts_queue_.back()) << ","
                  << t_now << "," << t0_new << ",-1,"
                  << msg_counter_ << "\n";
      }
    }

    // STEP 2: Determine if queue has a reference ready for execution
    while (ts_queue_.size() >= 2 && ts_queue_.front() + dts_queue_.front() <= t_now) {
      ts_queue_.pop_front();
      dts_queue_.pop_front();
      xs_queue_.pop_front();
      dxs_queue_.pop_front();
      us_queue_.pop_front();
      Ks_queue_.pop_front();
      types_queue_.pop_front();
      params_queue_.pop_front();
    }
    if (verbose_) {
      if (!ts_queue_.empty()) {
        ROS_INFO_STREAM("  Final queue: start=" << ts_queue_.front()
                        << " | end=" << (ts_queue_.back() + dts_queue_.back())
                        << " | size=" << ts_queue_.size());
      } else {
        ROS_INFO_STREAM("  Final queue is empty.");
      }
    }
    // Now check the head entry
    if (!ts_queue_.empty() && ts_queue_.front() <= t_now &&
        t_now <= ts_queue_.front() + dts_queue_.front()) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * @brief Indicate whether we have received a new message
   */
  bool has_new_msg() const { return has_new_msg_; }

  double get_communication_delay() {
    std::lock_guard<std::mutex> guard(mutex_);
    return communication_delay_;
  }

private:
#ifdef ROS2
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor spinner_;
  std::thread thread_;
  void spin() { spinner_.spin(); }
  rclcpp::Subscription<SolverTrajectory>::SharedPtr sub_; //!< ROS subscriber
#else
  ros::NodeHandle node_;
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_; //!< ROS subscriber
#endif
  std::mutex mutex_;       //!< Mutex to prevent race condition on callback
  SolverTrajectory msg_;   //!< Solver trajectory message
  bool has_new_msg_;       //!< Indcate when a new message has been received
  bool is_processing_msg_; //!< Indicate when we are processing the message
  double last_msg_time_; //!< Last message time needed to ensure each message is
                         //!< newer
  double communication_delay_;
  std::vector<double> ts_;
  std::vector<double> dts_;
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> dxs_;
  std::vector<Eigen::VectorXd> us_;
  std::vector<Eigen::MatrixXd> Ks_;
  std::vector<crocoddyl_msgs::ControlType> types_;
  std::vector<crocoddyl_msgs::ControlParametrization> params_;
  std::deque<double> ts_queue_;
  std::deque<double> dts_queue_;
  std::deque<Eigen::VectorXd> xs_queue_;
  std::deque<Eigen::VectorXd> dxs_queue_;
  std::deque<Eigen::VectorXd> us_queue_;
  std::deque<Eigen::MatrixXd> Ks_queue_;
  std::deque<crocoddyl_msgs::ControlType> types_queue_;
  std::deque<crocoddyl_msgs::ControlParametrization> params_queue_;
  bool verbose_;
  double init_time_;
  std::size_t msg_counter_;
  std::ofstream log_file_;

  void callback(SolverTrajectorySharedPtr msg) {
    if (!is_processing_msg_) {
#ifdef ROS2
      double msg_time = rclcpp::Time(msg->header.stamp).seconds();
      double now = node_->get_clock()->now().seconds();
#else
      double msg_time = msg->header.stamp.toSec();
      double now = ros::Time::now().toSec();
#endif
      communication_delay_ = now - msg_time;
      if (last_msg_time_ <= msg_time) {
        // std::cout << "Adding new message with initial time: " <<
        // msg->intervals[0].time << std::endl;
        std::lock_guard<std::mutex> guard(mutex_);
        msg_ = *msg;
        has_new_msg_ = true;
        last_msg_time_ = msg_time;
      } else {
#ifdef ROS2
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "Out of order message. Last timestamp: "
                               << std::fixed << last_msg_time_
                               << ", current timestamp: " << msg_time);
#else
        ROS_WARN_STREAM("Out of order message. Last timestamp: "
                        << std::fixed << last_msg_time_
                        << ", current timestamp: " << msg_time);
#endif
      }
    }
  }
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_
