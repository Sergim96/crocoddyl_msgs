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
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <thread>
#include <utility>

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
  using StateInterpolator = std::function<Eigen::VectorXd(
      const Eigen::VectorXd &, const Eigen::VectorXd &, double)>;

  /**
   * @brief Initialize the solver trajectory subscriber
   *
   * @param[in] topic  Topic name
   * @param[in] frame  Odometry frame
   */
  SolverTrajectoryRosSubscriber(
      const std::string &topic = "/crocoddyl/solver_trajectory",
      bool interpolation = false, unsigned int interpolation_window = 0,
      StateInterpolator state_interpolator = StateInterpolator())
#ifdef ROS2
      : node_(rclcpp::Node::make_shared(
            "solver_trajectory_subscriber",
            // This helper owns its clock and executor.  Inheriting the host
            // process' global use_sim_time override can put this clock in a
            // different time domain from the equally private trajectory
            // publisher, making every reference appear indefinitely future.
            rclcpp::NodeOptions().use_global_arguments(false))),
        sub_(node_->create_subscription<SolverTrajectory>(
            topic, 1,
            std::bind(&SolverTrajectoryRosSubscriber::callback, this,
                      std::placeholders::_1))),
        has_new_msg_(false), last_msg_time_(0.), communication_delay_(0.),
        interpolation_(interpolation),
        interpolation_window_(interpolation_window),
        state_interpolator_(std::move(state_interpolator)) {
    spinner_.add_node(node_);
    thread_ = std::thread([this]() { this->spin(); });
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Subscribing SolverTrajectory messages on " << topic);
#else
      : node_(), spinner_(2),
        sub_(node_.subscribe<SolverTrajectory>(
            topic, 1, &SolverTrajectoryRosSubscriber::callback, this,
            ros::TransportHints().tcpNoDelay())),
        has_new_msg_(false), last_msg_time_(0.), communication_delay_(0.),
        interpolation_(interpolation),
        interpolation_window_(interpolation_window),
        state_interpolator_(std::move(state_interpolator)) {
    // std::cout << "interpolation_window_: " << interpolation_window_
    //           << std::endl;
    spinner_.start();
    ROS_INFO_STREAM("Subscribing SolverTrajectory messages on " << topic);
#endif
  }
  ~SolverTrajectoryRosSubscriber() {
#ifdef ROS2
    stop_requested_.store(true, std::memory_order_release);
    spinner_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
#endif
  }

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
    SolverTrajectory message;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      message = msg_;
      has_new_msg_.store(false, std::memory_order_release);
    }

    const std::size_t N = message.intervals.size();
    if (message.state_trajectory.size() != N) {
      throw std::invalid_argument(
          "The size of the state trajectory vector needs to equal "
          "the size of the intervals vector.");
    }
    if (message.control_trajectory.size() != N) {
      throw std::invalid_argument(
          "The size of the control trajectory vector needs to equal "
          "the size of the intervals vector.");
    }
    std::vector<double> ts(N);
    std::vector<double> dts(N);
    std::vector<Eigen::VectorXd> xs(N);
    std::vector<Eigen::VectorXd> dxs(N);
    std::vector<Eigen::VectorXd> us(N);
    std::vector<Eigen::MatrixXd> Ks(N);
    std::vector<crocoddyl_msgs::ControlType> types(N);
    std::vector<crocoddyl_msgs::ControlParametrization> params(N);
    for (std::size_t i = 0; i < N; ++i) {
      const TimeInterval &interval = message.intervals[i];
      const State &state = message.state_trajectory[i];
      const Control &control = message.control_trajectory[i];
      if (!std::isfinite(interval.time) || !std::isfinite(interval.duration) ||
          interval.duration <= 0.0 || (i > 0 && interval.time <= ts[i - 1])) {
        throw std::invalid_argument(
            "Trajectory timestamps must be finite and strictly increasing, "
            "with finite positive durations.");
      }
      ts[i] = interval.time;
      dts[i] = interval.duration;
      xs[i].resize(state.x.size());
      dxs[i].resize(state.dx.size());
      us[i].resize(control.u.size());
      Ks[i].resize(control.gain.nu, control.gain.nx);
      crocoddyl_msgs::fromMsg(state, xs[i], dxs[i]);
      crocoddyl_msgs::fromMsg(control, us[i], Ks[i], types[i], params[i]);
      if (!xs[i].allFinite() || !dxs[i].allFinite() || !us[i].allFinite() ||
          !Ks[i].allFinite()) {
        throw std::invalid_argument(
            "Trajectory state, control and feedback data must be finite.");
      }
    }
    return {ts, dts, xs, dxs, us, Ks, types, params};
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
    const double now = get_clock_time();
    const double communication_delay = get_communication_delay();

    if (ts_queue_.empty()) {
      throw std::runtime_error("Reference queue is empty.");
    }

    double t0 = ts_queue_.front();
    double dt0 = dts_queue_.front();
    const double t_now = now - communication_delay;

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
      return {ts_queue_.front(),    dts_queue_.front(),   xs_queue_.front(),
              dxs_queue_.front(),   us_queue_.front(),    Ks_queue_.front(),
              types_queue_.front(), params_queue_.front()};
    }
    // Nothing ready
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(5);
    oss << "[SolverTrajectoryRosSubscriber::get_current_reference] \n"
        << "No point ready. Waiting window not reached. \n"
        << "Next timestamp: " << t0 << "\n"
        << "Current time: " << now << "\n"
        << "Communication delay: " << communication_delay << "\n"
        << "Current time - delay: " << t_now << "\n"
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
   *    A message whose last interval is already in the past is ignored.
   *
   * 2. **Replace (Same Start Time):**
   *    If the new message starts at approximately the same time as the current
   * queue (within the measured communication delay), the entire queue is
   * cleared and replaced with the new one. This models re-planning from the
   * same initial state.
   *
   * 3. **Append (Starts After Current Ends):**
   *    If the new message starts after the current trajectory has finished
   * executing, the new message is appended
   * to the end of the queue. This allows smooth extension of the current plan.
   *
   * 4. **Merge (Partial Overlap):**
   *    If the new message overlaps partially with the current queue, the prefix
   * of the current queue strictly before `t0_new` is preserved and the new
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
    bool ignored = false;
    return process_queue_and_report(ignored);
  }

  bool process_queue_and_report(bool &processed_new_message) {
    processed_new_message = false;
    const bool pending_message = has_new_msg();
    const double communication_delay = get_communication_delay();
    const double t_now = get_clock_time() - communication_delay;

    // STEP 1: Handle new message if available
    if (pending_message) {
      auto [ts_new, dts_new, xs_new, dxs_new, us_new, Ks_new, types_new,
            params_new] = get_solver_trajectory(); // by value, safe to modify
      processed_new_message = true;

      if (ts_new.empty()) {
        throw std::runtime_error(
            "[SolverTrajectoryRosSubscriber::process_queue] "
            "Received empty trajectory.");
      }

      double t0_new = ts_new.front();
      double tN_new = ts_new.back() + dts_new.back();

      // (A) Reject message that is fully in the past w.r.t. current time
      if (tN_new < t_now) {
#ifndef ROS2
        ROS_WARN_STREAM("[SolverTrajectoryRosSubscriber::process_queue] "
                        "Received fully outdated trajectory: "
                        << "t0_new=" << t0_new << "  tN_new=" << tN_new
                        << "  t_now=" << t_now);
#else
        RCLCPP_WARN_STREAM(node_->get_logger(),
                           "[SolverTrajectoryRosSubscriber::process_queue] "
                           "Received fully outdated trajectory: "
                               << "t0_new=" << t0_new << "  tN_new=" << tN_new
                               << "  t_now=" << t_now);
#endif
      } else {
        // Helper: blend first M samples of *new* traj with a single old anchor
        auto apply_blending =
            [&](const Eigen::VectorXd &x_old, const Eigen::VectorXd &dx_old,
                const Eigen::VectorXd &u_old, const Eigen::MatrixXd &K_old) {
              if (!interpolation_ || interpolation_window_ == 0)
                return;
              std::size_t M =
                  std::min<std::size_t>(interpolation_window_, xs_new.size());
              for (std::size_t i = 0; i < M; ++i) {
                const double alpha = static_cast<double>(i + 1) /
                                     static_cast<double>(M); // (0, 1]

                if (state_interpolator_) {
                  Eigen::VectorXd blended_state =
                      state_interpolator_(x_old, xs_new[i], alpha);
                  if (blended_state.size() != xs_new[i].size() ||
                      !blended_state.allFinite()) {
                    throw std::invalid_argument(
                        "The state interpolator returned an invalid state.");
                  }
                  xs_new[i] = std::move(blended_state);
                }
                if (dx_old.size() != dxs_new[i].size() ||
                    u_old.size() != us_new[i].size() ||
                    K_old.rows() != Ks_new[i].rows() ||
                    K_old.cols() != Ks_new[i].cols()) {
                  throw std::invalid_argument(
                      "Cannot blend trajectory samples with different "
                      "tangent, control, or feedback dimensions.");
                }
                dxs_new[i] = (1.0 - alpha) * dx_old + alpha * dxs_new[i];
                us_new[i] = (1.0 - alpha) * u_old + alpha * us_new[i];
                Ks_new[i] = (1.0 - alpha) * K_old + alpha * Ks_new[i];
              }
            };

        // (B) Message still has future content -> do replace/append/merge
        double t0_cur = ts_queue_.empty() ? t0_new : ts_queue_.front();
        double tN_cur =
            ts_queue_.empty() ? t0_new : ts_queue_.back() + dts_queue_.back();

        // 1) REPLACE (same start time)
        const double timing_tolerance = std::max(communication_delay, 1e-9);
        if (std::abs(t0_new - t0_cur) <= timing_tolerance) {
          // std::cout << "Replacing current trajectory." << std::endl;

          if (!ts_queue_.empty() && interpolation_ &&
              interpolation_window_ > 0) {
            // std::cout << "Replacing with interpolation." << std::endl;
            const Eigen::VectorXd &x_old = xs_queue_.front();
            const Eigen::VectorXd &dx_old = dxs_queue_.front();
            const Eigen::VectorXd &u_old = us_queue_.front();
            const Eigen::MatrixXd &K_old = Ks_queue_.front();
            apply_blending(x_old, dx_old, u_old, K_old);
          } else {
            // std::cout << "Replacing without interpolation." << std::endl;
          }

          // Replace queue by (possibly blended) new trajectory
          ts_queue_.assign(ts_new.begin(), ts_new.end());
          dts_queue_.assign(dts_new.begin(), dts_new.end());
          xs_queue_.assign(xs_new.begin(), xs_new.end());
          dxs_queue_.assign(dxs_new.begin(), dxs_new.end());
          us_queue_.assign(us_new.begin(), us_new.end());
          Ks_queue_.assign(Ks_new.begin(), Ks_new.end());
          types_queue_.assign(types_new.begin(), types_new.end());
          params_queue_.assign(params_new.begin(), params_new.end());

          // 2) APPEND (new starts after current ends)
        } else if (tN_cur + timing_tolerance < t0_new) {
          // std::cout << "Appending current trajectory." << std::endl;

          if (!ts_queue_.empty() && interpolation_ &&
              interpolation_window_ > 0) {
            // std::cout << "Appending with interpolation." << std::endl;
            // Anchor = last point of current queue
            const Eigen::VectorXd &x_old = xs_queue_.back();
            const Eigen::VectorXd &dx_old = dxs_queue_.back();
            const Eigen::VectorXd &u_old = us_queue_.back();
            const Eigen::MatrixXd &K_old = Ks_queue_.back();
            apply_blending(x_old, dx_old, u_old, K_old);
          } else {
            // std::cout << "Appending without interpolation." << std::endl;
          }

          // Append (possibly blended) new trajectory
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

          // 3) MERGE (partial overlap)
        } else {
          // std::cout << "Merging current trajectory." << std::endl;

          std::deque<double> ts_merged;
          std::deque<double> dts_merged;
          std::deque<Eigen::VectorXd> xs_merged;
          std::deque<Eigen::VectorXd> dxs_merged;
          std::deque<Eigen::VectorXd> us_merged;
          std::deque<Eigen::MatrixXd> Ks_merged;
          std::deque<crocoddyl_msgs::ControlType> types_merged;
          std::deque<crocoddyl_msgs::ControlParametrization> params_merged;

          // Preserve only samples strictly before the new trajectory. Using
          // t0_new + communication_delay here produces a non-monotonic queue
          // when the new samples beginning at t0_new are appended.
          std::size_t last_preserved_idx = 0;
          bool preserved_any = false;
          for (std::size_t i = 0; i < ts_queue_.size(); ++i) {
            if (ts_queue_[i] < t0_new) {
              ts_merged.push_back(ts_queue_[i]);
              dts_merged.push_back(dts_queue_[i]);
              xs_merged.push_back(xs_queue_[i]);
              dxs_merged.push_back(dxs_queue_[i]);
              us_merged.push_back(us_queue_[i]);
              Ks_merged.push_back(Ks_queue_[i]);
              types_merged.push_back(types_queue_[i]);
              params_merged.push_back(params_queue_[i]);

              last_preserved_idx = ts_merged.size() - 1;
              preserved_any = true;
            } else {
              break;
            }
          }

          // --- Decide behaviour for MERGE ---
          if (!interpolation_ || interpolation_window_ == 0) {
            // Global interpolation disabled -> plain merge
            // std::cout << "Merging without interpolation (disabled)."
            //           << std::endl;

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

          } else if (!preserved_any) {
            // No prefix preserved -> this is effectively a REPLACE with
            // interpolation
            // std::cout << "Merge: no preserved prefix -> treating as REPLACE "
            //              "with interpolation."
            //           << std::endl;

            if (!xs_queue_.empty()) {
              const Eigen::VectorXd &x_old = xs_queue_.front();
              const Eigen::VectorXd &dx_old = dxs_queue_.front();
              const Eigen::VectorXd &u_old = us_queue_.front();
              const Eigen::MatrixXd &K_old = Ks_queue_.front();

              // Blend first M samples of NEW towards the old front
              apply_blending(x_old, dx_old, u_old, K_old);
            }

            // Now fully REPLACE the queue with the (possibly blended) new
            // trajectory
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

          } else {
            // Normal MERGE with interpolation: keep prefix, blend from last
            // preserved sample
            // std::cout << "Merging with interpolation." << std::endl;

            const Eigen::VectorXd &x_old = xs_merged[last_preserved_idx];
            const Eigen::VectorXd &dx_old = dxs_merged[last_preserved_idx];
            const Eigen::VectorXd &u_old = us_merged[last_preserved_idx];
            const Eigen::MatrixXd &K_old = Ks_merged[last_preserved_idx];

            apply_blending(x_old, dx_old, u_old, K_old);

            // Append (now blended) new trajectory
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

            // Commit merged queue
            std::swap(ts_queue_, ts_merged);
            std::swap(dts_queue_, dts_merged);
            std::swap(xs_queue_, xs_merged);
            std::swap(dxs_queue_, dxs_merged);
            std::swap(us_queue_, us_merged);
            std::swap(Ks_queue_, Ks_merged);
            std::swap(types_queue_, types_merged);
            std::swap(params_queue_, params_merged);
          }
        }
      }
    }

    // STEP 2: Determine if queue has a reference ready for execution

    // Drop fully expired intervals
    while (ts_queue_.size() >= 2 &&
           ts_queue_.front() + dts_queue_.front() <= t_now) {
      ts_queue_.pop_front();
      dts_queue_.pop_front();
      xs_queue_.pop_front();
      dxs_queue_.pop_front();
      us_queue_.pop_front();
      Ks_queue_.pop_front();
      types_queue_.pop_front();
      params_queue_.pop_front();
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
  bool has_new_msg() const {
    return has_new_msg_.load(std::memory_order_acquire);
  }

  double get_communication_delay() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return communication_delay_;
  }

  /**
   * @brief Return the time used to select and interpolate trajectory samples.
   */
  double get_execution_time() const {
    return get_clock_time() - get_communication_delay();
  }

  /**
   * @brief Return a snapshot of queued timestamps for diagnostics and tests.
   */
  std::vector<double> get_queue_timestamps() const {
    return std::vector<double>(ts_queue_.begin(), ts_queue_.end());
  }

private:
#ifdef ROS2
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor spinner_;
  std::thread thread_;
  std::atomic_bool stop_requested_{false};
  void spin() {
    while (!stop_requested_.load(std::memory_order_acquire)) {
      spinner_.spin_once(std::chrono::milliseconds(10));
    }
  }
  rclcpp::Subscription<SolverTrajectory>::SharedPtr sub_; //!< ROS subscriber
#else
  ros::NodeHandle node_;
  ros::AsyncSpinner spinner_;
  ros::Subscriber sub_; //!< ROS subscriber
#endif
  mutable std::mutex mutex_;     //!< Protects the message timing and payload
  SolverTrajectory msg_;         //!< Solver trajectory message
  std::atomic_bool has_new_msg_; //!< Indicates a pending unprocessed message
  double last_msg_time_; //!< Last message time needed to ensure each message
                         //!< is newer
  double communication_delay_;
  std::deque<double> ts_queue_;
  std::deque<double> dts_queue_;
  std::deque<Eigen::VectorXd> xs_queue_;
  std::deque<Eigen::VectorXd> dxs_queue_;
  std::deque<Eigen::VectorXd> us_queue_;
  std::deque<Eigen::MatrixXd> Ks_queue_;
  std::deque<crocoddyl_msgs::ControlType> types_queue_;
  std::deque<crocoddyl_msgs::ControlParametrization> params_queue_;

  bool interpolation_;
  unsigned int interpolation_window_;
  StateInterpolator state_interpolator_;

  double get_clock_time() const {
#ifdef ROS2
    return node_->get_clock()->now().seconds();
#else
    return ros::Time::now().toSec();
#endif
  }

  void callback(SolverTrajectorySharedPtr msg) {
#ifdef ROS2
    const double msg_time = rclcpp::Time(msg->header.stamp).seconds();
#else
    const double msg_time = msg->header.stamp.toSec();
#endif
    const double now = get_clock_time();
    bool accepted = false;
    double previous_msg_time = 0.0;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      previous_msg_time = last_msg_time_;
      if (last_msg_time_ <= msg_time) {
        communication_delay_ = std::max(0.0, now - msg_time);
        msg_ = *msg;
        last_msg_time_ = msg_time;
        has_new_msg_.store(true, std::memory_order_release);
        accepted = true;
      }
    }
    if (!accepted) {
#ifdef ROS2
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Out of order message. Last timestamp: "
                             << std::fixed << previous_msg_time
                             << ", current timestamp: " << msg_time);
#else
      ROS_WARN_STREAM("Out of order message. Last timestamp: "
                      << std::fixed << previous_msg_time
                      << ", current timestamp: " << msg_time);
#endif
    }
  }
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_SOLVER_TRAJECTORY_SUBSCRIBER_H_
