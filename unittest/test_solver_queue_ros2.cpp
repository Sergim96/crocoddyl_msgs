// BSD 3-Clause License

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "crocoddyl_msgs/solver_trajectory_publisher.h"
#include "crocoddyl_msgs/solver_trajectory_subscriber.h"

namespace {

using crocoddyl_msgs::ControlParametrization;
using crocoddyl_msgs::ControlType;
using crocoddyl_msgs::MpcValueFunctionData;
using crocoddyl_msgs::SolverTrajectoryRosPublisher;
using crocoddyl_msgs::SolverTrajectoryRosSubscriber;
using namespace std::chrono_literals;

class RosContextEnvironment : public ::testing::Environment {
public:
  void SetUp() override {
    // Simulate embedding the helper in a process such as controller_manager
    // that globally enables simulation time.  The helper nodes deliberately
    // ignore these global arguments and must continue using system time.
    char program[] = "test_solver_queue_ros2";
    char ros_args[] = "--ros-args";
    char parameter[] = "-p";
    char use_sim_time[] = "use_sim_time:=true";
    char *argv[] = {program, ros_args, parameter, use_sim_time};
    int argc = 4;
    rclcpp::init(argc, argv, rclcpp::InitOptions(),
                 rclcpp::SignalHandlerOptions::None);
  }

  void TearDown() override { rclcpp::shutdown(); }
};

const auto *const ros_context =
    ::testing::AddGlobalTestEnvironment(new RosContextEnvironment());

struct TrajectoryData {
  std::vector<double> ts;
  std::vector<double> dts;
  std::vector<Eigen::VectorXd> xs;
  std::vector<Eigen::VectorXd> dxs;
  std::vector<Eigen::VectorXd> us;
  std::vector<Eigen::MatrixXd> Ks;
  std::vector<ControlType> types;
  std::vector<ControlParametrization> params;
  std::vector<MpcValueFunctionData> values;
};

TrajectoryData makeTrajectory(const double t0,
                              const std::vector<double> &offsets,
                              const double duration = 0.1) {
  TrajectoryData data;
  for (const double offset : offsets) {
    data.ts.push_back(t0 + offset);
    data.dts.push_back(duration);
    Eigen::VectorXd x(2);
    x << 1.0, 0.0;
    data.xs.push_back(x);
    data.dxs.push_back(Eigen::VectorXd::Zero(2));
    data.us.push_back(Eigen::VectorXd::Zero(1));
    data.Ks.push_back(Eigen::MatrixXd::Zero(1, 2));
    data.types.push_back(ControlType::EFFORT);
    data.params.push_back(ControlParametrization::POLYZERO);
    MpcValueFunctionData value;
    value.valid = true;
    value.value_valid = true;
    value.value_gradient = Eigen::VectorXd::Zero(2);
    value.value_hessian = Eigen::MatrixXd::Identity(2, 2);
    value.endpoint_value_valid = true;
    value.endpoint_value_gradient = Eigen::VectorXd::Ones(2);
    value.endpoint_value_hessian = 2.0 * Eigen::MatrixXd::Identity(2, 2);
    value.running_cost_valid = true;
    value.running_cost_constant = 3.0;
    value.running_state_gradient = Eigen::VectorXd::Ones(2);
    value.running_control_gradient = Eigen::VectorXd::Ones(1);
    value.running_state_hessian = 3.0 * Eigen::MatrixXd::Identity(2, 2);
    value.running_state_control_hessian = Eigen::MatrixXd::Ones(2, 1);
    value.running_control_hessian = 4.0 * Eigen::MatrixXd::Identity(1, 1);
    value.action_state_gradient = Eigen::VectorXd::Zero(2);
    value.action_control_gradient = Eigen::VectorXd::Zero(1);
    value.action_state_hessian = Eigen::MatrixXd::Identity(2, 2);
    value.action_state_control_hessian = Eigen::MatrixXd::Zero(2, 1);
    value.action_control_hessian = Eigen::MatrixXd::Identity(1, 1);
    value.active_contacts = {"left_foot"};
    value.regularization = 1e-9;
    data.values.push_back(std::move(value));
  }
  return data;
}

void publishUntilReceived(SolverTrajectoryRosPublisher &publisher,
                          SolverTrajectoryRosSubscriber &subscriber,
                          const TrajectoryData &data) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!subscriber.has_new_msg() &&
         std::chrono::steady_clock::now() < deadline) {
    publisher.publish(data.ts, data.dts, data.xs, data.dxs, data.us,
                      data.Ks, data.types, data.params, data.values);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(subscriber.has_new_msg());
}

TEST(SolverTrajectoryQueueRos2, FutureTrajectoryActivatesWithoutAnotherMessage) {
  SolverTrajectoryRosSubscriber subscriber("future_trajectory");
  SolverTrajectoryRosPublisher publisher("future_trajectory");
  const double t0 = subscriber.get_execution_time() + 0.08;
  const TrajectoryData data = makeTrajectory(t0, {0.0, 0.1});

  publishUntilReceived(publisher, subscriber, data);
  bool processed_new_message = false;
  EXPECT_FALSE(subscriber.process_queue_and_report(processed_new_message));
  EXPECT_TRUE(processed_new_message);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (subscriber.get_execution_time() < t0 &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_GE(subscriber.get_execution_time(), t0);
  EXPECT_TRUE(subscriber.process_queue());
  EXPECT_FALSE(subscriber.has_new_msg());
  EXPECT_NO_THROW(subscriber.get_current_reference());
  const MpcValueFunctionData value = subscriber.get_current_value_function();
  EXPECT_TRUE(value.valid);
  EXPECT_EQ(value.value_gradient.size(), 2);
  EXPECT_TRUE(value.endpoint_value_valid);
  EXPECT_TRUE(value.endpoint_value_gradient.isOnes());
  EXPECT_TRUE(value.endpoint_value_hessian.isApprox(
      2.0 * Eigen::MatrixXd::Identity(2, 2)));
  EXPECT_TRUE(value.running_cost_valid);
  EXPECT_TRUE(value.running_state_gradient.isOnes());
  EXPECT_TRUE(value.running_state_control_hessian.isOnes());
  EXPECT_DOUBLE_EQ(value.running_control_hessian(0, 0), 4.0);
  EXPECT_EQ(value.action_control_hessian.rows(), 1);
  ASSERT_EQ(value.active_contacts.size(), 1u);
  EXPECT_EQ(value.active_contacts.front(), "left_foot");
}

TEST(SolverTrajectoryQueueRos2,
     ReferenceAndTimestampUseOneAtomicQueueSelectionTime) {
  SolverTrajectoryRosSubscriber subscriber("atomic_reference_time");
  SolverTrajectoryRosPublisher publisher("atomic_reference_time");
  const double t0 = subscriber.get_execution_time() + 0.05;
  const TrajectoryData data = makeTrajectory(t0, {0.0, 0.020}, 0.020);

  publishUntilReceived(publisher, subscriber, data);
  bool processed_new_message = false;
  EXPECT_FALSE(subscriber.process_queue_and_report(processed_new_message));
  ASSERT_TRUE(processed_new_message);
  while (subscriber.get_execution_time() < t0 + 0.002) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(subscriber.process_queue());
  const double selection_time = subscriber.get_reference_execution_time();

  // Cross the second knot without processing the queue again. The reference
  // accessor must still return the head chosen at selection_time.
  std::this_thread::sleep_for(25ms);
  const auto [time, duration, x, dx, u, gain, type, params] =
      subscriber.get_current_reference();
  EXPECT_DOUBLE_EQ(time, t0);
  EXPECT_DOUBLE_EQ(duration, 0.020);
  EXPECT_GE(selection_time, t0);
  EXPECT_LT(selection_time, t0 + 0.020);
}

TEST(SolverTrajectoryQueueRos2, StateBlendingUsesProvidedManifoldOperation) {
  int interpolation_calls = 0;
  SolverTrajectoryRosSubscriber subscriber(
      "manifold_blending", true, 2,
      [&interpolation_calls](const Eigen::VectorXd &x0,
                             const Eigen::VectorXd &x1, const double alpha) {
        ++interpolation_calls;
        Eigen::VectorXd result = (1.0 - alpha) * x0 + alpha * x1;
        result.normalize();
        return result;
      });
  SolverTrajectoryRosPublisher publisher("manifold_blending");
  const double t0 = subscriber.get_execution_time() - 0.01;
  TrajectoryData first = makeTrajectory(t0, {0.0, 0.2}, 0.2);
  publishUntilReceived(publisher, subscriber, first);
  ASSERT_TRUE(subscriber.process_queue());

  TrajectoryData replacement = makeTrajectory(t0, {0.0, 0.2}, 0.2);
  for (auto &x : replacement.xs) {
    x << 0.0, 1.0;
  }
  publishUntilReceived(publisher, subscriber, replacement);
  ASSERT_TRUE(subscriber.process_queue());
  EXPECT_EQ(interpolation_calls, 2);

  const auto reference = subscriber.get_current_reference();
  const Eigen::VectorXd &state = std::get<2>(reference);
  EXPECT_NEAR(state.norm(), 1.0, 1e-12);
  EXPECT_GT(state[0], 0.0);
  EXPECT_GT(state[1], 0.0);
  EXPECT_FALSE(subscriber.get_current_value_function().valid);
}

TEST(SolverTrajectoryQueueRos2, PartialMergeKeepsTimestampsMonotonic) {
  SolverTrajectoryRosSubscriber subscriber("monotonic_merge");
  SolverTrajectoryRosPublisher publisher("monotonic_merge");
  const double t0 = subscriber.get_execution_time() - 0.01;
  const TrajectoryData first = makeTrajectory(t0, {0.0, 0.1, 0.2}, 0.1);
  publishUntilReceived(publisher, subscriber, first);
  ASSERT_TRUE(subscriber.process_queue());

  const TrajectoryData overlap = makeTrajectory(t0 + 0.15, {0.0, 0.1}, 0.1);
  publishUntilReceived(publisher, subscriber, overlap);
  ASSERT_TRUE(subscriber.process_queue());

  const std::vector<double> timestamps = subscriber.get_queue_timestamps();
  ASSERT_FALSE(timestamps.empty());
  EXPECT_TRUE(std::is_sorted(timestamps.begin(), timestamps.end()));
  EXPECT_EQ(std::adjacent_find(timestamps.begin(), timestamps.end()),
            timestamps.end());
}

TEST(SolverTrajectoryQueueRos2, SubscriberExecutorThreadIsJoinableAtDestruction) {
  for (int index = 0; index < 20; ++index) {
    auto subscriber = std::make_unique<SolverTrajectoryRosSubscriber>(
        "subscriber_lifetime_" + std::to_string(index));
  }
  SUCCEED();
}

TEST(SolverTrajectoryQueueRos2, BackToBackHorizonsKeepTheNewestTrajectory) {
  SolverTrajectoryRosSubscriber subscriber("back_to_back_horizons");
  SolverTrajectoryRosPublisher publisher("back_to_back_horizons");

  // Establish DDS discovery before the behavior under test.  Repeated warm-up
  // publication is deliberately excluded from the assertion below.
  const double warmup_time = subscriber.get_execution_time() - 0.01;
  const TrajectoryData warmup = makeTrajectory(warmup_time, {0.0}, 0.1);
  publishUntilReceived(publisher, subscriber, warmup);
  (void)subscriber.get_solver_trajectory();
  ASSERT_FALSE(subscriber.has_new_msg());

  std::vector<double> offsets;
  offsets.reserve(100);
  for (int index = 0; index < 100; ++index) {
    offsets.push_back(0.01 * static_cast<double>(index));
  }
  const double first_time = subscriber.get_execution_time() + 0.05;
  const TrajectoryData first = makeTrajectory(first_time, offsets, 0.01);
  const TrajectoryData newest = makeTrajectory(first_time + 0.02, offsets, 0.01);

  // Caracal can finish two horizons before a background real-time publisher
  // has released its slot.  The non-real-time solver publisher must hand both
  // calls to rclcpp instead of silently dropping the second (newest) plan.
  publisher.publish(first.ts, first.dts, first.xs, first.dxs, first.us,
                    first.Ks, first.types, first.params, first.values);
  publisher.publish(newest.ts, newest.dts, newest.xs, newest.dxs, newest.us,
                    newest.Ks, newest.types, newest.params, newest.values);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!subscriber.has_new_msg() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(subscriber.has_new_msg());
  // Allow both depth-one deliveries to settle; the subscriber stores the most
  // recently stamped message under its mutex.
  std::this_thread::sleep_for(50ms);
  const auto received = subscriber.get_solver_trajectory();
  const auto &timestamps = std::get<0>(received);
  ASSERT_EQ(timestamps.size(), newest.ts.size());
  EXPECT_NEAR(timestamps.front(), newest.ts.front(), 1e-9);
}

} // namespace
