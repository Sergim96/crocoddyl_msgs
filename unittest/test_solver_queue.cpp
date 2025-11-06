// BSD 3-Clause License
//
// Copyright (C) 2023-2023, Heriot-Watt University
// All rights reserved.

#include <algorithm>
#include <pinocchio/fwd.hpp>

#include <chrono>
#include <deque>
#include <gtest/gtest.h>
#include <random>
#include <ros/ros.h>
#include <thread>

#include "crocoddyl_msgs/solver_trajectory_publisher.h"
#include "crocoddyl_msgs/solver_trajectory_subscriber.h"

using namespace crocoddyl_msgs;

typedef std::chrono::milliseconds ms;

namespace {

struct TestFixture : public ::testing::Test {
  SolverTrajectoryRosSubscriber sub;
  SolverTrajectoryRosPublisher pub;

  std::tuple<std::vector<double>, std::vector<double>,
             std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>,
             std::vector<Eigen::VectorXd>, std::vector<Eigen::MatrixXd>,
             std::vector<ControlType>, std::vector<ControlParametrization>>
  build_msg(double t0 = 0.0) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dur_dist(0.1, 0.2);
    std::uniform_int_distribution<> N_dist(20, 40);
    std::uniform_int_distribution<> dim_dist(4, 20);

    double h = dur_dist(gen);
    int N = N_dist(gen);
    std::vector<double> dts(N, h);
    std::vector<double> ts(N);
    int nx = dim_dist(gen);
    int nu = std::min(nx, dim_dist(gen));

    for (int i = 0; i < N; ++i)
      ts[i] = t0 + i * h;

    std::vector<Eigen::VectorXd> xs(N, Eigen::VectorXd::Random(nx));
    std::vector<Eigen::VectorXd> dxs(N, Eigen::VectorXd::Random(nx));
    std::vector<Eigen::VectorXd> us(N, Eigen::VectorXd::Random(nu));
    std::vector<Eigen::MatrixXd> Ks(N, Eigen::MatrixXd::Random(nu, nx));

    ControlType type = (rand() % 2 == 0)
                           ? ControlType::EFFORT
                           : ControlType::ACCELERATION_CONTACTFORCE;
    ControlParametrization param =
        static_cast<ControlParametrization>(rand() % 3);

    std::vector<ControlType> types(N, type);
    std::vector<ControlParametrization> params(N, param);

    return {ts, dts, xs, dxs, us, Ks, types, params};
  }
};

// Verifies that if a trajectory starts in the future, process_queue() returns
// false until the time is reached.
TEST_F(TestFixture, TestGetCurrentReferenceDelayed) {
  ros::Duration(1.0).sleep();
  double t0 = ros::Time::now().toSec() + 10;
  auto [ts, dts, xs, dxs, us, Ks, types, params] = build_msg(t0);
  while (!sub.has_new_msg())
    pub.publish(ts, dts, xs, dxs, us, Ks, types, params);
  ASSERT_FALSE(sub.process_queue());
  ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
}

// Verifies that calling get_current_reference() with an empty queue throws an
// exception.
TEST_F(TestFixture, TestQueueEmptyRaises) {
  ASSERT_THROW(
      {
        sub.process_queue();
        sub.get_current_reference();
      },
      std::runtime_error);
}

// Verifies correct behavior when all queue entries are exhausted, including
// proper time-based transitions and clearing.
TEST_F(TestFixture, TestQueueExhaustion) {
  double t0 = ros::Time::now().toSec();
  auto [ts, dts, xs, dxs, us, Ks, types, params] = build_msg(t0);
  while (!sub.has_new_msg()) {
    pub.publish(ts, dts, xs, dxs, us, Ks, types, params);
    std::this_thread::sleep_for(ms(10));
  }

  size_t N = ts.size();
  while (ros::Time::now().toSec() < ts.back() + 1.0) {
    double t_now = ros::Time::now().toSec() - sub.get_communication_delay();
    if (t_now < ts.front() + 1e-6) {
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    } else if (t_now <= ts.back() + dts.back() + 1e-6) {
      ASSERT_TRUE(sub.process_queue());
      auto [t, dt, x, dx, u, K, type_, param] = sub.get_current_reference();
      size_t index = 0;
      while (index + 1 < N && ts[index + 1] <= t_now)
        index++;
      ASSERT_NEAR(t, ts[index], 1e-9);
      ASSERT_NEAR(dt, dts[index], 1e-9);
      ASSERT_TRUE(x.isApprox(xs[index], 1e-9));
      ASSERT_TRUE(dx.isApprox(dxs[index], 1e-9));
      ASSERT_TRUE(u.isApprox(us[index], 1e-9));
      ASSERT_TRUE(K.isApprox(Ks[index], 1e-9));
      ASSERT_EQ(type_, types[index]);
      ASSERT_EQ(param, params[index]);
    } else {
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    }
    std::this_thread::sleep_for(ms(1));
  }
}

// Verifies that a trajectory with timestamps in the past is processed correctly
// and references are served properly.
TEST_F(TestFixture, TestQueuePast) {
  double t0 = ros::Time::now().toSec() - 5;
  auto [ts, dts, xs, dxs, us, Ks, types, params] = build_msg(t0);
  while (!sub.has_new_msg()) {
    pub.publish(ts, dts, xs, dxs, us, Ks, types, params);
    std::this_thread::sleep_for(ms(10));
  }

  size_t N = ts.size();
  while (ros::Time::now().toSec() < ts.back() + 1.0) {
    double t_now = ros::Time::now().toSec() - sub.get_communication_delay();
    if (t_now < ts.front()) {
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    } else if (t_now <= ts.back() + dts.back() + 1e-6) {
      ASSERT_TRUE(sub.process_queue());
      auto [t, dt, x, dx, u, K, type_, param] = sub.get_current_reference();
      size_t index = 0;
      while (index + 1 < N && ts[index + 1] <= t_now + 1e-6)
        index++;
      ASSERT_NEAR(t, ts[index], 1e-9);
      ASSERT_NEAR(dt, dts[index], 1e-9);
      ASSERT_TRUE(x.isApprox(xs[index], 1e-9));
      ASSERT_TRUE(dx.isApprox(dxs[index], 1e-9));
      ASSERT_TRUE(u.isApprox(us[index], 1e-9));
      ASSERT_TRUE(K.isApprox(Ks[index], 1e-9));
      ASSERT_EQ(type_, types[index]);
      ASSERT_EQ(param, params[index]);
    } else {
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    }
    std::this_thread::sleep_for(ms(1));
  }
}

// Verifies that a trajectory with timestamps in the near future waits before
// becoming active and behaves correctly thereafter.
TEST_F(TestFixture, TestQueueFuture) {
  double t0 = ros::Time::now().toSec() + 5;
  auto [ts, dts, xs, dxs, us, Ks, types, params] = build_msg(t0);
  while (!sub.has_new_msg()) {
    pub.publish(ts, dts, xs, dxs, us, Ks, types, params);
    std::this_thread::sleep_for(ms(10));
  }

  size_t N = ts.size();
  while (ros::Time::now().toSec() < ts.back() + 1.0) {
    double t_now = ros::Time::now().toSec() - sub.get_communication_delay();
    if (t_now < ts.front()) {
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    } else if (t_now <= ts.back() + dts.back() + 1e-6) {
      ASSERT_TRUE(sub.process_queue());
      auto [t, dt, x, dx, u, K, type_, param] = sub.get_current_reference();
      size_t index = 0;
      while (index + 1 < N && ts[index + 1] <= t_now)
        index++;
      ASSERT_NEAR(t, ts[index], 1e-9);
      ASSERT_NEAR(dt, dts[index], 1e-9);
      ASSERT_TRUE(x.isApprox(xs[index], 1e-9));
      ASSERT_TRUE(dx.isApprox(dxs[index], 1e-9));
      ASSERT_TRUE(u.isApprox(us[index], 1e-9));
      ASSERT_TRUE(K.isApprox(Ks[index], 1e-9));
      ASSERT_EQ(type_, types[index]);
      ASSERT_EQ(param, params[index]);
    } else {
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    }
    std::this_thread::sleep_for(ms(1));
  }
}

// Tests merging behavior when a new trajectory partially overlaps with the
// current one. Ensures correct merging and execution.
TEST_F(TestFixture, TestPartialMerge) {
  double t0 = ros::Time::now().toSec();
  auto [ts1, dts1, xs1, dxs1, us1, Ks1, types1, params1] = build_msg(t0);
  while (!sub.has_new_msg()) {
    pub.publish(ts1, dts1, xs1, dxs1, us1, Ks1, types1, params1);
    std::this_thread::sleep_for(ms(10));
  }
  ASSERT_TRUE(sub.process_queue());

  // Compute overlap time (1/4 into the first trajectory)
  double overlap_start = ts1.front() + ((ts1.back() - ts1.front()) / 4);

  // Count preserved elements before overlap + delay
  size_t preserved = 0;
  for (; preserved < ts1.size(); ++preserved) {
    if (ts1[preserved] >= overlap_start + sub.get_communication_delay())
      break;
  }

  // Build and publish second trajectory starting at overlap
  auto [ts2, dts2, xs2, dxs2, us2, Ks2, types2, params2] =
      build_msg(overlap_start);
  while (!sub.has_new_msg()) {
    pub.publish(ts2, dts2, xs2, dxs2, us2, Ks2, types2, params2);
    std::this_thread::sleep_for(ms(10));
  }
  ASSERT_TRUE(sub.process_queue());

  // Build merged reference arrays
  std::vector<double> ts_merged;
  std::vector<double> dts_merged;
  std::vector<Eigen::VectorXd> xs_merged, dxs_merged, us_merged;
  std::vector<Eigen::MatrixXd> Ks_merged;
  std::vector<crocoddyl_msgs::ControlType> types_merged;
  std::vector<crocoddyl_msgs::ControlParametrization> params_merged;

  // Add preserved part of first trajectory
  for (size_t i = 0; i < preserved; ++i) {
    ts_merged.push_back(ts1[i]);
    dts_merged.push_back(dts1[i]);
    xs_merged.push_back(xs1[i]);
    dxs_merged.push_back(dxs1[i]);
    us_merged.push_back(us1[i]);
    Ks_merged.push_back(Ks1[i]);
    types_merged.push_back(types1[i]);
    params_merged.push_back(params1[i]);
  }

  // Add full second trajectory
  for (size_t i = 0; i < ts2.size(); ++i) {
    ts_merged.push_back(ts2[i]);
    dts_merged.push_back(dts2[i]);
    xs_merged.push_back(xs2[i]);
    dxs_merged.push_back(dxs2[i]);
    us_merged.push_back(us2[i]);
    Ks_merged.push_back(Ks2[i]);
    types_merged.push_back(types2[i]);
    params_merged.push_back(params2[i]);
  }

  // Main loop: simulate execution by time and validate actual references
  size_t N = ts_merged.size();
  while (ros::Time::now().toSec() < ts_merged.back() + 1.0) {
    double t_now = ros::Time::now().toSec() - sub.get_communication_delay();

    if (t_now < ts_merged.front() + 1e-6) {
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    } else if (t_now <= ts_merged.back() + dts_merged.back() + 1e-6) {
      ASSERT_TRUE(sub.process_queue());
      auto [t, dt, x, dx, u, K, type_, param] = sub.get_current_reference();

      size_t index = 0;
      while (index + 1 < N && ts_merged[index + 1] <= t_now)
        index++;

      ASSERT_NEAR(t, ts_merged[index], 1e-9);
      ASSERT_NEAR(dt, dts_merged[index], 1e-9);
      ASSERT_TRUE(x.isApprox(xs_merged[index], 1e-9));
      ASSERT_TRUE(dx.isApprox(dxs_merged[index], 1e-9));
      ASSERT_TRUE(u.isApprox(us_merged[index], 1e-9));
      ASSERT_TRUE(K.isApprox(Ks_merged[index], 1e-9));
      ASSERT_EQ(type_, types_merged[index]);
      ASSERT_EQ(param, params_merged[index]);
    } else {
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    }

    std::this_thread::sleep_for(ms(1));
  }
}

// Verifies that if a new trajectory starts after the previous one ends, it is
// appended correctly to the queue and both sets of references are returned at
// the right time.
TEST_F(TestFixture, TestAppendAfterEnd) {
  // First trajectory: starting now
  double t0 = ros::Time::now().toSec();
  auto [ts1, dts1, xs1, dxs1, us1, Ks1, types1, params1] = build_msg(t0);
  while (!sub.has_new_msg()) {
    pub.publish(ts1, dts1, xs1, dxs1, us1, Ks1, types1, params1);
    std::this_thread::sleep_for(ms(10));
  }
  ASSERT_TRUE(sub.process_queue());
  while (sub.has_new_msg()) {
    sub.process_queue();
    std::this_thread::sleep_for(ms(10));
  }

  // Second trajectory: starts well after the first ends
  double t_end1 = ts1.back() + dts1.back();
  double t2 = t_end1 + 1.0;
  auto [ts2, dts2, xs2, dxs2, us2, Ks2, types2, params2] = build_msg(t2);
  while (sub.has_new_msg()) {
    sub.process_queue(); // clear any leftover from the previous
    std::this_thread::sleep_for(ms(10));
  }
  while (!sub.has_new_msg()) {
    pub.publish(ts2, dts2, xs2, dxs2, us2, Ks2, types2, params2);
    std::this_thread::sleep_for(ms(10));
  }
  ASSERT_TRUE(sub.process_queue());

  size_t N1 = ts1.size();
  size_t N2 = ts2.size();
  double t_deadzone_start = ts1.back() + dts1.back();
  double t_deadzone_end = ts2.front();

  while (ros::Time::now().toSec() < ts2.back() + 1.0) {
    double t_now = ros::Time::now().toSec() - sub.get_communication_delay();
    if (t_now < ts1.front()) {
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    } else if (t_now <= t_deadzone_start + 1e-6) {
      ASSERT_TRUE(sub.process_queue());
      auto [t, dt, x, dx, u, K, type_, param] = sub.get_current_reference();
      size_t index = 0;
      while (index + 1 < N1 && ts1[index + 1] <= t_now)
        index++;
      ASSERT_NEAR(t, ts1[index], 1e-9);
      ASSERT_NEAR(dt, dts1[index], 1e-9);
      ASSERT_TRUE(x.isApprox(xs1[index], 1e-9));
      ASSERT_TRUE(dx.isApprox(dxs1[index], 1e-9));
      ASSERT_TRUE(u.isApprox(us1[index], 1e-9));
      ASSERT_TRUE(K.isApprox(Ks1[index], 1e-9));
      ASSERT_EQ(type_, types1[index]);
      ASSERT_EQ(param, params1[index]);
    } else if (t_now < t_deadzone_end + 1e-6) {
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    } else if (t_now > t_deadzone_end + 1e-6 &&
               t_now < ts2.back() + dts2.back()) {
      ASSERT_TRUE(sub.process_queue());
      auto [t, dt, x, dx, u, K, type_, param] = sub.get_current_reference();
      size_t index = 0;
      while (index + 1 < N2 && ts2[index + 1] <= t_now)
        index++;
      ASSERT_NEAR(t, ts2[index], 1e-9);
      ASSERT_NEAR(dt, dts2[index], 1e-9);
      ASSERT_TRUE(x.isApprox(xs2[index], 1e-9));
      ASSERT_TRUE(dx.isApprox(dxs2[index], 1e-9));
      ASSERT_TRUE(u.isApprox(us2[index], 1e-9));
      ASSERT_TRUE(K.isApprox(Ks2[index], 1e-9));
      ASSERT_EQ(type_, types2[index]);
      ASSERT_EQ(param, params2[index]);
    } else {
      ASSERT_FALSE(sub.process_queue());
      ASSERT_THROW(sub.get_current_reference(), std::runtime_error);
    }
    std::this_thread::sleep_for(ms(10));
  }
}

// Ensures that old messages (with older internal timestamps) are correctly
// rejected even if the ROS header stamp is newer.
TEST_F(TestFixture, TestDropOldNewMessage) {
  // Publish a valid trajectory
  double t0 = ros::Time::now().toSec();
  auto [ts1, dts1, xs1, dxs1, us1, Ks1, types1, params1] = build_msg(t0);
  while (!sub.has_new_msg()) {
    pub.publish(ts1, dts1, xs1, dxs1, us1, Ks1, types1, params1);
    std::this_thread::sleep_for(ms(10));
  }
  ASSERT_TRUE(sub.process_queue());

  // Now try to send an "older" message but with newer header.stamp
  double t_old = ts1.front() - 1.0; // earlier than queue start
  auto [ts2, dts2, xs2, dxs2, us2, Ks2, types2, params2] = build_msg(t_old);

  while (!sub.has_new_msg()) {
    pub.publish(ts2, dts2, xs2, dxs2, us2, Ks2, types2, params2);
    std::this_thread::sleep_for(ms(10));
  }

  // Ensure process_queue does not replace the queue
  ASSERT_TRUE(sub.process_queue());
  auto [t, dt, x, dx, u, K, type_, param] = sub.get_current_reference();
  size_t idx = 0;
  double t_now = ros::Time::now().toSec() - sub.get_communication_delay();
  while (idx + 1 < ts1.size() && ts1[idx + 1] <= t_now)
    ++idx;
  ASSERT_NEAR(t, ts1[idx], 1e-9); // still pointing to original
}

} // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "solver_trajectory_test");
  return RUN_ALL_TESTS();
}
