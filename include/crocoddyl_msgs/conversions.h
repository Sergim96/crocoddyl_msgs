///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2024, Heriot-Watt University, University of Oxford
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <algorithm>
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/motion.hpp>

#include "crocoddyl_msgs/mpc_value_function.h"

#include <pinocchio/bindings/python/pybind11.hpp>
#define SCALAR double
#define OPTIONS 0
#define JOINT_MODEL_COLLECTION ::pinocchio::JointCollectionDefaultTpl
#include <pinocchio/bindings/python/pybind11-all.hpp>

#ifdef ROS2
#include "crocoddyl_msgs/msg/control.hpp"
#include "crocoddyl_msgs/msg/feedback_gain.hpp"
#include "crocoddyl_msgs/msg/mpc_value_function.hpp"
#include "crocoddyl_msgs/msg/multibody_inertia.hpp"
#include "crocoddyl_msgs/msg/state.hpp"
#include "crocoddyl_msgs/msg/time_interval.hpp"
#include <rclcpp/rclcpp.hpp>
#include <whole_body_state_msgs/msg/whole_body_state.hpp>
#include <whole_body_state_msgs/msg/whole_body_trajectory.hpp>
#else
#include "crocoddyl_msgs/Control.h"
#include "crocoddyl_msgs/FeedbackGain.h"
#include "crocoddyl_msgs/MpcValueFunction.h"
#include "crocoddyl_msgs/MultibodyInertia.h"
#include "crocoddyl_msgs/State.h"
#include "crocoddyl_msgs/TimeInterval.h"
#include <whole_body_state_msgs/WholeBodyState.h>
#include <whole_body_state_msgs/WholeBodyTrajectory.h>
#endif

namespace crocoddyl_msgs {

typedef Eigen::Matrix<double, 10, 1> Vector10d;

static std::vector<std::string> DEFAULT_VECTOR;

enum ControlType { EFFORT = 0, ACCELERATION_CONTACTFORCE };

enum ControlParametrization { POLYZERO = 0, POLYONE, POLYTWO };

enum ContactType { LOCOMOTION = 0, MANIPULATION };

enum ContactStatus { UNKNOWN = 0, SEPARATION, STICKING, SLIPPING };

#ifdef ROS2
typedef crocoddyl_msgs::msg::TimeInterval TimeInterval;
typedef crocoddyl_msgs::msg::State State;
typedef crocoddyl_msgs::msg::Control Control;
typedef crocoddyl_msgs::msg::FeedbackGain FeedbackGain;
typedef crocoddyl_msgs::msg::MpcValueFunction MpcValueFunction;
typedef crocoddyl_msgs::msg::BodyInertia BodyInertia;
typedef crocoddyl_msgs::msg::MultibodyInertia MultibodyInertia;
typedef whole_body_state_msgs::msg::WholeBodyState WholeBodyState;
typedef whole_body_state_msgs::msg::WholeBodyTrajectory WholeBodyTrajectory;
typedef whole_body_state_msgs::msg::ContactState ContactState;
#else
typedef crocoddyl_msgs::TimeInterval TimeInterval;
typedef crocoddyl_msgs::State State;
typedef crocoddyl_msgs::Control Control;
typedef crocoddyl_msgs::FeedbackGain FeedbackGain;
typedef crocoddyl_msgs::MpcValueFunction MpcValueFunction;
typedef crocoddyl_msgs::BodyInertia BodyInertia;
typedef crocoddyl_msgs::MultibodyInertia MultibodyInertia;
typedef whole_body_state_msgs::WholeBodyState WholeBodyState;
typedef whole_body_state_msgs::WholeBodyTrajectory WholeBodyTrajectory;
typedef whole_body_state_msgs::ContactState ContactState;
#endif

static inline void toRowMajor(std::vector<double> &output,
                              const Eigen::Ref<const Eigen::MatrixXd> &input) {
  output.resize(static_cast<std::size_t>(input.rows() * input.cols()));
  for (Eigen::Index row = 0; row < input.rows(); ++row) {
    for (Eigen::Index column = 0; column < input.cols(); ++column) {
      output[static_cast<std::size_t>(row * input.cols() + column)] =
          input(row, column);
    }
  }
}

static inline Eigen::MatrixXd fromRowMajor(const std::vector<double> &input,
                                           const Eigen::Index rows,
                                           const Eigen::Index columns,
                                           const char *name) {
  if (rows < 0 || columns < 0 ||
      input.size() != static_cast<std::size_t>(rows * columns)) {
    throw std::invalid_argument(std::string("Invalid ") + name +
                                " matrix dimensions");
  }
  Eigen::MatrixXd output(rows, columns);
  for (Eigen::Index row = 0; row < rows; ++row) {
    for (Eigen::Index column = 0; column < columns; ++column) {
      output(row, column) =
          input[static_cast<std::size_t>(row * columns + column)];
    }
  }
  return output;
}

static inline void toMsg(MpcValueFunction &msg,
                         const MpcValueFunctionData &value) {
  const Eigen::Index ndx = value.value_gradient.size();
  const Eigen::Index nu = value.action_control_gradient.size();
  if (ndx <= 0 || nu <= 0 ||
      value.value_hessian.rows() != ndx ||
      value.value_hessian.cols() != ndx ||
      (value.endpoint_value_valid &&
       (value.endpoint_value_gradient.size() != ndx ||
        value.endpoint_value_hessian.rows() != ndx ||
        value.endpoint_value_hessian.cols() != ndx)) ||
      (value.running_cost_valid &&
       (value.running_state_gradient.size() != ndx ||
        value.running_control_gradient.size() != nu ||
        value.running_state_hessian.rows() != ndx ||
        value.running_state_hessian.cols() != ndx ||
        value.running_state_control_hessian.rows() != ndx ||
        value.running_state_control_hessian.cols() != nu ||
        value.running_control_hessian.rows() != nu ||
        value.running_control_hessian.cols() != nu)) ||
      value.action_state_gradient.size() != ndx ||
      value.action_state_hessian.rows() != ndx ||
      value.action_state_hessian.cols() != ndx ||
      value.action_state_control_hessian.rows() != ndx ||
      value.action_state_control_hessian.cols() != nu ||
      value.action_control_hessian.rows() != nu ||
      value.action_control_hessian.cols() != nu) {
    throw std::invalid_argument(
        "Inconsistent MPC value/action-value dimensions");
  }
  msg.valid = value.valid;
  msg.value_valid = value.value_valid;
  msg.ndx = static_cast<uint32_t>(ndx);
  msg.nu = static_cast<uint32_t>(nu);
  msg.value_constant = value.value_constant;
  msg.value_gradient.assign(value.value_gradient.data(),
                            value.value_gradient.data() + ndx);
  toRowMajor(msg.value_hessian, value.value_hessian);
  msg.endpoint_value_valid = value.endpoint_value_valid;
  msg.endpoint_value_constant = value.endpoint_value_constant;
  if (value.endpoint_value_valid) {
    msg.endpoint_value_gradient.assign(value.endpoint_value_gradient.data(),
                                       value.endpoint_value_gradient.data() +
                                           ndx);
    toRowMajor(msg.endpoint_value_hessian, value.endpoint_value_hessian);
  } else {
    msg.endpoint_value_gradient.clear();
    msg.endpoint_value_hessian.clear();
  }
  msg.running_cost_valid = value.running_cost_valid;
  msg.running_cost_constant = value.running_cost_constant;
  if (value.running_cost_valid) {
    msg.running_state_gradient.assign(value.running_state_gradient.data(),
                                      value.running_state_gradient.data() +
                                          ndx);
    msg.running_control_gradient.assign(
        value.running_control_gradient.data(),
        value.running_control_gradient.data() + nu);
    toRowMajor(msg.running_state_hessian, value.running_state_hessian);
    toRowMajor(msg.running_state_control_hessian,
               value.running_state_control_hessian);
    toRowMajor(msg.running_control_hessian, value.running_control_hessian);
  } else {
    msg.running_state_gradient.clear();
    msg.running_control_gradient.clear();
    msg.running_state_hessian.clear();
    msg.running_state_control_hessian.clear();
    msg.running_control_hessian.clear();
  }
  msg.action_constant = value.action_constant;
  msg.action_state_gradient.assign(value.action_state_gradient.data(),
                                   value.action_state_gradient.data() + ndx);
  msg.action_control_gradient.assign(
      value.action_control_gradient.data(),
      value.action_control_gradient.data() + nu);
  toRowMajor(msg.action_state_hessian, value.action_state_hessian);
  toRowMajor(msg.action_state_control_hessian,
             value.action_state_control_hessian);
  toRowMajor(msg.action_control_hessian, value.action_control_hessian);
  msg.active_contacts = value.active_contacts;
  msg.regularization = value.regularization;
}

static inline MpcValueFunctionData fromMsg(const MpcValueFunction &msg) {
  const Eigen::Index ndx = static_cast<Eigen::Index>(msg.ndx);
  const Eigen::Index nu = static_cast<Eigen::Index>(msg.nu);
  if (ndx <= 0 || nu <= 0 ||
      msg.value_gradient.size() != static_cast<std::size_t>(ndx) ||
      msg.action_state_gradient.size() != static_cast<std::size_t>(ndx) ||
      msg.action_control_gradient.size() != static_cast<std::size_t>(nu)) {
    throw std::invalid_argument(
        "Invalid MPC value/action-value vector dimensions");
  }
  MpcValueFunctionData value;
  value.valid = msg.valid;
  value.value_valid = msg.value_valid;
  value.value_constant = msg.value_constant;
  value.value_gradient = Eigen::Map<const Eigen::VectorXd>(
      msg.value_gradient.data(), ndx);
  value.value_hessian =
      fromRowMajor(msg.value_hessian, ndx, ndx, "value Hessian");
  value.endpoint_value_valid = msg.endpoint_value_valid;
  value.endpoint_value_constant = msg.endpoint_value_constant;
  if (value.endpoint_value_valid) {
    if (msg.endpoint_value_gradient.size() !=
        static_cast<std::size_t>(ndx)) {
      throw std::invalid_argument(
          "Invalid endpoint MPC value gradient dimensions");
    }
    value.endpoint_value_gradient = Eigen::Map<const Eigen::VectorXd>(
        msg.endpoint_value_gradient.data(), ndx);
    value.endpoint_value_hessian = fromRowMajor(
        msg.endpoint_value_hessian, ndx, ndx, "endpoint value Hessian");
  }
  value.running_cost_valid = msg.running_cost_valid;
  value.running_cost_constant = msg.running_cost_constant;
  if (value.running_cost_valid) {
    if (msg.running_state_gradient.size() !=
            static_cast<std::size_t>(ndx) ||
        msg.running_control_gradient.size() !=
            static_cast<std::size_t>(nu)) {
      throw std::invalid_argument(
          "Invalid running-cost gradient dimensions");
    }
    value.running_state_gradient = Eigen::Map<const Eigen::VectorXd>(
        msg.running_state_gradient.data(), ndx);
    value.running_control_gradient = Eigen::Map<const Eigen::VectorXd>(
        msg.running_control_gradient.data(), nu);
    value.running_state_hessian = fromRowMajor(
        msg.running_state_hessian, ndx, ndx, "running state Hessian");
    value.running_state_control_hessian = fromRowMajor(
        msg.running_state_control_hessian, ndx, nu,
        "running state-control Hessian");
    value.running_control_hessian = fromRowMajor(
        msg.running_control_hessian, nu, nu, "running control Hessian");
  }
  value.action_constant = msg.action_constant;
  value.action_state_gradient = Eigen::Map<const Eigen::VectorXd>(
      msg.action_state_gradient.data(), ndx);
  value.action_control_gradient = Eigen::Map<const Eigen::VectorXd>(
      msg.action_control_gradient.data(), nu);
  value.action_state_hessian = fromRowMajor(
      msg.action_state_hessian, ndx, ndx, "action state Hessian");
  value.action_state_control_hessian =
      fromRowMajor(msg.action_state_control_hessian, ndx, nu,
                   "action state-control Hessian");
  value.action_control_hessian = fromRowMajor(
      msg.action_control_hessian, nu, nu, "action control Hessian");
  value.active_contacts = msg.active_contacts;
  value.regularization = msg.regularization;
  if (!std::isfinite(value.value_constant) ||
      !std::isfinite(value.endpoint_value_constant) ||
      !std::isfinite(value.running_cost_constant) ||
      !std::isfinite(value.action_constant) ||
      !std::isfinite(value.regularization) ||
      !value.value_gradient.allFinite() || !value.value_hessian.allFinite() ||
      (value.endpoint_value_valid &&
       (!value.endpoint_value_gradient.allFinite() ||
        !value.endpoint_value_hessian.allFinite())) ||
      (value.running_cost_valid &&
       (!value.running_state_gradient.allFinite() ||
        !value.running_control_gradient.allFinite() ||
        !value.running_state_hessian.allFinite() ||
        !value.running_state_control_hessian.allFinite() ||
        !value.running_control_hessian.allFinite())) ||
      !value.action_state_gradient.allFinite() ||
      !value.action_control_gradient.allFinite() ||
      !value.action_state_hessian.allFinite() ||
      !value.action_state_control_hessian.allFinite() ||
      !value.action_control_hessian.allFinite()) {
    throw std::invalid_argument("MPC value/action-value data must be finite");
  }
  return value;
}

/**
 * @brief Return the root joint id
 *
 * @param return  Root joint Id
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline std::size_t getRootJointId(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model) {
  return model.existJointName("root_joint")
             ? model.getJointId("root_joint")
             : (model.existJointName("freeflyer_joint")
                    ? model.getJointId("freeflyer_joint")
                    : 0);
}

/**
 * @brief Return the root nq dimension
 *
 * @param return  Root joint nq dimension
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline std::size_t getRootNq(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model) {
  const std::size_t root_joint_id = getRootJointId(model);
  return model.frames[root_joint_id].name != "universe"
             ? model.joints[root_joint_id].nq()
             : 0;
}

/**
 * @brief Return the root nv dimension
 *
 * @param return  Root joint nv dimension
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline std::size_t getRootNv(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model) {
  const std::size_t root_joint_id = getRootJointId(model);
  return model.frames[root_joint_id].name != "universe"
             ? model.joints[root_joint_id].nv()
             : 0;
}

/**
 * @brief Update the Pinocchio model's inertial parameters of a given frame
 *
 * The inertial parameters vector is defined as [m, h_x, h_y, h_z,
 * I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T, where h=mc is
 * the first moment of inertial (mass * barycenter) and the rotational
 * inertia I = I_C + mS^T(c)S(c) where I_C has its origin at the
 * barycenter. Additionally, the type of frame supported are joints,
 * fixed joints, and bodies.
 *
 * @param model[in]      Pinocchio model
 * @param frame_name[in] Frame name
 * @param psi[in]        Inertial parameters
 */
template <int Options, template <typename, int> class JointCollectionTpl>
void updateBodyInertialParameters(
    pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    const std::string &frame_name, const Eigen::Ref<const Vector10d> &psi) {
  if (model.existFrame(frame_name)) {
    const std::size_t frame_id = model.getFrameId(frame_name);
    switch (model.frames[frame_id].type) {
    case pinocchio::FrameType::JOINT: {
      const std::size_t joint_id = model.frames[frame_id].parentJoint;
      model.inertias[joint_id] = pinocchio::Inertia::FromDynamicParameters(psi);
    } break;
    case pinocchio::FrameType::BODY: {
      const pinocchio::Inertia &I_updated =
          pinocchio::Inertia::FromDynamicParameters(psi);
      const std::size_t parent_frame_id = model.frames[frame_id].parentFrame;

      if (model.frames[parent_frame_id].type ==
          pinocchio::FrameType::FIXED_JOINT) {
        // URDF fixed links keep their individual inertia on the fixed-joint
        // frame.  The BODY frame itself is only a kinematic alias.
        auto &fixed_frame = model.frames[parent_frame_id];
        const std::size_t joint_id = fixed_frame.parentJoint;
        Vector10d joint_parameters =
            model.inertias[joint_id].toDynamicParameters();
        joint_parameters -= fixed_frame.placement.act(fixed_frame.inertia)
                                .toDynamicParameters();
        joint_parameters +=
            fixed_frame.placement.act(I_updated).toDynamicParameters();
        fixed_frame.inertia = I_updated;
        model.inertias[joint_id] =
            pinocchio::Inertia::FromDynamicParameters(joint_parameters);
      } else {
        // Pinocchio 4.1 folds a directly joint-attached body's inertia into the
        // parent joint. Recover that contribution by subtracting every fixed
        // body already aggregated into the same joint, then replace it.
        auto &body_frame = model.frames[frame_id];
        const std::size_t joint_id = body_frame.parentJoint;
        Vector10d current_joint_body_parameters =
            model.inertias[joint_id].toDynamicParameters();
        for (const auto &frame : model.frames) {
          if (frame.type == pinocchio::FrameType::FIXED_JOINT &&
              frame.parentJoint == joint_id) {
            current_joint_body_parameters -=
                frame.placement.act(frame.inertia).toDynamicParameters();
          }
        }
        const pinocchio::Inertia current_joint_body =
            pinocchio::Inertia::FromDynamicParameters(
                current_joint_body_parameters);
        const pinocchio::Inertia I_current =
            body_frame.placement.actInv(current_joint_body);
        Vector10d joint_parameters =
            model.inertias[joint_id].toDynamicParameters();
        joint_parameters -=
            body_frame.placement.act(I_current).toDynamicParameters();
        joint_parameters +=
            body_frame.placement.act(I_updated).toDynamicParameters();
        model.inertias[joint_id] =
            pinocchio::Inertia::FromDynamicParameters(joint_parameters);
        // Preserve the per-body value as metadata. Pinocchio algorithms use
        // model.inertias; Frame::inertia is not processed after addFrame().
        body_frame.inertia = I_updated;
      }
    } break;
    case pinocchio::FrameType::FIXED_JOINT: {
      const std::size_t joint_id = model.frames[frame_id].parentJoint;
      const pinocchio::SE3 &jMb = model.frames[frame_id].placement;
      const pinocchio::Inertia &I_updated =
          pinocchio::Inertia::FromDynamicParameters(psi);
      Vector10d joint_parameters =
          model.inertias[joint_id].toDynamicParameters();
      joint_parameters -=
          jMb.act(model.frames[frame_id].inertia).toDynamicParameters();
      joint_parameters += jMb.act(I_updated).toDynamicParameters();
      model.frames[frame_id].inertia = I_updated;
      model.inertias[joint_id] =
          pinocchio::Inertia::FromDynamicParameters(joint_parameters);
    } break;
    default: {
      throw std::invalid_argument("The type of frame " + frame_name +
                                  " is not supported");
    }
    }
  } else {
    throw std::invalid_argument("Doesn't exist " + frame_name + " frame");
  }
}

/**
 * @brief Return the Pinocchio model's inertial parameters of a given frame
 *
 * The inertial parameters vector is defined as [m, h_x, h_y, h_z,
 * I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T, where h=mc is
 * the first moment of inertial (mass * barycenter) and the rotational
 * inertia I = I_C + mS^T(c)S(c) where I_C has its origin at the
 * barycenter. Additionally, the type of frame supported are joints,
 * fixed joints, and bodies
 *
 * @param model[in]      Pinocchio model
 * @param frame_name[in] Frame name
 * @return inertial parameters.
 */
template <int Options, template <typename, int> class JointCollectionTpl>
const Vector10d getBodyInertialParameters(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    const std::string &frame_name) {
  if (model.existFrame(frame_name)) {
    const std::size_t frame_id = model.getFrameId(frame_name);
    switch (model.frames[frame_id].type) {
    case pinocchio::FrameType::JOINT: {
      const std::size_t joint_id = model.frames[frame_id].parentJoint;
      return model.inertias[joint_id].toDynamicParameters();
    } break;
    case pinocchio::FrameType::BODY: {
      const auto &body_frame = model.frames[frame_id];
      const std::size_t parent_frame_id = body_frame.parentFrame;
      if (model.frames[parent_frame_id].type ==
          pinocchio::FrameType::FIXED_JOINT) {
        return model.frames[parent_frame_id].inertia.toDynamicParameters();
      }

      Vector10d joint_body_parameters =
          model.inertias[body_frame.parentJoint].toDynamicParameters();
      for (const auto &frame : model.frames) {
        if (frame.type == pinocchio::FrameType::FIXED_JOINT &&
            frame.parentJoint == body_frame.parentJoint) {
          joint_body_parameters -=
              frame.placement.act(frame.inertia).toDynamicParameters();
        }
      }
      const pinocchio::Inertia joint_body =
          pinocchio::Inertia::FromDynamicParameters(joint_body_parameters);
      return body_frame.placement.actInv(joint_body).toDynamicParameters();
    } break;
    case pinocchio::FrameType::FIXED_JOINT: {
      return model.frames[frame_id].inertia.toDynamicParameters();
    } break;
    default: {
      throw std::invalid_argument("The type of frame " + frame_name +
                                  " is not supported");
    }
    }
  } else {
    throw std::invalid_argument("Doesn't exist " + frame_name + " frame");
  }
}

/**
 * @brief Conversion of Eigen to message for a given
 * crocoddyl_msgs::FeedbackGain message reference
 *
 * @param[out] msg  ROS message that contains the feedback gain
 * @param[in] K     Feedback gain (size nu * nx)
 */
static inline void toMsg(FeedbackGain &msg,
                         const Eigen::Ref<const Eigen::MatrixXd> &K) {
  msg.nu = static_cast<uint32_t>(K.rows());
  msg.nx = static_cast<uint32_t>(K.cols());
  msg.data.resize(msg.nx * msg.nu);
  for (uint32_t i = 0; i < msg.nu; ++i) {
    for (uint32_t j = 0; j < msg.nx; ++j) {
      msg.data[i * msg.nx + j] = K(i, j); // store in row-major order
    }
  }
}

/**
 * @brief Conversion of Eigen to message for a given crocoddyl_msgs::State
 * message reference
 *
 * @param[out] msg  ROS message that contains the state
 * @param[in] x     State at the beginning of the interval
 * @param[in] dx    State's rate of change during the interval
 */
static inline void toMsg(State &msg, const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &dx) {
  msg.x.resize(x.size());
  msg.dx.resize(dx.size());
  for (int i = 0; i < x.size(); ++i) {
    msg.x[i] = x(i);
  }
  for (int i = 0; i < dx.size(); ++i) {
    msg.dx[i] = dx(i);
  }
}

/**
 * @brief Conversion of Eigen to message for a given crocoddyl_msgs::Control
 * message reference
 *
 * @param[out] msg             ROS message that contains the control
 * @param[in] u                Control parameters of the interval
 * @param[in] K                Feedback gain of the interval
 * @param[in] type             Control type
 * @param[in] parametrization  Control parametrization
 */
static inline void toMsg(Control &msg,
                         const Eigen::Ref<const Eigen::VectorXd> &u,
                         const Eigen::Ref<const Eigen::MatrixXd> &K,
                         const ControlType type,
                         const ControlParametrization parametrization) {
  msg.u.resize(u.size());
  for (int i = 0; i < u.size(); ++i) {
    msg.u[i] = u(i);
  }
  toMsg(msg.gain, K);
  switch (type) {
  case ControlType::EFFORT:
    msg.input = crocoddyl_msgs::Control::EFFORT;
    break;
  case ControlType::ACCELERATION_CONTACTFORCE:
    msg.input = crocoddyl_msgs::Control::ACCELERATION_CONTACTFORCE;
    break;
  }
  switch (parametrization) {
  case ControlParametrization::POLYZERO:
    msg.parametrization = crocoddyl_msgs::Control::POLYZERO;
    break;
  case ControlParametrization::POLYONE:
    msg.parametrization = crocoddyl_msgs::Control::POLYONE;
    break;
  case ControlParametrization::POLYTWO:
    msg.parametrization = crocoddyl_msgs::Control::POLYTWO;
    break;
  }
}

/**
 * @brief Conversion of Eigen to crocoddyl_msgs::BodyInertia message
 *
 * @param[out] msg  ROS message that contains the inertial parameters
 * @param[in] psi   Inertial parameters
 */
static inline void toMsg(BodyInertia &msg,
                         const Eigen::Ref<const Vector10d> &psi) {
  const double eps = Eigen::NumTraits<double>::epsilon();
  msg.inertia.m = psi[0];
  msg.inertia.com.x = psi[1] / std::max(psi[0], eps);
  msg.inertia.com.y = psi[2] / std::max(psi[0], eps);
  msg.inertia.com.z = psi[3] / std::max(psi[0], eps);
  msg.inertia.ixx = psi[4];
  msg.inertia.ixy = psi[5];
  msg.inertia.iyy = psi[6];
  msg.inertia.ixz = psi[7];
  msg.inertia.iyz = psi[8];
  msg.inertia.izz = psi[9];
}

/**
 * @brief Conversion from vectors to `whole_body_state_msgs::WholeBodyState`
 *
 * @param model[in]  Pinocchio model
 * @param data[out]  Pinocchio data
 * @param msg[out]   ROS message that contains the whole-body state
 * @param t[in]      Time in secs
 * @param q[in]      Configuration vector (dimension: model.nq)
 * @param v[in]      Generalized velocity (dimension: model.nv)
 * @param a[in]      Generalized acceleration (dimension: model.nv)
 * @param tau[in]    Joint effort
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void
toMsg(const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
      pinocchio::DataTpl<double, Options, JointCollectionTpl> &data,
      WholeBodyState &msg, const double t,
      const Eigen::Ref<const Eigen::VectorXd> &q,
      const Eigen::Ref<const Eigen::VectorXd> &v,
      const Eigen::Ref<const Eigen::VectorXd> &a,
      const Eigen::Ref<const Eigen::VectorXd> &tau) {
  if (q.size() != model.nq) {
    throw std::invalid_argument("Expected q to be " + std::to_string(model.nq) +
                                " but received " + std::to_string(q.size()));
  }
  if (v.size() != model.nv) {
    throw std::invalid_argument("Expected v to be " + std::to_string(model.nv) +
                                " but received " + std::to_string(v.size()));
  }
  if (a.size() != model.nv) {
    throw std::invalid_argument("Expected a to be " + std::to_string(model.nv) +
                                " but received " + std::to_string(a.size()));
  }
  const std::size_t root_joint_id = getRootJointId(model);
  const std::size_t nq_root = getRootNq(model);
  const std::size_t nv_root = getRootNv(model);
  const std::size_t njoints = model.nv - nv_root;
  if (tau.size() != static_cast<int>(njoints) && tau.size() != 0) {
    throw std::invalid_argument("Expected tau to be 0 or " +
                                std::to_string(njoints) + " but received " +
                                std::to_string(tau.size()));
  }
  // Filling the time information
  msg.time = t;
#ifdef ROS2
  msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(t * 1e9));
#else
  msg.header.stamp = ros::Time(t);
#endif
  // Filling the centroidal state
  pinocchio::centerOfMass(model, data, q, v, a);
  // Center of mass
  msg.centroidal.com_position.x = data.com[0].x();
  msg.centroidal.com_position.y = data.com[0].y();
  msg.centroidal.com_position.z = data.com[0].z();
  // Velocity of the CoM expressed in the global frame.
  msg.centroidal.com_velocity.x = data.vcom[0].x();
  msg.centroidal.com_velocity.y = data.vcom[0].y();
  msg.centroidal.com_velocity.z = data.vcom[0].z();
  // Base
  if (nv_root == 6) { // TODO(cmastalli): handle other root joints
    msg.centroidal.base_orientation.x = q(3);
    msg.centroidal.base_orientation.y = q(4);
    msg.centroidal.base_orientation.z = q(5);
    msg.centroidal.base_orientation.w = q(6);
    msg.centroidal.base_angular_velocity.x = v(3);
    msg.centroidal.base_angular_velocity.y = v(4);
    msg.centroidal.base_angular_velocity.z = v(5);
  } else if (nv_root > 0) {
    std::cerr
        << "Warning: toMsg conversion does not yet support root joints "
           "different to a floating base. We cannot publish base information."
        << std::endl;
  }
  // Momenta
  const pinocchio::Force &momenta =
      pinocchio::computeCentroidalMomentum(model, data);
  msg.centroidal.momenta.linear.x = momenta.linear().x();
  msg.centroidal.momenta.linear.y = momenta.linear().y();
  msg.centroidal.momenta.linear.z = momenta.linear().z();
  msg.centroidal.momenta.angular.x = momenta.angular().x();
  msg.centroidal.momenta.angular.y = momenta.angular().y();
  msg.centroidal.momenta.angular.z = momenta.angular().z();
  const pinocchio::Force &momenta_rate =
      pinocchio::computeCentroidalMomentumTimeVariation(model, data);
  msg.centroidal.momenta_rate.linear.x = momenta_rate.linear().x();
  msg.centroidal.momenta_rate.linear.y = momenta_rate.linear().y();
  msg.centroidal.momenta_rate.linear.z = momenta_rate.linear().z();
  msg.centroidal.momenta_rate.angular.x = momenta_rate.angular().x();
  msg.centroidal.momenta_rate.angular.y = momenta_rate.angular().y();
  msg.centroidal.momenta_rate.angular.z = momenta_rate.angular().z();
  // Filling the joint state
  msg.joints.resize(njoints);
  for (std::size_t j = 0; j < njoints; ++j) {
    msg.joints[j].name = model.names[root_joint_id + j + 1];
    msg.joints[j].position = q(nq_root + j);
    msg.joints[j].velocity = v(nv_root + j);
    msg.joints[j].acceleration = a(nv_root + j);
    if (tau.size() != 0) {
      msg.joints[j].effort = tau(j);
    }
  }
}

/**
 * @brief Conversion from vectors to `whole_body_state_msgs::WholeBodyState`
 *
 * @param model[in]  Pinocchio model
 * @param data[out]  Pinocchio data
 * @param msg[out]   ROS message that contains the whole-body state
 * @param t[in]      Time in secs
 * @param q[in]      Configuration vector (dimension: model.nq)
 * @param v[in]      Generalized velocity (dimension: model.nv)
 * @param a[in]      Generalized acceleration (dimension: model.nv)
 * @param tau[in]    Joint effort
 * @param p[in]      Contact position
 * @param pd[in]     Contact velocity
 * @param f[in]      Contact force, type and status
 * @param s[in]      Contact surface and friction coefficient
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void toMsg(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    pinocchio::DataTpl<double, Options, JointCollectionTpl> &data,
    WholeBodyState &msg, const double t,
    const Eigen::Ref<const Eigen::VectorXd> &q,
    const Eigen::Ref<const Eigen::VectorXd> &v,
    const Eigen::Ref<const Eigen::VectorXd> &a,
    const Eigen::Ref<const Eigen::VectorXd> &tau,
    const std::map<std::string, pinocchio::SE3> &p,
    const std::map<std::string, pinocchio::Motion> &pd,
    const std::map<std::string,
                   std::tuple<pinocchio::Force, ContactType, ContactStatus>> &f,
    const std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
  if (p.size() != pd.size()) {
    throw std::invalid_argument(
        "Dimension of contact pose and velocity does not match.");
  }
  if (p.size() != f.size()) {
    throw std::invalid_argument(
        "Dimension of contact pose and force does not match.");
  }
  if (p.size() != s.size()) {
    throw std::invalid_argument(
        "Dimension of contact pose and surface does not match.");
  }
  toMsg(model, data, msg, t, q, v, a, tau);
  // Contacts
  msg.contacts.resize(p.size());
  std::size_t i = 0;
  for (const auto &p_item : p) {
    const std::string &name = p_item.first;
    msg.contacts[i].name = name;
    pinocchio::FrameIndex frame_id = model.getFrameId(name);
    if (static_cast<int>(frame_id) > model.nframes) {
      throw std::runtime_error("Frame '" + name + "' not found.");
    }
    std::map<std::string, pinocchio::Motion>::const_iterator pd_it =
        pd.find(name);
    if (pd_it == pd.end()) {
      throw std::runtime_error("Frame '" + name + "' not found in pd.");
    }
    std::map<std::string, std::tuple<pinocchio::Force, ContactType,
                                     ContactStatus>>::const_iterator f_it =
        f.find(name);
    if (f_it == f.end()) {
      throw std::runtime_error("Frame '" + name + "' not found in f.");
    }
    std::map<std::string, std::pair<Eigen::Vector3d, double>>::const_iterator
        s_it = s.find(name);
    if (s_it == s.end()) {
      throw std::runtime_error("Frame '" + name + "' not found in s.");
    }
    ++i;
  }
  i = 0;
  for (const auto &p_item : p) {
    const pinocchio::SE3 &pose = p_item.second;
    pinocchio::SE3::Quaternion quaternion(pose.rotation());
    msg.contacts[i].pose.position.x = pose.translation().x();
    msg.contacts[i].pose.position.y = pose.translation().y();
    msg.contacts[i].pose.position.z = pose.translation().z();
    msg.contacts[i].pose.orientation.x = quaternion.x();
    msg.contacts[i].pose.orientation.y = quaternion.y();
    msg.contacts[i].pose.orientation.z = quaternion.z();
    msg.contacts[i].pose.orientation.w = quaternion.w();
    ++i;
  }
  i = 0;
  for (const auto &pd_item : pd) {
    const pinocchio::Motion &vel = pd_item.second;
    msg.contacts[i].velocity.linear.x = vel.linear().x();
    msg.contacts[i].velocity.linear.y = vel.linear().y();
    msg.contacts[i].velocity.linear.z = vel.linear().z();
    msg.contacts[i].velocity.angular.x = vel.angular().x();
    msg.contacts[i].velocity.angular.y = vel.angular().y();
    msg.contacts[i].velocity.angular.z = vel.angular().z();
    ++i;
  }
  i = 0;
  for (const auto &f_item : f) {
    const std::tuple<pinocchio::Force, ContactType, ContactStatus> &force =
        f_item.second;
    const pinocchio::Force &wrench = std::get<0>(force);
    const ContactType type = std::get<1>(force);
    switch (type) {
    case ContactType::LOCOMOTION:
      msg.contacts[i].type = ContactState::LOCOMOTION;
      break;
    case ContactType::MANIPULATION:
      msg.contacts[i].type = ContactState::MANIPULATION;
      break;
    }
    const ContactStatus status = std::get<2>(force);
    switch (status) {
    case ContactStatus::UNKNOWN:
      msg.contacts[i].status = ContactState::UNKNOWN;
      break;
    case ContactStatus::SEPARATION:
      msg.contacts[i].status = ContactState::INACTIVE;
      break;
    case ContactStatus::STICKING:
      msg.contacts[i].status = ContactState::ACTIVE;
      break;
    case ContactStatus::SLIPPING:
      msg.contacts[i].status = ContactState::SLIPPING;
      break;
    }
    msg.contacts[i].wrench.force.x = wrench.linear().x();
    msg.contacts[i].wrench.force.y = wrench.linear().y();
    msg.contacts[i].wrench.force.z = wrench.linear().z();
    msg.contacts[i].wrench.torque.x = wrench.angular().x();
    msg.contacts[i].wrench.torque.y = wrench.angular().y();
    msg.contacts[i].wrench.torque.z = wrench.angular().z();
    ++i;
  }
  i = 0;
  for (const auto &s_item : s) {
    const std::pair<Eigen::Vector3d, double> &surf = s_item.second;
    const Eigen::Vector3d &norm = std::get<0>(surf);
    msg.contacts[i].surface_normal.x = norm.x();
    msg.contacts[i].surface_normal.y = norm.y();
    msg.contacts[i].surface_normal.z = norm.z();
    msg.contacts[i].friction_coefficient = std::get<1>(surf);
    ++i;
  }
}

/**
 * @brief Conversion of a feedback gain from a crocoddyl_msgs::FeedbackGain
 * message to Eigen
 *
 * @param[in] msg  ROS message that contains the feedback gain
 * @param[out] K   Feedback gain (size nu * nx)
 */
static inline void fromMsg(const FeedbackGain &msg,
                           Eigen::Ref<Eigen::MatrixXd> K) {
  if (K.rows() != msg.nu || K.cols() != msg.nx) {
    throw std::invalid_argument("The dimensions of K need to be: (" +
                                std::to_string(msg.nu) + ", " +
                                std::to_string(msg.nx) + ").");
  }
  if (msg.data.size() != msg.nu * msg.nx) {
    throw std::invalid_argument("Message incorrect - size of data does not "
                                "match given dimensions (nu,nx)");
  }

  for (std::size_t i = 0; i < msg.nu; ++i) {
    for (std::size_t j = 0; j < msg.nx; ++j) {
      K(i, j) = msg.data[i * msg.nx + j];
    }
  }
}

/**
 * @brief Conversion of a state from a crocoddyl_msgs::State message to Eigen
 *
 * @param[in] msg  ROS message that contains the state
 * @param[out] x   State at the beginning of the interval
 * @param[out] dx  State's rate of change during the interval
 */
static inline void fromMsg(const State &msg, Eigen::Ref<Eigen::VectorXd> x,
                           Eigen::Ref<Eigen::VectorXd> dx) {
  if (static_cast<std::size_t>(x.size()) != msg.x.size()) {
    throw std::invalid_argument("Expected x to be " +
                                std::to_string(msg.x.size()) +
                                " but received " + std::to_string(x.size()));
  }
  if (static_cast<std::size_t>(dx.size()) != msg.dx.size()) {
    throw std::invalid_argument("Expected dx to be " +
                                std::to_string(msg.dx.size()) +
                                " but received " + std::to_string(dx.size()));
  }
  for (std::size_t i = 0; i < msg.x.size(); ++i) {
    x(i) = msg.x[i];
  }
  for (std::size_t i = 0; i < msg.dx.size(); ++i) {
    dx(i) = msg.dx[i];
  }
}

/**
 * @brief Conversion of a control from a crocoddyl_msgs::Control message to
 * Eigen
 *
 * @param[in] msg               ROS message that contains the control
 * @param[out] u                Control parameters of the interval
 * @param[out] K                Feedback gain of the interval
 * @param[out] type             Control type
 * @param[out] parametrization  Control parametrization
 */
static inline void fromMsg(const Control &msg, Eigen::Ref<Eigen::VectorXd> u,
                           Eigen::Ref<Eigen::MatrixXd> K, ControlType &type,
                           ControlParametrization &parametrization) {
  if (static_cast<std::size_t>(u.size()) != msg.u.size()) {
    throw std::invalid_argument("Expected u to be " +
                                std::to_string(msg.u.size()) +
                                " but received " + std::to_string(u.size()));
  }
  for (std::size_t i = 0; i < msg.u.size(); ++i) {
    u(i) = msg.u[i];
  }
  fromMsg(msg.gain, K);
  type = static_cast<ControlType>(msg.input);
  parametrization = static_cast<ControlParametrization>(msg.parametrization);
}

/**
 * @brief Conversion of inertial parameters from a
 * crocoddyl_msgs::BodyInertia message to Eigen
 *
 * @param[in] msg   ROS message that contains the inertial parameters
 * @param[out] psi  Inertial parameters
 */
static inline void fromMsg(const BodyInertia &msg, Eigen::Ref<Vector10d> psi) {
  psi(0) = msg.inertia.m;
  psi(1) = msg.inertia.com.x * psi(0);
  psi(2) = msg.inertia.com.y * psi(0);
  psi(3) = msg.inertia.com.z * psi(0);
  psi(4) = msg.inertia.ixx;
  psi(5) = msg.inertia.ixy;
  psi(6) = msg.inertia.iyy;
  psi(7) = msg.inertia.ixz;
  psi(8) = msg.inertia.iyz;
  psi(9) = msg.inertia.izz;
}

/**
 * @brief Conversion from whole_body_state_msgs::WholeBodyState to deserialized
 * quantities
 *
 * @param model[in]  Pinocchio model
 * @param data[out]  Pinocchio data
 * @param msg[in]    ROS message that contains the whole-body state
 * @param t[out]     Time in secs
 * @param q[out]     Configuration vector (dimension: model.nq)
 * @param v[out]     Generalized velocity (dimension: model.nv)
 * @param a[out]     Generalized acceleratio (dimension: model.nv)
 * @param tau[out]   Joint effort
 * @param p[out]     Contact position
 * @param pd[out]    Contact velocity
 * @param f[out]     Contact force, type and status
 * @param s[out]     Contact surface and friction coefficient
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void
fromMsg(const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
        pinocchio::DataTpl<double, Options, JointCollectionTpl> &data,
        const WholeBodyState &msg, double &t, Eigen::Ref<Eigen::VectorXd> q,
        Eigen::Ref<Eigen::VectorXd> v, Eigen::Ref<Eigen::VectorXd> a,
        Eigen::Ref<Eigen::VectorXd> tau,
        std::map<std::string, pinocchio::SE3> &p,
        std::map<std::string, pinocchio::Motion> &pd,
        std::map<std::string,
                 std::tuple<pinocchio::Force, ContactType, ContactStatus>> &f,
        std::map<std::string, std::pair<Eigen::Vector3d, double>> &s) {
  if (q.size() != model.nq) {
    throw std::invalid_argument("Expected q to be " + std::to_string(model.nq) +
                                " but received " + std::to_string(q.size()));
  }
  if (v.size() != model.nv) {
    throw std::invalid_argument("Expected v to be " + std::to_string(model.nv) +
                                " but received " + std::to_string(v.size()));
  }
  if (a.size() != model.nv) {
    throw std::invalid_argument("Expected a to be " + std::to_string(model.nv) +
                                " but received " + std::to_string(v.size()));
  }
  const std::size_t nv_root = getRootNv(model);
  const std::size_t njoints = model.nv - nv_root;
  if (tau.size() != static_cast<int>(njoints)) {
    throw std::invalid_argument("Expected tau to be " +
                                std::to_string(njoints) + " but received " +
                                std::to_string(tau.size()));
  }
  if (msg.joints.size() != static_cast<std::size_t>(njoints)) {
    throw std::invalid_argument("Message incorrect - msg.joints size is " +
                                std::to_string(msg.joints.size()) +
                                " but expected to be " +
                                std::to_string(njoints));
  }
  t = msg.time;
  // Retrieve the joint state
  for (std::size_t j = 0; j < njoints; ++j) {
    auto joint_id = model.getJointId(msg.joints[j].name);
    auto q_idx = model.idx_qs[joint_id];
    auto v_idx = model.idx_vs[joint_id];
    q(q_idx) = msg.joints[j].position;
    v(v_idx) = msg.joints[j].velocity;
    a(v_idx) = msg.joints[j].acceleration;
    tau(v_idx - nv_root) = msg.joints[j].effort;
  }

  // Retrieve the base state
  if (nv_root == 6) {
    q.head<3>().setZero();
    v.head<3>().setZero();
    q(3) = msg.centroidal.base_orientation.x;
    q(4) = msg.centroidal.base_orientation.y;
    q(5) = msg.centroidal.base_orientation.z;
    q(6) = msg.centroidal.base_orientation.w;
    v(3) = msg.centroidal.base_angular_velocity.x;
    v(4) = msg.centroidal.base_angular_velocity.y;
    v(5) = msg.centroidal.base_angular_velocity.z;

    pinocchio::normalize(model, q);
    pinocchio::centerOfMass(model, data, q, v, a);
    q(0) = msg.centroidal.com_position.x - data.com[0](0);
    q(1) = msg.centroidal.com_position.y - data.com[0](1);
    q(2) = msg.centroidal.com_position.z - data.com[0](2);
    v(0) = msg.centroidal.com_velocity.x - data.vcom[0](0);
    v(1) = msg.centroidal.com_velocity.y - data.vcom[0](1);
    v(2) = msg.centroidal.com_velocity.z - data.vcom[0](2);
    v.head<3>() = Eigen::Quaterniond(q(6), q(3), q(4),
                                     q(5))
                      .toRotationMatrix()
                      .transpose() *
                  v.head<3>(); // local frame
  } else if (nv_root > 0) {
    std::cerr
        << "Warning: fromMsg conversion does not yet support root joints "
           "different to a floating base. We cannot publish base information."
        << std::endl;
  }

  // Retrieve the contact information
  for (const auto &contact : msg.contacts) {
    // Contact pose
    p[contact.name] = pinocchio::SE3(
        Eigen::Quaterniond(
            contact.pose.orientation.w, contact.pose.orientation.x,
            contact.pose.orientation.y, contact.pose.orientation.z),
        Eigen::Vector3d(contact.pose.position.x, contact.pose.position.y,
                        contact.pose.position.z));
    // Contact velocity
    pd[contact.name] = pinocchio::Motion(
        Eigen::Vector3d(contact.velocity.linear.x, contact.velocity.linear.y,
                        contact.velocity.linear.z),
        Eigen::Vector3d(contact.velocity.angular.x, contact.velocity.angular.y,
                        contact.velocity.angular.z));
    // Contact wrench
    ContactType type;
    switch (contact.type) {
    case ContactState::LOCOMOTION:
      type = ContactType::LOCOMOTION;
      break;
    case ContactState::MANIPULATION:
      type = ContactType::MANIPULATION;
      break;
    }
    ContactStatus status;
    switch (contact.status) {
    case ContactState::UNKNOWN:
      status = ContactStatus::UNKNOWN;
      break;
    case ContactState::INACTIVE:
      status = ContactStatus::SEPARATION;
      break;
    case ContactState::ACTIVE:
      status = ContactStatus::STICKING;
      break;
    case ContactState::SLIPPING:
      status = ContactStatus::SLIPPING;
      break;
    }
    f[contact.name] = {
        pinocchio::Force(
            Eigen::Vector3d(contact.wrench.force.x, contact.wrench.force.y,
                            contact.wrench.force.z),
            Eigen::Vector3d(contact.wrench.torque.x, contact.wrench.torque.y,
                            contact.wrench.torque.z)),
        type, status};
    // Surface normal and friction coefficient
    s[contact.name] = {Eigen::Vector3d(contact.surface_normal.x,
                                       contact.surface_normal.y,
                                       contact.surface_normal.z),
                       contact.friction_coefficient};
  }
}

/**
 * @brief Conversion from reduced position, velocity and effort to a full one
 *
 * @param model[in]  Pinocchio model
 * @param reduced_model[in]  Reduced Pinocchio model
 * @param q_out[out]  Configuration vector (dimension: model.nq)
 * @param v_out[out]  Generalized velocity (dimension: model.nv)
 * @param tau_out[out]  Joint effort (dimension: model.nv - nv_root)
 * @param q[in]  Reduced configuration vector (dimension: reduced_model.nq)
 * @param v[in]  Reduced generalized velocity (dimension: reduced_model.nv)
 * @param tau[in]  Reduced joint effort (dimension: reduced_model.nv - nv_root)
 * @param qref[in]  Reference configuration used in the reduced model
 * @param locked_joint_ids[in]  Ids of the locked joints
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void fromReduced(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
        &reduced_model,
    Eigen::Ref<Eigen::VectorXd> q_out, Eigen::Ref<Eigen::VectorXd> v_out,
    Eigen::Ref<Eigen::VectorXd> tau_out,
    const Eigen::Ref<const Eigen::VectorXd> &q_in,
    const Eigen::Ref<const Eigen::VectorXd> &v_in,
    const Eigen::Ref<const Eigen::VectorXd> &tau_in,
    const Eigen::Ref<const Eigen::VectorXd> &qref,
    const std::vector<pinocchio::JointIndex> &locked_joint_ids) {
  const std::size_t root_joint_id = getRootJointId(model);
  const std::size_t nq_root = getRootNq(model);
  const std::size_t nv_root = getRootNv(model);
  const std::size_t njoints = model.nv - nv_root;
  const std::size_t njoints_reduced = reduced_model.nv - nv_root;
  if (q_out.size() != model.nq) {
    throw std::invalid_argument("Expected q_out to be " +
                                std::to_string(model.nq) + " but received " +
                                std::to_string(q_out.size()));
  }
  if (q_in.size() != reduced_model.nq) {
    throw std::invalid_argument("Expected q_in to be " +
                                std::to_string(reduced_model.nq) +
                                " but received " + std::to_string(q_in.size()));
  }
  if (v_out.size() != model.nv) {
    throw std::invalid_argument("Expected v_out to be " +
                                std::to_string(model.nv) + " but received " +
                                std::to_string(v_out.size()));
  }
  if (v_in.size() != reduced_model.nv) {
    throw std::invalid_argument("Expected v_in to be " +
                                std::to_string(reduced_model.nv) +
                                " but received " + std::to_string(v_in.size()));
  }
  if (static_cast<std::size_t>(tau_out.size()) != model.nv - nv_root) {
    throw std::invalid_argument("Expected tau_out to be " +
                                std::to_string(njoints) + " but received " +
                                std::to_string(tau_out.size()));
  }
  if (static_cast<std::size_t>(tau_in.size()) != njoints_reduced) {
    throw std::invalid_argument(
        "Expected tau_in to be " + std::to_string(njoints_reduced) +
        " but received " + std::to_string(tau_in.size()));
  }

  typedef pinocchio::ModelTpl<double, Options, JointCollectionTpl> Model;
  typedef typename Model::JointModel JointModel;

  q_out.head(nq_root) = q_in.head(nq_root);
  v_out.head(nv_root) = v_in.head(nv_root);
  for (std::size_t j = root_joint_id + 1;
       j < static_cast<std::size_t>(reduced_model.njoints); ++j) {
    const std::string &name = reduced_model.names[j];
    const JointModel joint = model.joints[model.getJointId(name)];
    const JointModel reduced_joint =
        model.joints[reduced_model.getJointId(name)];
    q_out(joint.idx_q()) = q_in(reduced_joint.idx_q());
    v_out(joint.idx_v()) = v_in(reduced_joint.idx_v());
    tau_out(joint.idx_v() - nv_root) = tau_in(reduced_joint.idx_v() - nv_root);
  }
  for (pinocchio::JointIndex joint_id : locked_joint_ids) {
    const JointModel joint = model.joints[joint_id];
    q_out(joint.idx_q()) = qref(joint.idx_q());
    v_out(joint.idx_v()) = 0.;
    tau_out(joint.idx_v() - nv_root) = 0.;
  }
}

/**
 * @brief Conversion from reduced position, velocity, acceleration and effort to
 * a full one
 *
 * @param model[in]  Pinocchio model
 * @param reduced_model[in]  Reduced Pinocchio model
 * @param q_out[out]  Configuration vector (dimension: model.nq)
 * @param v_out[out]  Generalized velocity (dimension: model.nv)
 * @param a_out[out]  Generalized acceleration (dimension: model.nv)
 * @param tau_out[out]  Joint effort (dimension: model.nv - nv_root)
 * @param q[in]  Reduced configuration vector (dimension: reduced_model.nq)
 * @param v[in]  Reduced generalized velocity (dimension: reduced_model.nv)
 * @param a[in]  Reduced generalized acceleration (dimension: reduced_model.nv)
 * @param tau[in]  Reduced joint effort (dimension: reduced_model.nv - nv_root)
 * @param qref[in]  Reference configuration used in the reduced model
 * @param locked_joint_ids[in]  Ids of the locked joints
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void fromReduced(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
        &reduced_model,
    Eigen::Ref<Eigen::VectorXd> q_out, Eigen::Ref<Eigen::VectorXd> v_out,
    Eigen::Ref<Eigen::VectorXd> a_out, Eigen::Ref<Eigen::VectorXd> tau_out,
    const Eigen::Ref<const Eigen::VectorXd> &q_in,
    const Eigen::Ref<const Eigen::VectorXd> &v_in,
    const Eigen::Ref<const Eigen::VectorXd> &a_in,
    const Eigen::Ref<const Eigen::VectorXd> &tau_in,
    const Eigen::Ref<const Eigen::VectorXd> &qref,
    const std::vector<pinocchio::JointIndex> &locked_joint_ids) {
  const std::size_t root_joint_id = getRootJointId(model);
  const std::size_t nq_root = getRootNq(model);
  const std::size_t nv_root = getRootNv(model);
  const std::size_t njoints = model.nv - nv_root;
  const std::size_t njoints_reduced = reduced_model.nv - nv_root;
  if (q_out.size() != model.nq) {
    throw std::invalid_argument("Expected q_out to be " +
                                std::to_string(model.nq) + " but received " +
                                std::to_string(q_out.size()));
  }
  if (q_in.size() != reduced_model.nq) {
    throw std::invalid_argument("Expected q_in to be " +
                                std::to_string(reduced_model.nq) +
                                " but received " + std::to_string(q_in.size()));
  }
  if (v_out.size() != model.nv) {
    throw std::invalid_argument("Expected v_out to be " +
                                std::to_string(model.nv) + " but received " +
                                std::to_string(v_out.size()));
  }
  if (v_in.size() != reduced_model.nv) {
    throw std::invalid_argument("Expected v_in to be " +
                                std::to_string(reduced_model.nv) +
                                " but received " + std::to_string(v_in.size()));
  }
  if (a_out.size() != model.nv) {
    throw std::invalid_argument("Expected a_out to be " +
                                std::to_string(model.nv) + " but received " +
                                std::to_string(a_out.size()));
  }
  if (a_in.size() != reduced_model.nv) {
    throw std::invalid_argument("Expected a_in to be " +
                                std::to_string(reduced_model.nv) +
                                " but received " + std::to_string(a_in.size()));
  }
  if (static_cast<std::size_t>(tau_out.size()) != njoints) {
    throw std::invalid_argument("Expected tau_out to be " +
                                std::to_string(njoints) + " but received " +
                                std::to_string(tau_out.size()));
  }
  if (static_cast<std::size_t>(tau_in.size()) != njoints_reduced) {
    throw std::invalid_argument(
        "Expected tau_in to be " + std::to_string(njoints_reduced) +
        " but received " + std::to_string(tau_in.size()));
  }

  typedef pinocchio::ModelTpl<double, Options, JointCollectionTpl> Model;
  typedef typename Model::JointModel JointModel;

  q_out.head(nq_root) = q_in.head(nq_root);
  v_out.head(nv_root) = v_in.head(nv_root);
  a_out.head(nv_root) = a_in.head(nv_root);
  for (std::size_t j = root_joint_id + 1;
       j < static_cast<std::size_t>(reduced_model.njoints); ++j) {
    const std::string &name = reduced_model.names[j];
    const JointModel joint = model.joints[model.getJointId(name)];
    const JointModel reduced_joint =
        model.joints[reduced_model.getJointId(name)];
    q_out(joint.idx_q()) = q_in(reduced_joint.idx_q());
    v_out(joint.idx_v()) = v_in(reduced_joint.idx_v());
    a_out(joint.idx_v()) = a_in(reduced_joint.idx_v());
    tau_out(joint.idx_v() - nv_root) = tau_in(reduced_joint.idx_v() - nv_root);
  }
  for (pinocchio::JointIndex joint_id : locked_joint_ids) {
    const JointModel joint = model.joints[joint_id];
    q_out(joint.idx_q()) = qref(joint.idx_q());
    v_out(joint.idx_v()) = 0.;
    a_out(joint.idx_v()) = 0.;
    tau_out(joint.idx_v() - nv_root) = 0.;
  }
}

/**
 * @brief Conversion to reduced position, velocity and effort from a full one
 *
 * @param model[in]  Pinocchio model
 * @param reduced_model[in]  Reduced Pinocchio model
 * @param q_out[out]  Reduced configuration vector (dimension: reduced_model.nq)
 * @param v_out[out]  Reduced generalized velocity (dimension: reduced_model.nv)
 * @param tau_out[out]  Reduced joint effort (dimension: reduced_model.nv -
 * nv_root)
 * @param q[in]  Configuration vector (dimension: model.nq)
 * @param v[in]  Generalized velocity (dimension: model.nv)
 * @param tau[in]  Joint effort (dimension: model.nv - nv_root)
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void
toReduced(const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
          const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
              &reduced_model,
          Eigen::Ref<Eigen::VectorXd> q_out, Eigen::Ref<Eigen::VectorXd> v_out,
          Eigen::Ref<Eigen::VectorXd> tau_out,
          const Eigen::Ref<const Eigen::VectorXd> &q_in,
          const Eigen::Ref<const Eigen::VectorXd> &v_in,
          const Eigen::Ref<const Eigen::VectorXd> &tau_in) {
  const std::size_t root_joint_id = getRootJointId(model);
  const std::size_t nq_root = getRootNq(model);
  const std::size_t nv_root = getRootNv(model);
  const std::size_t njoints = model.nv - nv_root;
  const std::size_t njoints_reduced = reduced_model.nv - nv_root;
  if (q_out.size() != reduced_model.nq) {
    throw std::invalid_argument(
        "Expected q_out to be " + std::to_string(reduced_model.nq) +
        " but received " + std::to_string(q_out.size()));
  }
  if (q_in.size() != model.nq) {
    throw std::invalid_argument("Expected q_in to be " +
                                std::to_string(model.nq) + " but received " +
                                std::to_string(q_in.size()));
  }
  if (v_out.size() != reduced_model.nv) {
    throw std::invalid_argument(
        "Expected v_out to be " + std::to_string(reduced_model.nv) +
        " but received " + std::to_string(v_out.size()));
  }
  if (v_in.size() != model.nv) {
    throw std::invalid_argument("Expected v_in to be " +
                                std::to_string(model.nv) + " but received " +
                                std::to_string(v_in.size()));
  }
  if (static_cast<std::size_t>(tau_out.size()) != njoints_reduced) {
    throw std::invalid_argument(
        "Expected tau_out to be " + std::to_string(njoints_reduced) +
        " but received " + std::to_string(tau_out.size()));
  }
  if (static_cast<std::size_t>(tau_in.size()) != njoints) {
    throw std::invalid_argument("Expected tau_in to be " +
                                std::to_string(njoints) + " but received " +
                                std::to_string(tau_in.size()));
  }

  typedef pinocchio::ModelTpl<double, Options, JointCollectionTpl> Model;
  typedef typename Model::JointModel JointModel;

  q_out.head(nq_root) = q_in.head(nq_root);
  v_out.head(nv_root) = v_in.head(nv_root);
  for (std::size_t j = root_joint_id + 1;
       j < static_cast<std::size_t>(reduced_model.njoints); ++j) {
    const std::string &name = reduced_model.names[j];
    const JointModel joint = model.joints[model.getJointId(name)];
    const JointModel reduced_joint =
        model.joints[reduced_model.getJointId(name)];
    q_out(reduced_joint.idx_q()) = q_in(joint.idx_q());
    v_out(reduced_joint.idx_v()) = v_in(joint.idx_v());
    tau_out(reduced_joint.idx_v() - nv_root) = tau_in(joint.idx_v() - nv_root);
  }
}

/**
 * @brief Conversion to reduced position, velocity, acceleration and effort from
 * a full one
 *
 * @param model[in]  Pinocchio model
 * @param reduced_model[in]  Reduced Pinocchio model
 * @param q_out[out]  Reduced configuration vector (dimension: reduced_model.nq)
 * @param v_out[out]  Reduced generalized velocity (dimension: reduced_model.nv)
 * @param a_out[out]  Reduced generalized velocity (dimension: reduced_model.nv)
 * @param tau_out[out]  Reduced joint effort (dimension: reduced_model.nv -
 * nv_root)
 * @param q[in]  Configuration vector (dimension: model.nq)
 * @param v[in]  Generalized velocity (dimension: model.nv)
 * @param a[in]  Generalized velocity (dimension: model.nv)
 * @param tau[in]  Joint effort (dimension: model.nv - nv_root)
 */
template <int Options, template <typename, int> class JointCollectionTpl>
static inline void
toReduced(const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
          const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
              &reduced_model,
          Eigen::Ref<Eigen::VectorXd> q_out, Eigen::Ref<Eigen::VectorXd> v_out,
          Eigen::Ref<Eigen::VectorXd> a_out,
          Eigen::Ref<Eigen::VectorXd> tau_out,
          const Eigen::Ref<const Eigen::VectorXd> &q_in,
          const Eigen::Ref<const Eigen::VectorXd> &v_in,
          const Eigen::Ref<const Eigen::VectorXd> &a_in,
          const Eigen::Ref<const Eigen::VectorXd> &tau_in) {
  const std::size_t root_joint_id = getRootJointId(model);
  const std::size_t nq_root = getRootNq(model);
  const std::size_t nv_root = getRootNv(model);
  const std::size_t njoints = model.nv - nv_root;
  const std::size_t njoints_reduced = reduced_model.nv - nv_root;
  if (q_out.size() != reduced_model.nq) {
    throw std::invalid_argument(
        "Expected q_out to be " + std::to_string(reduced_model.nq) +
        " but received " + std::to_string(q_out.size()));
  }
  if (q_in.size() != model.nq) {
    throw std::invalid_argument("Expected q_in to be " +
                                std::to_string(model.nq) + " but received " +
                                std::to_string(q_in.size()));
  }
  if (v_out.size() != reduced_model.nv) {
    throw std::invalid_argument(
        "Expected v_out to be " + std::to_string(reduced_model.nv) +
        " but received " + std::to_string(v_out.size()));
  }
  if (v_in.size() != model.nv) {
    throw std::invalid_argument("Expected v_in to be " +
                                std::to_string(model.nv) + " but received " +
                                std::to_string(v_in.size()));
  }
  if (a_out.size() != reduced_model.nv) {
    throw std::invalid_argument(
        "Expected a_out to be " + std::to_string(reduced_model.nv) +
        " but received " + std::to_string(a_out.size()));
  }
  if (a_in.size() != model.nv) {
    throw std::invalid_argument("Expected a_in to be " +
                                std::to_string(model.nv) + " but received " +
                                std::to_string(a_in.size()));
  }
  if (static_cast<std::size_t>(tau_out.size()) != njoints_reduced) {
    throw std::invalid_argument(
        "Expected tau_out to be " + std::to_string(njoints_reduced) +
        " but received " + std::to_string(tau_out.size()));
  }
  if (static_cast<std::size_t>(tau_in.size()) != njoints) {
    throw std::invalid_argument("Expected tau_in to be " +
                                std::to_string(njoints) + " but received " +
                                std::to_string(tau_in.size()));
  }

  typedef pinocchio::ModelTpl<double, Options, JointCollectionTpl> Model;
  typedef typename Model::JointModel JointModel;

  q_out.head(nq_root) = q_in.head(nq_root);
  v_out.head(nv_root) = v_in.head(nv_root);
  a_out.head(nv_root) = a_in.head(nv_root);
  for (std::size_t j = root_joint_id + 1;
       j < static_cast<std::size_t>(reduced_model.njoints); ++j) {
    const std::string &name = reduced_model.names[j];
    const JointModel joint = model.joints[model.getJointId(name)];
    const JointModel reduced_joint =
        model.joints[reduced_model.getJointId(name)];
    q_out(reduced_joint.idx_q()) = q_in(joint.idx_q());
    v_out(reduced_joint.idx_v()) = v_in(joint.idx_v());
    a_out(reduced_joint.idx_v()) = a_in(joint.idx_v());
    tau_out(reduced_joint.idx_v() - nv_root) = tau_in(joint.idx_v() - nv_root);
  }
}

template <int Options, template <typename, int> class JointCollectionTpl>
static inline std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
fromReduced_return(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
        &reduced_model,
    const Eigen::Ref<const Eigen::VectorXd> &q_in,
    const Eigen::Ref<const Eigen::VectorXd> &v_in,
    const Eigen::Ref<const Eigen::VectorXd> &tau_in,
    const Eigen::Ref<const Eigen::VectorXd> &qref,
    const std::vector<pinocchio::JointIndex> &locked_joint_ids) {
  const std::size_t root_joint_id = getRootJointId(model);
  Eigen::VectorXd q_out = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v_out = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau_out =
      Eigen::VectorXd::Zero(model.nv - model.joints[root_joint_id].nv());
  fromReduced(model, reduced_model, q_out, v_out, tau_out, q_in, v_in, tau_in,
              qref, locked_joint_ids);
  return {q_out, v_out, tau_out};
}

template <int Options, template <typename, int> class JointCollectionTpl>
static inline std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
toReduced_return(
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl> &model,
    const pinocchio::ModelTpl<double, Options, JointCollectionTpl>
        &reduced_model,
    const Eigen::Ref<const Eigen::VectorXd> &q_in,
    const Eigen::Ref<const Eigen::VectorXd> &v_in,
    const Eigen::Ref<const Eigen::VectorXd> &tau_in) {
  const std::size_t nv_root = getRootNv(model);
  Eigen::VectorXd q_out = Eigen::VectorXd::Zero(reduced_model.nq);
  Eigen::VectorXd v_out = Eigen::VectorXd::Zero(reduced_model.nv);
  Eigen::VectorXd tau_out = Eigen::VectorXd::Zero(reduced_model.nv - nv_root);
  toReduced(model, reduced_model, q_out, v_out, tau_out, q_in, v_in, tau_in);
  return {q_out, v_out, tau_out};
}

} // namespace crocoddyl_msgs

#endif // CONVERSIONS_H_
