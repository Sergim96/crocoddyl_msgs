#ifndef CROCODDYL_MSGS__MPC_VALUE_FUNCTION_H_
#define CROCODDYL_MSGS__MPC_VALUE_FUNCTION_H_

#include <string>
#include <vector>

#include <Eigen/Core>

namespace crocoddyl_msgs {

struct MpcValueFunctionData {
  bool valid{false};
  bool value_valid{false};
  double value_constant{0.0};
  Eigen::VectorXd value_gradient;
  Eigen::MatrixXd value_hessian;
  bool endpoint_value_valid{false};
  double endpoint_value_constant{0.0};
  Eigen::VectorXd endpoint_value_gradient;
  Eigen::MatrixXd endpoint_value_hessian;
  bool running_cost_valid{false};
  double running_cost_constant{0.0};
  Eigen::VectorXd running_state_gradient;
  Eigen::VectorXd running_control_gradient;
  Eigen::MatrixXd running_state_hessian;
  Eigen::MatrixXd running_state_control_hessian;
  Eigen::MatrixXd running_control_hessian;
  double action_constant{0.0};
  Eigen::VectorXd action_state_gradient;
  Eigen::VectorXd action_control_gradient;
  Eigen::MatrixXd action_state_hessian;
  Eigen::MatrixXd action_state_control_hessian;
  Eigen::MatrixXd action_control_hessian;
  std::vector<std::string> active_contacts;
  double regularization{0.0};
};

}  // namespace crocoddyl_msgs

#endif  // CROCODDYL_MSGS__MPC_VALUE_FUNCTION_H_
