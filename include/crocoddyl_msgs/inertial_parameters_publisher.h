///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2023-2024, Heriot-Watt University
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_PUBLISHER_H_
#define CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_PUBLISHER_H_

#include <realtime_tools/realtime_publisher.h>

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#else
#include <ros/node_handle.h>
#endif
#include "crocoddyl_msgs/MultibodyInertialParameters.h"

typedef crocoddyl_msgs::MultibodyInertialParameters MultibodyInertialParameters;

namespace crocoddyl_msgs {

class MultibodyInertialParametersRosPublisher {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Matrix<double, 10, 1> Vector10d;
  /**
   * @brief Initialize the multi-body inertial parameters publisher.
   *
   * @param[in] topic  Topic name (default: "/crocoddyl/inertial_parameters")
   */
  MultibodyInertialParametersRosPublisher(
      const std::string &topic = "/crocoddyl/inertial_parameters")
#ifdef ROS2
      : node_("inertial_parameters_publisher"),
        pub_(node_.create_publisher<MultibodyInertialParameters>(topic, 1)) {
    RCLCPP_INFO_STREAM(node_.get_logger(),
                       "Publishing MultibodyInertialParameters messages on "
                           << topic);
  }
#else
  {
    ros::NodeHandle n;
    pub_.init(n, topic, 1);
    ROS_INFO_STREAM("Publishing MultibodyInertialParameters messages on "
                    << topic);
  }
#endif

  ~MultibodyInertialParametersRosPublisher() = default;

  /**
   * @brief Publish a multi-body inertial parameters ROS message.
   *
   * @param parameters[in]    multibody inertial parameters. The inertial
   * parameters vector is defined as [m, h_x, h_y, h_z, I_{xx}, I_{xy}, I_{yy},
   * I_{xz}, I_{yz}, I_{zz}]^T, where h=mc is the first moment of inertial m*COM
   * and I has its origin in the frame, I = I_C + mS^T(c)S(c) and I_C has its
   * origin at the barycenter
   */
  void publish(const std::map<std::string, const Eigen::Ref<const Vector10d>>
                   &parameters) {
    const std::size_t n_bodies = parameters.size();
    pub_.msg_.parameters.resize(n_bodies);

    if (pub_.trylock()) {
      pub_.msg_.header.stamp = ros::Time::now();
      unsigned int i = 0;
      for (const auto &[body_name, psi] : parameters) {
        pub_.msg_.parameters[i].name = body_name;
        pub_.msg_.parameters[i].inertia.m = psi[0];
        pub_.msg_.parameters[i].inertia.com.x = psi[1] / psi[0];
        pub_.msg_.parameters[i].inertia.com.y = psi[2] / psi[0];
        pub_.msg_.parameters[i].inertia.com.z = psi[3] / psi[0];
        pub_.msg_.parameters[i].inertia.ixx = psi[4];
        pub_.msg_.parameters[i].inertia.ixy = psi[5];
        pub_.msg_.parameters[i].inertia.iyy = psi[6];
        pub_.msg_.parameters[i].inertia.ixz = psi[7];
        pub_.msg_.parameters[i].inertia.iyz = psi[8];
        pub_.msg_.parameters[i].inertia.izz = psi[9];
        i++;
      }
      pub_.unlockAndPublish();
    }
  }

private:
  realtime_tools::RealtimePublisher<MultibodyInertialParameters> pub_;
};

} // namespace crocoddyl_msgs

#endif // CROCODDYL_MSG_MULTIBODY_INERTIAL_PARAMETERS_PUBLISHER_H_
