# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

* Supported fixed-based robots in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/17
* Add inertial parameters publisher/subscriber and extend whole-body state publisher with accelerations in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/16

## [1.3.1] - 2024-01-27

* Enabled to publish whole-body state and trajectory without the need of passing contact info in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/15

## [1.2.1] - 2023-12-13

* Extended CI jobs in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/12

## [1.2.0] - 2023-09-22

* Fixed bug in init functions for empty locked joints in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/11
* Introduced locked joints in publishers and subscribers in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/10

## [1.0.1] - 2023-08-24

* Used ROS to print starting message and fixed bug when using reduced models in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/9
* Developed other unit tests with Pinocchio in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/8

## [1.0.0] - 2023-07-07

* Supported ROS 2 in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/7
* Integrated pre-commit in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/6
* Replaced nv_root finding via frames to using the first joint in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/4
* Integrated CI in https://github.com/RobotMotorIntelligence/crocoddyl_msgs/pull/3
