#!/usr/bin/env python

###############################################################################
# BSD 3-Clause License
#
# Copyright (C) 2023-2024, Heriot-Watt University
# Copyright note valid unless otherwise stated in individual files.
# All rights reserved.
###############################################################################

import os
import random
import time
import unittest

import numpy as np
import pinocchio

ROS_VERSION = int(os.environ["ROS_VERSION"])
if ROS_VERSION == 2:
    import rclpy
else:
    import rospy
    import rosunit

from crocoddyl_ros import (
    MultibodyInertialParametersRosPublisher,
    MultibodyInertialParametersRosSubscriber,
)


class TestInertialParametersAbstract(unittest.TestCase):
    MODEL = None
    def setUp(self):
        if ROS_VERSION == 2:
            if not rclpy.ok():
                rclpy.init()
        else:
            rospy.init_node("crocoddyl_ros", anonymous=True)

    def test_communication(self):
        pub = MultibodyInertialParametersRosPublisher("inertial_parameters")
        sub = MultibodyInertialParametersRosSubscriber("inertial_parameters")
        time.sleep(1)
        # create the name index
        parameters = {}
        names = self.MODEL.names.tolist()
        # publish the inertial parameters
        # note: we have to skip the first body as it is the "universe" one and it is
        # initialized with mass=0, leading to a division by 0
        for i in range(1, self.MODEL.nbodies):
            parameters[names[i]] = self.MODEL.inertias[i].toDynamicParameters()

        while True:
            pub.publish(parameters)
            if sub.has_new_msg():
                break

        _parameters = sub.get_parameters()
        for i in range(1, self.MODEL.nbodies):
            self.assertTrue(
                np.allclose(_parameters[names[i]], parameters[names[i]], atol=1e-9),
                "Wrong parameters in "
                + names[i]
                + "\n"
                + "Published parameters:\n"
                + str(parameters[names[i]])
                + "\n"
                + "Subscribed parameters:\n"
                + str(_parameters[names[i]]),
            )

class SampleHumanoidTest(TestInertialParametersAbstract):
    MODEL = pinocchio.buildSampleModelHumanoid()

class SampleManipulatorTest(TestInertialParametersAbstract):
    MODEL = pinocchio.buildSampleModelManipulator()

if __name__ == "__main__":
    test_classes_to_run = [
        SampleHumanoidTest,
        SampleManipulatorTest,
    ]
    if ROS_VERSION == 2:
        # test to be run
        loader = unittest.TestLoader()
        suites_list = []
        for test_class in test_classes_to_run:
            suite = loader.loadTestsFromTestCase(test_class)
            suites_list.append(suite)
        big_suite = unittest.TestSuite(suites_list)
        runner = unittest.TextTestRunner()
        results = runner.run(big_suite)
    else:
        for test_class in test_classes_to_run:
            rosunit.unitrun("crocoddyl_msgs", "inertial_parameters", test_class)
