#!/usr/bin/env python

###############################################################################
# BSD 3-Clause License
#
# Copyright (C) 2023-2024, Heriot-Watt University
# Copyright note valid unless otherwise stated in individual files.
# All rights reserved.
###############################################################################

import os
import unittest

import numpy as np
import pinocchio

ROS_VERSION = int(os.environ["ROS_VERSION"])
if ROS_VERSION == 1:
    import rosunit

from crocoddyl_ros import (
    getRootJointId,
    getRootNq,
    getRootNv,
    getBodyInertialParameters,
    updateBodyInertialParameters,
)


class TestModelAbstract(unittest.TestCase):
    MODEL = None
    ROOT_NQ = None
    ROOT_NV = None
    LOCKED_JOINTS = None

    def test_root(self):
        root_joint_id = getRootJointId(self.MODEL)
        root_nq = getRootNq(self.MODEL)
        root_nv = getRootNv(self.MODEL)
        self.assertEqual(self.ROOT_JOINT_ID, root_joint_id, "Wrong root's joint id")
        self.assertEqual(self.ROOT_NQ, root_nq, "Wrong root's nq dimension")
        self.assertEqual(self.ROOT_NV, root_nv, "Wrong root's nv dimension")

    def test_root_with_reduced_model(self):
        qref = pinocchio.randomConfiguration(self.MODEL)
        reduced_model = pinocchio.buildReducedModel(
            self.MODEL, [self.MODEL.getJointId(name) for name in self.LOCKED_JOINTS], qref
        )
        root_joint_id = getRootJointId(reduced_model)
        root_nq = getRootNq(reduced_model)
        root_nv = getRootNv(reduced_model)
        self.assertEqual(self.ROOT_JOINT_ID, root_joint_id, "Wrong reduced root's joint id")
        self.assertEqual(self.ROOT_NQ, root_nq, "Wrong reduced root's nq dimension")
        self.assertEqual(self.ROOT_NV, root_nv, "Wrong reduced root's nv dimension")

    def test_get_joint_inertial_parameters(self):
        frame_names = [f.name for f in self.MODEL.frames if f.type == pinocchio.JOINT]
        for frame_name in frame_names:
            joint_id = self.MODEL.getJointId(frame_name)
            parameters = pinocchio.Inertia.Random().toDynamicParameters()
            self.MODEL.inertias[joint_id] = pinocchio.Inertia.FromDynamicParameters(parameters)
            new_parameters = getBodyInertialParameters(self.MODEL, frame_name)
            self.assertTrue(
                np.allclose(
                    parameters,
                    new_parameters,
                    atol=1e-9,
                ),
                "Wrong updated inertial parameters in frame "
                + frame_name
                + "\n"
                + "desired:\n"
                + str(parameters)
                + "\nobtained:\n"
                + str(new_parameters),
            )

    def test_get_joint_inertial_parameters_with_reduced_model(self):
        qref = pinocchio.randomConfiguration(self.MODEL)
        reduced_model = pinocchio.buildReducedModel(
            self.MODEL, [self.MODEL.getJointId(name) for name in self.LOCKED_JOINTS], qref
        )
        frame_names = [f.name for f in reduced_model.frames if f.type == pinocchio.JOINT]
        for frame_name in frame_names:
            joint_id = reduced_model.getJointId(frame_name)
            parameters = pinocchio.Inertia.Random().toDynamicParameters()
            reduced_model.inertias[joint_id] = pinocchio.Inertia.FromDynamicParameters(parameters)
            new_parameters = getBodyInertialParameters(reduced_model, frame_name)
            self.assertTrue(
                np.allclose(
                    parameters,
                    new_parameters,
                    atol=1e-9,
                ),
                "Wrong updated inertial parameters in frame "
                + frame_name
                + "\n"
                + "desired:\n"
                + str(parameters)
                + "\nobtained:\n"
                + str(new_parameters),
            )

    def test_update_body_inertial_parameters(self):
        qref = pinocchio.randomConfiguration(self.MODEL)
        reduced_model = pinocchio.buildReducedModel(
            self.MODEL, [self.MODEL.getJointId(name) for name in self.LOCKED_JOINTS], qref
        )
        if pinocchio.__version__ >= "2.7.1":
            frame_names = [f.name for f in reduced_model.frames if f.name != "universe" and (f.type == pinocchio.BODY or f.type == pinocchio.JOINT or f.type == pinocchio.FIXED_JOINT)]
        else:
            frame_names = [f.name for f in reduced_model.frames if f.type == pinocchio.JOINT]
        for frame_name in frame_names:
            parameters = pinocchio.Inertia.Random().toDynamicParameters()
            updateBodyInertialParameters(reduced_model, frame_name, parameters)
            new_parameters = getBodyInertialParameters(reduced_model, frame_name)
            self.assertTrue(
                np.allclose(
                    parameters,
                    new_parameters,
                    atol=1e-9,
                ),
                "Wrong updated inertial parameters in frame "
                + frame_name
                + "\n"
                + "desired:\n"
                + str(parameters)
                + "\nobtained:\n"
                + str(new_parameters),
            )

    def test_update_body_inertial_parameters_with_reduced_model(self):
        if pinocchio.__version__ >= "2.7.1":
            frame_names = [f.name for f in self.MODEL.frames if f.name != "universe" and (f.type == pinocchio.BODY or f.type == pinocchio.JOINT or f.type == pinocchio.FIXED_JOINT)]
        else:
            frame_names = [f.name for f in self.MODEL.frames if f.type == pinocchio.JOINT]
        for frame_name in frame_names:
            parameters = pinocchio.Inertia.Random().toDynamicParameters()
            updateBodyInertialParameters(self.MODEL, frame_name, parameters)
            new_parameters = getBodyInertialParameters(self.MODEL, frame_name)
            self.assertTrue(
                np.allclose(
                    parameters,
                    new_parameters,
                    atol=1e-9,
                ),
                "Wrong updated inertial parameters in frame "
                + frame_name
                + "\n"
                + "desired:\n"
                + str(parameters)
                + "\nobtained:\n"
                + str(new_parameters),
            )

class SampleHumanoidTest(TestModelAbstract):
    MODEL = pinocchio.buildSampleModelHumanoid()
    ROOT_JOINT_ID = 1
    ROOT_NQ = 7
    ROOT_NV = 6
    LOCKED_JOINTS = ["larm_elbow_joint", "rarm_elbow_joint"]

class SampleManipulatorTest(TestModelAbstract):
    MODEL = pinocchio.buildSampleModelManipulator()
    ROOT_JOINT_ID = 0
    ROOT_NQ = 0
    ROOT_NV = 0
    LOCKED_JOINTS = ["wrist1_joint", "wrist2_joint"]

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
            rosunit.unitrun("crocoddyl_msgs", "model", test_class)