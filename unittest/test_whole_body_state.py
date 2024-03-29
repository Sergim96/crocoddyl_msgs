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
    ContactStatus,
    ContactType,
    WholeBodyStateRosPublisher,
    WholeBodyStateRosSubscriber,
    toReduced,
    getRootNv,
)


class TestWholeBodyStateAbstract(unittest.TestCase):
    MODEL = None
    LOCKED_JOINTS = None

    def setUp(self):
        if ROS_VERSION == 2:
            if not rclpy.ok():
                rclpy.init()
        else:
            rospy.init_node("crocoddyl_ros", anonymous=True)
        # Create random state
        self.t = random.uniform(0, 1)
        self.p = {
            "lleg_effector_body": pinocchio.SE3.Random(),
            "rleg_effector_body": pinocchio.SE3.Random(),
        }
        self.pd = {
            "lleg_effector_body": pinocchio.Motion.Random(),
            "rleg_effector_body": pinocchio.Motion.Random(),
        }
        type = random.randint(0, 1)
        if type == 0:
            contact_type = ContactType.LOCOMOTION
        else:
            contact_type = ContactType.MANIPULATION
        status = random.randint(0, 3)
        if status == 0:
            contact_status = ContactStatus.UNKNOWN
        elif status == 1:
            contact_status = ContactStatus.SEPARATION
        elif status == 2:
            contact_status = ContactStatus.STICKING
        else:
            contact_status = ContactStatus.SLIPPING
        self.f = {
            "lleg_effector_body": [
                pinocchio.Force.Random(),
                contact_type,
                contact_status,
            ],
            "rleg_effector_body": [
                pinocchio.Force.Random(),
                contact_type,
                contact_status,
            ],
        }
        self.s = {
            "lleg_effector_body": [np.random.rand(3), random.uniform(0, 1)],
            "rleg_effector_body": [np.random.rand(3), random.uniform(0, 1)],
        }

    def test_publisher_without_contact(self):
        sub = WholeBodyStateRosSubscriber(self.MODEL, "whole_body_state_without_contact")
        pub = WholeBodyStateRosPublisher(self.MODEL, "whole_body_state_without_contact")
        time.sleep(1)
        # publish whole-body state messages
        nv_root = getRootNv(self.MODEL)
        q = pinocchio.randomConfiguration(self.MODEL)
        v = np.random.rand(self.MODEL.nv)
        tau = np.random.rand(self.MODEL.nv - nv_root)
        while True:
            pub.publish(self.t, q=q, v=v, tau=tau)
            if sub.has_new_msg():
                break
        # get whole-body state
        _t, _q, _v, _tau, _, _, _, _ = sub.get_state()
        qdiff = pinocchio.difference(self.MODEL, q, _q)
        mask = ~np.isnan(qdiff)
        self.assertEqual(self.t, _t, "Wrong time interval")
        self.assertTrue(np.allclose(qdiff[mask], np.zeros(self.MODEL.nv)[mask], atol=1e-9), "Wrong q")
        self.assertTrue(np.allclose(v, _v, atol=1e-9), "Wrong v")
        self.assertTrue(np.allclose(tau, _tau, atol=1e-9), "Wrong tau")

    def test_communication(self):
        sub = WholeBodyStateRosSubscriber(self.MODEL, "whole_body_state")
        pub = WholeBodyStateRosPublisher(self.MODEL, "whole_body_state")
        time.sleep(1)
        # publish whole-body state messages
        nv_root = getRootNv(self.MODEL)
        q = pinocchio.randomConfiguration(self.MODEL)
        v = np.random.rand(self.MODEL.nv)
        tau = np.random.rand(self.MODEL.nv - nv_root)
        while True:
            pub.publish(self.t, q, v, tau, self.p, self.pd, self.f, self.s)
            if sub.has_new_msg():
                break
        # get whole-body state
        _t, _q, _v, _tau, _p, _pd, _f, _s = sub.get_state()
        qdiff = pinocchio.difference(self.MODEL, q, _q)
        mask = ~np.isnan(qdiff)
        self.assertEqual(self.t, _t, "Wrong time interval")
        self.assertTrue(np.allclose(qdiff[mask], np.zeros(self.MODEL.nv)[mask], atol=1e-9), "Wrong q")
        self.assertTrue(np.allclose(v, _v, atol=1e-9), "Wrong v")
        self.assertTrue(np.allclose(tau, _tau, atol=1e-9), "Wrong tau")
        for name in self.p:
            M, [_t, _R] = self.p[name], _p[name]
            F, _F = self.f[name], _f[name]
            S, _S = self.s[name], _s[name]
            self.assertTrue(
                np.allclose(M.translation, _t, atol=1e-9),
                "Wrong contact translation at " + name,
            )
            self.assertTrue(
                np.allclose(M.rotation, _R, atol=1e-9),
                "Wrong contact rotation at " + name,
            )
            self.assertTrue(
                np.allclose(self.pd[name].vector, _pd[name], atol=1e-9),
                "Wrong contact velocity translation at " + name,
            )
            self.assertTrue(
                np.allclose(F[0], _F[0], atol=1e-9),
                "Wrong contact wrench translation at " + name,
            )
            self.assertTrue(F[1] == _F[1], "Wrong contact type at " + name)
            self.assertTrue(F[2] == _F[2], "Wrong contact status at " + name)
            self.assertTrue(
                np.allclose(S[0], _S[0], atol=1e-9),
                "Wrong contact surface translation at " + name,
            )
            self.assertEqual(
                S[1], _S[1], "Wrong contact friction coefficient at " + name
            )

    def test_communication_with_reduced_model(self):
        qref = pinocchio.randomConfiguration(self.MODEL)
        reduced_model = pinocchio.buildReducedModel(
            self.MODEL, [self.MODEL.getJointId(name) for name in self.LOCKED_JOINTS], qref
        )
        sub = WholeBodyStateRosSubscriber(
            self.MODEL, self.LOCKED_JOINTS, qref, "reduced_whole_body_state"
        )
        pub = WholeBodyStateRosPublisher(self.MODEL, self.LOCKED_JOINTS, qref, "reduced_whole_body_state")
        time.sleep(1)
        # publish whole-body state messages
        nv_root = getRootNv(self.MODEL)
        q = pinocchio.randomConfiguration(self.MODEL)
        v = np.random.rand(self.MODEL.nv)
        tau = np.random.rand(self.MODEL.nv - nv_root)
        q, v, tau = toReduced(self.MODEL, reduced_model, q, v, tau)
        while True:
            pub.publish(self.t, q, v, tau, self.p, self.pd, self.f, self.s)
            if sub.has_new_msg():
                break
        # get whole-body state
        _t, _q, _v, _tau, _p, _pd, _f, _s = sub.get_state()
        qdiff = pinocchio.difference(reduced_model, q, _q)
        mask = ~np.isnan(qdiff)
        self.assertEqual(self.t, _t, "Wrong time interval")
        self.assertTrue(np.allclose(qdiff[mask], np.zeros(reduced_model.nv)[mask], atol=1e-9), "Wrong q")
        self.assertTrue(np.allclose(v, _v, atol=1e-9), "Wrong v")
        self.assertTrue(np.allclose(tau, _tau, atol=1e-9), "Wrong tau")
        for name in self.p:
            M, [_t, _R] = self.p[name], _p[name]
            F, _F = self.f[name], _f[name]
            S, _S = self.s[name], _s[name]
            self.assertTrue(
                np.allclose(M.translation, _t, atol=1e-9),
                "Wrong contact translation at " + name,
            )
            self.assertTrue(
                np.allclose(M.rotation, _R, atol=1e-9),
                "Wrong contact rotation at " + name,
            )
            self.assertTrue(
                np.allclose(self.pd[name].vector, _pd[name], atol=1e-9),
                "Wrong contact velocity translation at " + name,
            )
            self.assertTrue(
                np.allclose(F[0], _F[0], atol=1e-9),
                "Wrong contact wrench translation at " + name,
            )
            self.assertTrue(F[1] == _F[1], "Wrong contact type at " + name)
            self.assertTrue(F[2] == _F[2], "Wrong contact status at " + name)
            self.assertTrue(
                np.allclose(S[0], _S[0], atol=1e-9),
                "Wrong contact surface translation at " + name,
            )
            self.assertEqual(
                S[1], _S[1], "Wrong contact friction coefficient at " + name
            )

    def test_communication_with_non_locked_joints(self):
        locked_joints = []
        qref = pinocchio.randomConfiguration(self.MODEL)
        reduced_model = pinocchio.buildReducedModel(
            self.MODEL, [self.MODEL.getJointId(name) for name in locked_joints], qref
        )
        sub = WholeBodyStateRosSubscriber(
            self.MODEL, locked_joints, qref, "non_locked_whole_body_state"
        )
        pub = WholeBodyStateRosPublisher(self.MODEL, locked_joints, qref, "non_locked_whole_body_state")
        time.sleep(1)
        # publish whole-body state messages
        nv_root = getRootNv(self.MODEL)
        q = pinocchio.randomConfiguration(self.MODEL)
        v = np.random.rand(self.MODEL.nv)
        tau = np.random.rand(self.MODEL.nv - nv_root)
        q, v, tau = toReduced(self.MODEL, reduced_model, q, v, tau)
        while True:
            pub.publish(self.t, q, v, tau, self.p, self.pd, self.f, self.s)
            if sub.has_new_msg():
                break
        # get whole-body state
        _t, _q, _v, _tau, _p, _pd, _f, _s = sub.get_state()
        qdiff = pinocchio.difference(reduced_model, q, _q)
        mask = ~np.isnan(qdiff)
        self.assertEqual(self.t, _t, "Wrong time interval")
        self.assertTrue(np.allclose(qdiff[mask], np.zeros(reduced_model.nv)[mask], atol=1e-9), "Wrong q")
        self.assertTrue(np.allclose(v, _v, atol=1e-9), "Wrong v")
        self.assertTrue(np.allclose(tau, _tau, atol=1e-9), "Wrong tau")
        for name in self.p:
            M, [_t, _R] = self.p[name], _p[name]
            F, _F = self.f[name], _f[name]
            S, _S = self.s[name], _s[name]
            self.assertTrue(
                np.allclose(M.translation, _t, atol=1e-9),
                "Wrong contact translation at " + name,
            )
            self.assertTrue(
                np.allclose(M.rotation, _R, atol=1e-9),
                "Wrong contact rotation at " + name,
            )
            self.assertTrue(
                np.allclose(self.pd[name].vector, _pd[name], atol=1e-9),
                "Wrong contact velocity translation at " + name,
            )
            self.assertTrue(
                np.allclose(F[0], _F[0], atol=1e-9),
                "Wrong contact wrench translation at " + name,
            )
            self.assertTrue(F[1] == _F[1], "Wrong contact type at " + name)
            self.assertTrue(F[2] == _F[2], "Wrong contact status at " + name)
            self.assertTrue(
                np.allclose(S[0], _S[0], atol=1e-9),
                "Wrong contact surface translation at " + name,
            )
            self.assertEqual(
                S[1], _S[1], "Wrong contact friction coefficient at " + name
            )

    def test_update_model(self):
        model = pinocchio.buildSampleModelHumanoid()
        pub = WholeBodyStateRosPublisher(model, "whole_body_state_update_model")
        sub = WholeBodyStateRosPublisher(model, "whole_body_state_update_model")
        time.sleep(1)
        # publish whole-body state messages
        names = model.names.tolist()
        new_parameters = []
        for name in names:
            psi = pinocchio.Inertia().Random().toDynamicParameters()
            new_parameters.append(psi)
            pub.update_body_inertial_parameters(name, psi)
            sub.update_body_inertial_parameters(name, psi)

        for i, name in enumerate(names):
            self.assertTrue(
                np.allclose(
                    pub.get_body_inertial_parameters(name),
                    new_parameters[i],
                    atol=1e-9,
                ),
                "Wrong pub inerital parameters in body "
                + name
                + "\n"
                + "desired:\n"
                + str(new_parameters[i])
                + "obtained:\n"
                + str(pub.get_body_inertial_parameters(name)),
            )
            self.assertTrue(
                np.allclose(
                    sub.get_body_inertial_parameters(name),
                    new_parameters[i],
                    atol=1e-9,
                ),
                "Wrong sub inerital parameters in body "
                + name
                + "\n"
                + "desired:\n"
                + str(new_parameters[i])
                + "obtained:\n"
                + str(pub.get_body_inertial_parameters(name)),
            )

class SampleHumanoidTest(TestWholeBodyStateAbstract):
    MODEL = pinocchio.buildSampleModelHumanoid()
    LOCKED_JOINTS = ["larm_elbow_joint", "rarm_elbow_joint"]

class SampleManipulatorTest(TestWholeBodyStateAbstract):
    MODEL = pinocchio.buildSampleModelManipulator()
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
            rosunit.unitrun("crocoddyl_msgs", "whole_body_state", test_class)
