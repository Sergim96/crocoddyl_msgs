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
    WholeBodyTrajectoryRosPublisher,
    WholeBodyTrajectoryRosSubscriber,
    toReduced,
    getRootNv,
)


class TestWholeBodyTrajectoryAbstract(unittest.TestCase):
    MODEL = None
    LOCKED_JOINTS = None

    def setUp(self) -> None:
        if ROS_VERSION == 2:
            if not rclpy.ok():
                rclpy.init()
        else:
            rospy.init_node("crocoddyl_ros", anonymous=True)
        # Create random trajectories
        h = random.uniform(0.1, 0.2)
        self.ts = np.arange(0.0, 1.0, h).tolist()
        N = len(self.ts)
        self.ps, self.pds, self.fs, self.ss = [], [], [], []
        for _ in range(N):
            self.ps.append(
                {
                    "lleg_effector_body": pinocchio.SE3.Random(),
                    "rleg_effector_body": pinocchio.SE3.Random(),
                }
            )
            self.pds.append(
                {
                    "lleg_effector_body": pinocchio.Motion.Random(),
                    "rleg_effector_body": pinocchio.Motion.Random(),
                }
            )
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
            self.fs.append(
                {
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
            )
            self.ss.append(
                {
                    "lleg_effector_body": [np.random.rand(3), random.uniform(0, 1)],
                    "rleg_effector_body": [np.random.rand(3), random.uniform(0, 1)],
                }
            )

    def test_publisher_without_contact(self):
        sub = WholeBodyTrajectoryRosSubscriber(self.MODEL, "whole_body_trajectory_without_contact")
        pub = WholeBodyTrajectoryRosPublisher(self.MODEL, "whole_body_trajectory_without_contact")
        time.sleep(1)
        # publish whole-body trajectory messages
        N = len(self.ts)
        xs = []
        nv_root = getRootNv(self.MODEL)
        for _ in range(N):
            q = pinocchio.randomConfiguration(self.MODEL)
            q[:3] = np.random.rand(3)
            v = np.random.rand(self.MODEL.nv)
            xs.append(np.hstack([q, v]))
        us = [np.random.rand(self.MODEL.nv - nv_root) for _ in range(N)]
        while True:
            pub.publish(self.ts, xs, us)
            if sub.has_new_msg():
                break
        # get whole-body trajectory
        _ts, _xs, _us, _, _, _, _ = sub.get_trajectory()
        for i in range(N):
            self.assertEqual(self.ts[i], _ts[i], "Wrong time interval at " + str(i))
            self.assertTrue(
                np.allclose(xs[i], _xs[i], atol=1e-9), "Wrong x at " + str(i)
            )
            self.assertTrue(np.allclose(us, _us, atol=1e-9), "Wrong u at " + str(i))

    def test_communication(self):
        sub = WholeBodyTrajectoryRosSubscriber(self.MODEL, "whole_body_trajectory")
        pub = WholeBodyTrajectoryRosPublisher(self.MODEL, "whole_body_trajectory")
        time.sleep(1)
        # publish whole-body trajectory messages
        N = len(self.ts)
        xs = []
        for _ in range(N):
            q = pinocchio.randomConfiguration(self.MODEL)
            q[:3] = np.random.rand(3)
            v = np.random.rand(self.MODEL.nv)
            xs.append(np.hstack([q, v]))
        nv_root = getRootNv(self.MODEL)
        us = [np.random.rand(self.MODEL.nv - nv_root) for _ in range(N)]
        while True:
            pub.publish(self.ts, xs, us, self.ps, self.pds, self.fs, self.ss)
            if sub.has_new_msg():
                break
        # get whole-body trajectory
        _ts, _xs, _us, _ps, _pds, _fs, _ss = sub.get_trajectory()
        for i in range(N):
            self.assertEqual(self.ts[i], _ts[i], "Wrong time interval at " + str(i))
            self.assertTrue(
                np.allclose(xs[i], _xs[i], atol=1e-9), "Wrong x at " + str(i)
            )
            self.assertTrue(np.allclose(us, _us, atol=1e-9), "Wrong u at " + str(i))
            for name in self.ps[i]:
                M, [_t, _R] = self.ps[i][name], _ps[i][name]
                F, _F = self.fs[i][name], _fs[i][name]
                S, _S = self.ss[i][name], _ss[i][name]
                self.assertTrue(
                    np.allclose(M.translation, _t, atol=1e-9),
                    "Wrong contact translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(M.rotation, _R, atol=1e-9),
                    "Wrong contact rotation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(self.pds[i][name].vector, _pds[i][name], atol=1e-9),
                    "Wrong contact velocity translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(F[0], _F[0], atol=1e-9),
                    "Wrong contact wrench translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    F[1] == _F[1], "Wrong contact type at " + name + ", " + str(i)
                )
                self.assertTrue(
                    F[2] == _F[2], "Wrong contact status at " + name + ", " + str(i)
                )
                self.assertTrue(
                    np.allclose(S[0], _S[0], atol=1e-9),
                    "Wrong contact surface translation at " + name + ", " + str(i),
                )
                self.assertEqual(
                    S[1],
                    _S[1],
                    "Wrong contact friction coefficient at " + name + ", " + str(i),
                )

    def test_communication_with_reduced_model(self):
        qref = pinocchio.randomConfiguration(self.MODEL)
        reduced_model = pinocchio.buildReducedModel(
            self.MODEL, [self.MODEL.getJointId(name) for name in self.LOCKED_JOINTS], qref
        )
        sub = WholeBodyTrajectoryRosSubscriber(
            self.MODEL, self.LOCKED_JOINTS, qref, "reduced_whole_body_trajectory"
        )
        pub = WholeBodyTrajectoryRosPublisher(
            self.MODEL, self.LOCKED_JOINTS, qref, "reduced_whole_body_trajectory"
        )
        time.sleep(1)
        # publish whole-body trajectory messages
        N = len(self.ts)
        xs, us = [], []
        nv_root = getRootNv(self.MODEL)
        for _ in range(N):
            q = pinocchio.randomConfiguration(self.MODEL)
            q[:3] = np.random.rand(3)
            v = np.random.rand(self.MODEL.nv)
            tau = np.random.rand(self.MODEL.nv - nv_root)
            q, v, tau = toReduced(self.MODEL, reduced_model, q, v, tau)
            xs.append(np.hstack([q, v]))
            us.append(tau)
        while True:
            pub.publish(self.ts, xs, us, self.ps, self.pds, self.fs, self.ss)
            if sub.has_new_msg():
                break
        # get whole-body trajectory
        _ts, _xs, _us, _ps, _pds, _fs, _ss = sub.get_trajectory()
        for i in range(N):
            self.assertEqual(self.ts[i], _ts[i], "Wrong time interval at " + str(i))
            self.assertTrue(
                np.allclose(xs[i], _xs[i], atol=1e-9), "Wrong x at " + str(i)
            )
            self.assertTrue(np.allclose(us, _us, atol=1e-9), "Wrong u at " + str(i))
            for name in self.ps[i]:
                M, [_t, _R] = self.ps[i][name], _ps[i][name]
                F, _F = self.fs[i][name], _fs[i][name]
                S, _S = self.ss[i][name], _ss[i][name]
                self.assertTrue(
                    np.allclose(M.translation, _t, atol=1e-9),
                    "Wrong contact translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(M.rotation, _R, atol=1e-9),
                    "Wrong contact rotation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(self.pds[i][name].vector, _pds[i][name], atol=1e-9),
                    "Wrong contact velocity translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(F[0], _F[0], atol=1e-9),
                    "Wrong contact wrench translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    F[1] == _F[1], "Wrong contact type at " + name + ", " + str(i)
                )
                self.assertTrue(
                    F[2] == _F[2], "Wrong contact status at " + name + ", " + str(i)
                )
                self.assertTrue(
                    np.allclose(S[0], _S[0], atol=1e-9),
                    "Wrong contact surface translation at " + name + ", " + str(i),
                )
                self.assertEqual(
                    S[1],
                    _S[1],
                    "Wrong contact friction coefficient at " + name + ", " + str(i),
                )

    def test_communication_with_non_locked_joints(self):
        locked_joints = []
        qref = pinocchio.randomConfiguration(self.MODEL)
        reduced_model = pinocchio.buildReducedModel(
            self.MODEL, [self.MODEL.getJointId(name) for name in locked_joints], qref
        )
        sub = WholeBodyTrajectoryRosSubscriber(
            self.MODEL, locked_joints, qref, "non_locked_whole_body_trajectory"
        )
        pub = WholeBodyTrajectoryRosPublisher(
            self.MODEL, locked_joints, qref, "non_locked_whole_body_trajectory"
        )
        time.sleep(1)
        # publish whole-body trajectory messages
        N = len(self.ts)
        xs, us = [], []
        nv_root = getRootNv(self.MODEL)
        for _ in range(N):
            q = pinocchio.randomConfiguration(self.MODEL)
            q[:3] = np.random.rand(3)
            v = np.random.rand(self.MODEL.nv)
            tau = np.random.rand(self.MODEL.nv - nv_root)
            q, v, tau = toReduced(self.MODEL, reduced_model, q, v, tau)
            xs.append(np.hstack([q, v]))
            us.append(tau)
        while True:
            pub.publish(self.ts, xs, us, self.ps, self.pds, self.fs, self.ss)
            if sub.has_new_msg():
                break
        # get whole-body trajectory
        _ts, _xs, _us, _ps, _pds, _fs, _ss = sub.get_trajectory()
        for i in range(N):
            self.assertEqual(self.ts[i], _ts[i], "Wrong time interval at " + str(i))
            self.assertTrue(
                np.allclose(xs[i], _xs[i], atol=1e-9), "Wrong x at " + str(i)
            )
            self.assertTrue(np.allclose(us, _us, atol=1e-9), "Wrong u at " + str(i))
            for name in self.ps[i]:
                M, [_t, _R] = self.ps[i][name], _ps[i][name]
                F, _F = self.fs[i][name], _fs[i][name]
                S, _S = self.ss[i][name], _ss[i][name]
                self.assertTrue(
                    np.allclose(M.translation, _t, atol=1e-9),
                    "Wrong contact translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(M.rotation, _R, atol=1e-9),
                    "Wrong contact rotation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(self.pds[i][name].vector, _pds[i][name], atol=1e-9),
                    "Wrong contact velocity translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    np.allclose(F[0], _F[0], atol=1e-9),
                    "Wrong contact wrench translation at " + name + ", " + str(i),
                )
                self.assertTrue(
                    F[1] == _F[1], "Wrong contact type at " + name + ", " + str(i)
                )
                self.assertTrue(
                    F[2] == _F[2], "Wrong contact status at " + name + ", " + str(i)
                )
                self.assertTrue(
                    np.allclose(S[0], _S[0], atol=1e-9),
                    "Wrong contact surface translation at " + name + ", " + str(i),
                )
                self.assertEqual(
                    S[1],
                    _S[1],
                    "Wrong contact friction coefficient at " + name + ", " + str(i),
                )

    def test_update_model(self):
        sub = WholeBodyTrajectoryRosSubscriber(self.MODEL, "whole_body_trajectory_update_model")
        pub = WholeBodyTrajectoryRosPublisher(self.MODEL, "whole_body_trajectory_update_model")
        time.sleep(1)
        # publish whole-body state messages
        if pinocchio.__version__ >= "2.7.1":
            frame_names = [f.name for f in self.MODEL.frames if f.name != "universe" and (f.type == pinocchio.BODY or f.type == pinocchio.JOINT or f.type == pinocchio.FIXED_JOINT)]
        else:
            frame_names = [f.name for f in self.MODEL.frames if f.type == pinocchio.JOINT]
        new_parameters = []
        for name in frame_names:
            psi = pinocchio.Inertia.Random().toDynamicParameters()
            new_parameters.append(psi)
            pub.update_body_inertial_parameters(name, psi)
            sub.update_body_inertial_parameters(name, psi)

        for i, name in enumerate(frame_names):
            pub_parameters = pub.get_body_inertial_parameters(name)
            sub_parameters = sub.get_body_inertial_parameters(name)
            self.assertTrue(
                np.allclose(
                    pub_parameters,
                    new_parameters[i],
                    atol=1e-9,
                ),
                "Wrong publisher's inertial parameters in frame "
                + name
                + "\n"
                + "desired:\n"
                + str(new_parameters[i])
                + "obtained:\n"
                + str(pub_parameters),
            )
            self.assertTrue(
                np.allclose(
                    sub_parameters,
                    new_parameters[i],
                    atol=1e-9,
                ),
                "Wrong subscriber's inertial parameters in frame "
                + name
                + "\n"
                + "desired:\n"
                + str(new_parameters[i])
                + "obtained:\n"
                + str(sub_parameters),
            )

    def test_update_reduced_model(self):
        qref = pinocchio.randomConfiguration(self.MODEL)
        reduced_model = pinocchio.buildReducedModel(
            self.MODEL, [self.MODEL.getJointId(name) for name in self.LOCKED_JOINTS], qref
        )
        sub = WholeBodyTrajectoryRosSubscriber(reduced_model, "whole_body_trajectory_update_model")
        pub = WholeBodyTrajectoryRosPublisher(reduced_model, "whole_body_trajectory_update_model")
        time.sleep(1)
        # publish whole-body state messages
        if pinocchio.__version__ >= "2.7.1":
            frame_names = [f.name for f in reduced_model.frames if f.name != "universe" and (f.type == pinocchio.BODY or f.type == pinocchio.JOINT or f.type == pinocchio.FIXED_JOINT)]
        else:
            frame_names = [f.name for f in reduced_model.frames if f.type == pinocchio.JOINT]
        new_parameters = []
        for name in frame_names:
            psi = pinocchio.Inertia.Random().toDynamicParameters()
            new_parameters.append(psi)
            pub.update_body_inertial_parameters(name, psi)
            sub.update_body_inertial_parameters(name, psi)

        for i, name in enumerate(frame_names):
            pub_parameters = pub.get_body_inertial_parameters(name)
            sub_parameters = sub.get_body_inertial_parameters(name)
            self.assertTrue(
                np.allclose(
                    pub_parameters,
                    new_parameters[i],
                    atol=1e-9,
                ),
                "Wrong publisher's inertial parameters in frame "
                + name
                + "\n"
                + "desired:\n"
                + str(new_parameters[i])
                + "obtained:\n"
                + str(pub_parameters),
            )
            self.assertTrue(
                np.allclose(
                    sub_parameters,
                    new_parameters[i],
                    atol=1e-9,
                ),
                "Wrong subscriber's inertial parameters in frame "
                + name
                + "\n"
                + "desired:\n"
                + str(new_parameters[i])
                + "obtained:\n"
                + str(sub_parameters),
            )

class SampleHumanoidTest(TestWholeBodyTrajectoryAbstract):
    MODEL = pinocchio.buildSampleModelHumanoid()
    LOCKED_JOINTS = ["larm_elbow_joint", "rarm_elbow_joint"]

class SampleManipulatorTest(TestWholeBodyTrajectoryAbstract):
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
