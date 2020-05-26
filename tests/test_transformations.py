#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
import numpy as np
from pyphysx_utils.transformations import *
from pyphysx import *
import quaternion as npq
import sys


def get_t_matrix(pose):
    p, q = cast_transformation(pose)
    t = np.eye(4)
    t[:3, :3] = npq.as_rotation_matrix(q)
    t[:3, 3] = p
    return t


class TransformationTestCase(unittest.TestCase):

    def test_casting_from_position(self):
        pref = [1., 2., 3.]
        for pin in [([1, 2, 3],), [1, 2, 3], (1, 2, 3), np.array([1, 2, 3])]:
            p, q = cast_transformation(pin)
            self.assertTrue(np.isclose(pref, p).all())
            self.assertTrue(npq.isclose(q, npq.one))

    def test_casting_from_pose(self):
        pref = [5., 3., 6.]
        qref = npq.from_rotation_vector([3., 2., 3])
        self.assertTrue(type(qref) == npq.quaternion)

        p, q = cast_transformation((pref, qref))
        self.assertTrue(np.isclose(pref, p).all())
        self.assertTrue(npq.isclose(q, qref, rtol=1e-5))

        p, q = cast_transformation((pref, [qref.w, qref.x, qref.y, qref.z]))
        self.assertTrue(np.isclose(pref, p).all())
        self.assertTrue(npq.isclose(q, qref, rtol=1e-5))

        p, q = cast_transformation((pref, (qref.w, qref.x, qref.y, qref.z)))
        self.assertTrue(np.isclose(pref, p).all())
        self.assertTrue(npq.isclose(q, qref, rtol=1e-5))

        p, q = cast_transformation([5., 3., 6., qref.w, qref.x, qref.y, qref.z])
        self.assertTrue(np.isclose(pref, p).all())
        self.assertTrue(npq.isclose(q, qref, rtol=1e-5))

    def test_multiply_transform(self):
        pose1 = (np.array((0.1, 0.2, 0.4)), npq.from_rotation_vector([0.1, 0.3, 0.5]))
        pose2 = ((0.3, 0.2, -0.4), npq.from_rotation_vector([0.5, 0.1, 0.2]))
        pose3 = multiply_transformations(pose1, pose2)

        t1 = get_t_matrix(pose1)
        t2 = get_t_matrix(pose2)
        t3 = np.matmul(t1, t2)

        identity = np.matmul(np.linalg.inv(t3), get_t_matrix(pose3))
        self.assertAlmostEqual(0., np.linalg.norm(identity - np.eye(4)), places=4)

    def test_multiply_incomplete_transform(self):
        q1 = npq.from_rotation_vector([0.1, 0.3, 0.5])
        pose1 = ((0.1, 0.2, 0.4), npq.from_rotation_vector([0.1, 0.3, 0.5]))
        pose2 = ((0.3, 0.2, -0.4), npq.one)
        pose3 = multiply_transformations([0.1, 0.2, 0.4, q1.w, q1.x, q1.y, q1.z], (0.3, 0.2, -0.4))

        t1 = get_t_matrix(pose1)
        t2 = get_t_matrix(pose2)
        t3 = np.matmul(t1, t2)

        identity = np.matmul(np.linalg.inv(t3), get_t_matrix(pose3))
        self.assertAlmostEqual(0., np.linalg.norm(identity - np.eye(4)))

    def test_inverse(self):
        pose = ((0.1, 0.2, 0.4), npq.from_rotation_vector([0.1, 0.3, 0.5]))
        tn = get_t_matrix(pose)
        inv_pose = inverse_transform(pose)
        tm = get_t_matrix(inv_pose)
        identity = np.matmul(tn, tm)
        self.assertAlmostEqual(0., np.linalg.norm(identity - np.eye(4)))

    def test_get_transformation_matrix(self):
        t = pose_to_transformation_matrix((1, 2, 3))
        expected = np.eye(4)
        expected[:3, 3] = [1, 2, 3]
        self.assertAlmostEqual(0., np.linalg.norm(t - expected), places=3)

    def test_ensure_complete_pose(self):
        pose1 = (np.array((0.1, 0.2, 0.4)), npq.from_rotation_vector([0.1, 0.3, 0.5]))
        pose1a = pose_ensure_complete(pose1)
        self.assertEqual(id(pose1), id(pose1a))

        pose2 = ((0.3, 0.2, -0.4), npq.from_rotation_vector([0.5, 0.1, 0.2]))
        pose2a = pose_ensure_complete(pose2)
        self.assertNotEqual(id(pose2), id(pose2a))

    def test_unit_pose(self):
        pose = unit_pose()
        self.assertAlmostEqual(np.linalg.norm(pose[0]), 0.)
        self.assertAlmostEqual(npq.rotation_intrinsic_distance(pose[1], npq.one), 0.)


if __name__ == '__main__':
    unittest.main()
