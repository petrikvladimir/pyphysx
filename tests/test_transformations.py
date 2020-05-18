#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
import numpy as np
from pyphysx_utils.transformations import *


def get_t_matrix(pos, rot):
    pos = np.array(pos)
    r = rot.as_matrix()
    t = np.append(r, pos.reshape((3, 1)), axis=1)
    return np.append(t, np.array([0, 0, 0, 1]).reshape(1, 4), axis=0)


class TransformationTestCase(unittest.TestCase):

    def test_multiply_transform(self):
        rot1 = Rotation.from_euler('xyz', [0.1, 0.5, 0.4])
        rot2 = Rotation.from_euler('xyz', [0.5, 0.4, 0.3])
        pos1 = [0.12, -0.5, 0.3]
        pos2 = [0.65, 0.1, -0.2]

        t1 = get_t_matrix(pos1, rot1)
        t2 = get_t_matrix(pos2, rot2)

        tn = np.matmul(t1, t2)
        pos, quat = multiply_transformations(pos1, rot1.as_quat(), pos2, rot2.as_quat())
        tm = get_t_matrix(pos, Rotation.from_quat(quat))

        identity = np.matmul(np.linalg.inv(tn), tm)
        self.assertAlmostEqual(0., np.linalg.norm(identity - np.eye(4)))

    def test_inverse(self):
        pos = [0.12, -0.5, 0.3]
        rot = Rotation.from_euler('xyz', [0.1, 0.5, 0.4])
        tn = get_t_matrix(pos, rot)
        pos, quat = inverse_transform(pos, rot.as_quat())
        tm = get_t_matrix(pos, Rotation.from_quat(quat))
        identity = np.matmul(tn, tm)
        self.assertAlmostEqual(0., np.linalg.norm(identity - np.eye(4)))


if __name__ == '__main__':
    unittest.main()
