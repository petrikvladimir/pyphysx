#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/1/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import numpy as np
import unittest
import sys

sys.path.append('lib')

from pyphysx import *


class ActorTest(unittest.TestCase):

    def test_global_pose(self):
        actor = RigidDynamic()
        actor.set_global_pose([0, 2, 1])
        p, q = actor.get_global_pose()
        np.testing.assert_almost_equal(p, [0, 2, 1])
        np.testing.assert_almost_equal(q, [0, 0, 0, 1])

        actor.set_global_pose([0, 2, 1], [1, 0, 0, 1])
        p, q = actor.get_global_pose()
        np.testing.assert_almost_equal(p, [0, 2, 1])
        np.testing.assert_almost_equal(q, [1, 0, 0, 1] / np.sqrt(2))  # quaternion is always normalized

    def test_mass(self):
        actor = RigidDynamic()
        actor.set_mass(0.5)
        self.assertAlmostEqual(0.5, actor.get_mass())

    def test_get_shapes(self):
        actor = RigidDynamic()
        s1 = Shape.create_box([0.1] * 3, Material())
        s2 = Shape.create_box([0.2] * 3, Material())
        actor.attach_shape(s1)
        actor.attach_shape(s2)
        shapes = actor.get_atached_shapes()
        self.assertEqual(2, len(shapes))


if __name__ == '__main__':
    unittest.main()
