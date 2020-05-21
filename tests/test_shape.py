#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/1/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import numpy as np
import unittest
import sys

sys.path.append('lib')

from pyphysx import *


class SceneTestCase(unittest.TestCase):

    def test_local_pose(self):
        shape = Shape.create_sphere(1., Material())
        shape.set_local_pose([0, 2, 1])
        p, q = shape.get_local_pose()
        np.testing.assert_almost_equal(p, [0, 2, 1])
        np.testing.assert_almost_equal(q, [0, 0, 0, 1])

        shape.set_local_pose([0, 2, 1], [1, 0, 0, 1])
        p, q = shape.get_local_pose()
        np.testing.assert_almost_equal(p, [0, 2, 1])
        np.testing.assert_almost_equal(q, np.array([1, 0, 0, 1]) / np.sqrt(2))  # quaternion is always normalized

    def test_convex_mesh(self):
        points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        s = Shape.create_convex_mesh_from_points(points, Material(), scale=0.5)
        self.assertEqual(s.get_shape_data().shape[0], 4)  # 4 trianglular faces
        points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0.5]])
        s = Shape.create_convex_mesh_from_points(points, Material(), scale=0.5)
        self.assertEqual(s.get_shape_data().shape[0], 6)  # additional 3 faces but one is removed from previous shape

    def test_userdata(self):
        name1 = "asdf"
        shape = Shape.create_sphere(1., Material())
        shape.set_user_data(name1)
        self.assertEqual("asdf", shape.get_user_data())
        del name1
        self.assertEqual("asdf", shape.get_user_data())
        shape.set_user_data("2")
        other_actors = [RigidDynamic() for _ in range(10)]
        other_actors2 = [RigidStatic() for _ in range(10)]
        other_actors2[2].set_user_data('51')
        self.assertEqual("2", shape.get_user_data())
        self.assertEqual("51", other_actors2[2].get_user_data())

    def test_flags(self):
        shape = Shape.create_sphere(1., Material())
        self.assertTrue(shape.get_flag_value(ShapeFlag.SIMULATION_SHAPE))
        self.assertTrue(shape.get_flag_value(ShapeFlag.VISUALIZATION))
        self.assertFalse(shape.get_flag_value(ShapeFlag.TRIGGER_SHAPE))
        self.assertFalse(shape.get_flag_value(ShapeFlag.SCENE_QUERY_SHAPE))

        shape.set_flag(ShapeFlag.VISUALIZATION, False)
        self.assertFalse(shape.get_flag_value(ShapeFlag.VISUALIZATION))


if __name__ == '__main__':
    unittest.main()
