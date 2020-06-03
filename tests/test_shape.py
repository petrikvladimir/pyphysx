#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/1/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import numpy as np
import unittest
import sys

sys.path.append('lib')

from pyphysx import *
import quaternion as npq


class ShapeTestCase(unittest.TestCase):

    def test_local_pose(self):
        shape = Shape.create_sphere(1., Material())
        shape.set_local_pose([0, 2, 1])
        p, q = shape.get_local_pose()
        np.testing.assert_almost_equal(p, [0, 2, 1])
        self.assertTrue(npq.isclose(q, npq.one))

        shape.set_local_pose(([0, 2, 1], [1, 0, 0, 1]))
        p, q = shape.get_local_pose()
        np.testing.assert_almost_equal(p, [0, 2, 1])
        np.testing.assert_almost_equal(npq.as_float_array(q), np.array([1, 0, 0, 1]) / np.sqrt(2))  # is  normalized

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
        self.assertEqual(shape.get_user_data(), None)
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

    def test_userdata_mutable(self):
        import sys
        data = {'color': 0, 'auto': 'no'}
        shape = Shape.create_sphere(1., Material())
        self.assertEqual(shape.get_user_data(), None)
        self.assertEqual(sys.getrefcount(data), 2)  # original data + one in refcount
        shape.set_user_data(data)
        self.assertEqual(sys.getrefcount(data), 3)  # original data + one in refcount + one in shape
        del data
        self.assertEqual(sys.getrefcount(shape.get_user_data()), 2)  # one in refcount + one in shape
        new_data = shape.get_user_data()
        self.assertEqual(sys.getrefcount(new_data), 3)  # new data + one in refcount + one in shape
        self.assertFalse(new_data is None)
        self.assertEqual("no", new_data['auto'])
        self.assertEqual(0, new_data['color'])
        new_data['color'] = 1
        self.assertEqual(1, shape.get_user_data()['color'])

        shape.set_user_data(None)
        self.assertEqual(sys.getrefcount(new_data), 2)  # new data + one in refcount

    def test_flags(self):
        shape = Shape.create_sphere(1., Material())
        self.assertTrue(shape.get_flag_value(ShapeFlag.SIMULATION_SHAPE))
        self.assertTrue(shape.get_flag_value(ShapeFlag.VISUALIZATION))
        self.assertFalse(shape.get_flag_value(ShapeFlag.TRIGGER_SHAPE))
        self.assertFalse(shape.get_flag_value(ShapeFlag.SCENE_QUERY_SHAPE))

        shape.set_flag(ShapeFlag.VISUALIZATION, False)
        self.assertFalse(shape.get_flag_value(ShapeFlag.VISUALIZATION))

    def test_get_material(self):
        mat = Material(0.5)
        shape = Shape.create_sphere(0.3, mat)
        mats = shape.get_materials()
        self.assertEqual(len(mats), 1)
        self.assertAlmostEqual(mats[0].get_static_friction(), 0.5)

    def test_box_data(self):
        s = Shape.create_box(size=[1, 2, 3], material=Material())
        data = s.get_shape_data()
        self.assertEqual(data.shape[0], 6)
        self.assertEqual(data.shape[1], 12)
        x = data[:, 0::3].flatten()
        y = data[:, 1::3].flatten()
        z = data[:, 2::3].flatten()
        self.assertAlmostEqual(np.max(x), 0.5)
        self.assertAlmostEqual(np.min(x), -0.5)
        self.assertAlmostEqual(np.max(y), 1.0)
        self.assertAlmostEqual(np.min(y), -1.0)
        self.assertAlmostEqual(np.max(z), 1.5)
        self.assertAlmostEqual(np.min(z), -1.5)

    def test_sphere_data(self):
        s = Shape.create_sphere(radius=6., material=Material())
        data = s.get_shape_data()
        self.assertEqual(data.shape[1], 12)
        points = data.reshape(-1, 3)
        dist = np.linalg.norm(points, axis=1)  # per point distance to origin should be close to radius
        self.assertAlmostEqual(np.min(dist), 6., places=2)
        self.assertAlmostEqual(np.max(dist), 6., places=2)


if __name__ == '__main__':
    unittest.main()
