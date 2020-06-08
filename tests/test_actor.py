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


class ActorTest(unittest.TestCase):

    def test_global_pose(self):
        actor = RigidDynamic()
        actor.set_global_pose(([0, 2, 1], npq.one))
        p, q = actor.get_global_pose()

        np.testing.assert_almost_equal(p, [0, 2, 1])
        self.assertTrue(npq.isclose(q, npq.one))

        actor.set_global_pose(([0, 2, 1], [1, 0, 0, 1]))
        p, q = actor.get_global_pose()
        np.testing.assert_almost_equal(p, [0, 2, 1])
        np.testing.assert_almost_equal(npq.as_float_array(q), np.array([1, 0, 0, 1]) / np.sqrt(2))  # is normalized

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

    def test_dampings(self):
        actor = RigidDynamic()
        actor.set_linear_damping(0.1)
        actor.set_angular_damping(0.2)
        self.assertAlmostEqual(0.1, actor.get_linear_damping())
        self.assertAlmostEqual(0.2, actor.get_angular_damping())

    def test_userdata(self):
        name1 = "asdf"
        actor = RigidDynamic()
        self.assertEqual(actor.get_user_data(), None)
        actor.set_user_data(name1)
        self.assertEqual("asdf", actor.get_user_data())
        del name1
        self.assertEqual("asdf", actor.get_user_data())
        actor.set_user_data("2")
        other_actors = [RigidDynamic() for _ in range(10)]
        other_actors2 = [RigidStatic() for _ in range(10)]
        other_actors2[2].set_user_data('51')
        self.assertEqual("2", actor.get_user_data())
        self.assertEqual("51", other_actors2[2].get_user_data())

    def test_userdata_mutable(self):
        import sys
        data = {'color': 0, 'auto': 'no'}
        actor = RigidDynamic()
        self.assertEqual(actor.get_user_data(), None)
        self.assertEqual(sys.getrefcount(data), 2)  # original data + one in refcount
        actor.set_user_data(data)
        self.assertEqual(sys.getrefcount(data), 3)  # original data + one in refcount + one in shape
        del data
        self.assertEqual(sys.getrefcount(actor.get_user_data()), 2)  # one in refcount + one in shape
        new_data = actor.get_user_data()
        self.assertEqual(sys.getrefcount(new_data), 3)  # new data + one in refcount + one in shape
        self.assertFalse(new_data is None)
        self.assertEqual("no", new_data['auto'])
        self.assertEqual(0, new_data['color'])
        new_data['color'] = 1
        self.assertEqual(1, actor.get_user_data()['color'])

    def test_overlap(self):
        a1 = RigidDynamic()
        a1.attach_shape(Shape.create_box([0.1] * 3, Material()))
        a2 = RigidDynamic()
        a2.attach_shape(Shape.create_box([0.1] * 3, Material()))

        s3 = Shape.create_box([0.1] * 3, Material())
        s3.set_local_pose([0., 0., -0.3])
        a3 = RigidDynamic()
        a3.attach_shape(s3)
        self.assertTrue(a1.overlaps(a2))
        self.assertFalse(a1.overlaps(a3))
        a2.set_global_pose([0, 0., 0.3])
        a3.set_global_pose([0, 0., 0.3])
        self.assertFalse(a1.overlaps(a2))
        self.assertTrue(a1.overlaps(a3))

    def test_overlap_plane(self):
        a1 = RigidDynamic()
        a1.attach_shape(Shape.create_box([0.1] * 3, Material()))
        a2 = RigidStatic.create_plane(Material())
        self.assertTrue(a1.overlaps(a2))
        a1.set_global_pose((0., 0., 0.1 + 1e-8))
        self.assertFalse(a1.overlaps(a2))
        a1.set_global_pose((0., 0., -0.2))
        self.assertTrue(a1.overlaps(a2))  # plane is treated as a solid half-plane

    def test_velocity(self):
        scene = Scene()
        a = RigidDynamic()
        a.attach_shape(Shape.create_box([0.1] * 3, Material()))
        a.disable_gravity()
        scene.add_actor(a)
        a.set_max_linear_velocity(0.05)
        a.set_max_angular_velocity(0.1)
        a.set_linear_velocity([0, 0, 0.1])
        a.set_angular_velocity([0, 0, 0.2])
        self.assertAlmostEqual(np.linalg.norm(a.get_linear_velocity() - [0, 0, 0.1]), 0.)
        self.assertAlmostEqual(np.linalg.norm(a.get_angular_velocity() - [0, 0, 0.2]), 0.)
        scene.simulate()  # maximum velocities should be obey after simulation
        self.assertAlmostEqual(np.linalg.norm(a.get_linear_velocity() - [0, 0, 0.05]), 0.)
        self.assertAlmostEqual(np.linalg.norm(a.get_angular_velocity() - [0, 0, 0.1]), 0.)


if __name__ == '__main__':
    unittest.main()
