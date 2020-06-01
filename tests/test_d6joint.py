#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/16/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
import sys

sys.path.append('lib')

from pyphysx import *


class D6JointTest(unittest.TestCase):

    def test_local_pose(self):
        a1, a2 = RigidDynamic(), RigidDynamic()
        j = D6Joint(a1, a2, local_pose0=(0., 3., 0.), local_pose1=(0., -5., 0.))
        self.assertAlmostEqual(j.get_local_pose(0)[0][1], 3.)
        self.assertAlmostEqual(j.get_local_pose(1)[0][1], -5.)

    def test_relative_transform(self):
        """
            p0 = I, p1 = I
            T0 = (0,5,0), T1 = (0,3,0.)
            relative transform returns value for joint required to have actors at identity
        """
        a0, a1 = RigidDynamic(), RigidDynamic()
        j = D6Joint(a0, a1, local_pose0=(0., 3., 0.), local_pose1=(0., -5., 0.))
        self.assertAlmostEqual(j.get_relative_transform()[0][1], -8.)

    def test_broken_joint(self):
        scene = Scene()
        a0, a1 = RigidStatic(), RigidDynamic()
        a1.attach_shape(Shape.create_sphere(0.3, Material()))
        a1.set_mass(1.)
        a1.disable_gravity()
        scene.add_actor(a0)
        scene.add_actor(a1)
        j = D6Joint(a0, a1, local_pose0=(0., 3., 0.))
        j.set_break_force(10, 1000)
        scene.simulate()
        self.assertFalse(j.is_broken())
        a1.add_force([-9., 0, 0])
        scene.simulate()
        self.assertFalse(j.is_broken())
        a1.add_force([-11., 0, 0])
        scene.simulate()
        self.assertTrue(j.is_broken())

    def test_limits(self):
        a1, a2 = RigidDynamic(), RigidDynamic()
        j = D6Joint(a1, a2, local_pose0=(0., 3., 0.), local_pose1=(0., -5., 0.))
        j.set_linear_limit(D6Axis.Y, -1, 1)
        low, up = j.get_linear_limit(D6Axis.Y)
        self.assertAlmostEqual(low, -1)
        self.assertAlmostEqual(up, 1)
        j.set_twist_limit(-2, 2)
        low, up = j.get_twist_limit()
        self.assertAlmostEqual(low, -2)
        self.assertAlmostEqual(up, 2)
