#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/30/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
from pyphysx_utils.tree_robot import *
from pyphysx_utils.transformations import *
import quaternion as npq


class TreeRobotTestCase(unittest.TestCase):

    def test_joint(self):
        j0 = Joint(name='j0')
        self.assertEqual(j0.name, 'j0')
        p, q = j0.joint_transformation()
        self.assertAlmostEqual(np.linalg.norm(p), 0.)
        self.assertAlmostEqual(npq.rotation_intrinsic_distance(q, npq.one), 0.)
        p, q = j0.joint_transformation(5.)
        self.assertAlmostEqual(np.linalg.norm(p), 0.)
        self.assertAlmostEqual(npq.rotation_intrinsic_distance(q, npq.one), 0.)
        self.assertTrue(j0.is_fixed)
        self.assertFalse(j0.is_prismatic)
        self.assertFalse(j0.is_revolute)
        j2 = Joint(name='j2', joint_type='prismatic', null_value=0.1)
        p, q = j2.joint_transformation(0)
        self.assertAlmostEqual(np.linalg.norm(p), 0.)
        self.assertAlmostEqual(npq.rotation_intrinsic_distance(q, npq.one), 0.)
        p, q = j2.joint_transformation()
        self.assertAlmostEqual(np.linalg.norm(p - [0.1, 0., 0.]), 0.)
        self.assertAlmostEqual(npq.rotation_intrinsic_distance(q, npq.one), 0.)
        self.assertFalse(j2.is_fixed)
        self.assertTrue(j2.is_prismatic)
        self.assertFalse(j2.is_revolute)
        j3 = Joint(name='j3', joint_type='revolute')
        p, q = j3.joint_transformation()
        self.assertAlmostEqual(np.linalg.norm(p), 0.)
        self.assertAlmostEqual(npq.rotation_intrinsic_distance(q, npq.one), 0.)
        p, q = j3.joint_transformation(0.1)
        self.assertAlmostEqual(np.linalg.norm(p), 0.)
        self.assertAlmostEqual(npq.rotation_intrinsic_distance(q, quat_from_euler('x', [0.1])), 0.)
        self.assertFalse(j3.is_fixed)
        self.assertFalse(j3.is_prismatic)
        self.assertTrue(j3.is_revolute)

    def test_link(self):
        link0 = Link(name='l0')
        self.assertEqual(link0.name, 'l0')

    def test_root(self):
        r = TreeRobot()
        [r.add_link(Link('l{}'.format(i))) for i in range(5)]
        r.add_joint('l0', 'l1')
        r.add_joint('l0', 'l2')
        r.add_joint('l2', 'l3')
        r.add_joint('l3', 'l4')
        self.assertEqual(r.root_node.name, 'l0')

    def test_root_pose(self):
        pose = ((1, 2, 3), npq.from_rotation_vector([0.1, 0.2, 0.3]))
        r = TreeRobot()
        r.add_link(Link('l0', RigidDynamic()))
        r.attach_root_node_to_pose(pose)
        self.assert_pose(pose, r.root_pose)

    def test_root_pose_actor(self):
        act = RigidDynamic()
        r = TreeRobot()
        r.add_link(Link('l0', RigidDynamic()))
        r.attach_root_node_to_actor(act)
        pose = ((1, 2, 3), npq.from_rotation_vector([0.1, 0.2, 0.3]))
        act.set_global_pose(pose)
        self.assert_pose(pose, r.root_pose)

    def test_robot(self):
        """
        Use a robot specified in yz plane as (each link has 1m):
        l1
        |
        l0 -- l2
              |
              l3 -- l4 -- l5
        """
        r = TreeRobot()
        [r.add_link(Link('l{}'.format(i), RigidDynamic())) for i in range(6)]
        r.add_joint('l0', 'l1', Joint('j0', joint_type='fixed'), local_pose0=(0, 0, 1))
        r.add_joint('l0', 'l2', Joint('j1', joint_type='fixed'), local_pose0=(0, 1, 0))
        r.add_joint('l2', 'l3', Joint('j2', joint_type='prismatic'), local_pose0=(0, 0, -1))
        r.add_joint('l3', 'l4', Joint('j3', joint_type='revolute'), local_pose0=(0, 1, 0))
        r.add_joint('l4', 'l5', Joint('j4', joint_type='fixed'), local_pose0=(0, 1, 0))
        self.assertListEqual(r.get_joint_names(), ['j2', 'j3'])
        transformations = r.compute_link_transformations()
        self.assert_pose(transformations['l0'], unit_pose())
        self.assert_pose(transformations['l1'], (0, 0, 1))
        self.assert_pose(transformations['l2'], (0, 1, 0))
        self.assert_pose(transformations['l3'], (0, 1, -1))
        self.assert_pose(transformations['l4'], (0, 2, -1))

        transformations = r.compute_link_transformations(dict(j2=1))
        self.assert_pose(transformations['l0'], unit_pose())
        self.assert_pose(transformations['l1'], (0, 0, 1))
        self.assert_pose(transformations['l2'], (0, 1, 0))
        self.assert_pose(transformations['l3'], (1, 1, -1))
        self.assert_pose(transformations['l4'], (1, 2, -1))

        transformations = r.compute_link_transformations(dict(j2=2, j3=np.deg2rad(90)))
        self.assert_pose(transformations['l0'], unit_pose())
        self.assert_pose(transformations['l1'], (0, 0, 1))
        self.assert_pose(transformations['l2'], (0, 1, 0))
        self.assert_pose(transformations['l3'], (2, 1, -1))
        self.assert_pose(transformations['l4'], ((2, 2, -1), quat_from_euler('x', [np.deg2rad(90)])))
        self.assert_pose(transformations['l5'], ((2, 2, 0), quat_from_euler('x', [np.deg2rad(90)])))

    def assert_pose(self, current_pose, desired_pose):
        """ Assert pose based on the distances. """
        current_pose = cast_transformation(current_pose)
        desired_pose = cast_transformation(desired_pose)
        self.assertAlmostEqual(np.linalg.norm(current_pose[0] - desired_pose[0]), 0.)
        self.assertAlmostEqual(npq.rotation_intrinsic_distance(current_pose[1], desired_pose[1]), 0.)


if __name__ == '__main__':
    unittest.main()
