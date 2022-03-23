#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 8/5/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import os
import unittest
from pyphysx_utils.urdf_robot_parser import *


class UrdfParserTestCase(unittest.TestCase):

    def test_urdf_load(self):
        robot = URDFRobot(urdf_path=Path(os.path.realpath(__file__)).parent.joinpath('data/test_urdf_crane.urdf'))
        self.assertEqual(4, len(robot.links))
        self.assertEqual(3, len(robot.movable_joints))

    def test_urdf_load_fixed_joint(self):
        robot = URDFRobot(urdf_path=Path(os.path.realpath(__file__)).parent.joinpath('data/test_urdf_fixed_joint.urdf'))
        self.assertEqual(2, len(robot.links))
        self.assertEqual(0, len(robot.movable_joints))
        self.assertEqual(None, robot.links['base'].parent)
        self.assertEqual(robot.links['base'], robot.links['l1'].parent)

    def test_urdf_no_joint_position_limits(self):
        robot = URDFRobot(urdf_path=Path(os.path.realpath(__file__)).parent.joinpath('data/test_urdf_limit_joint.urdf'))
        j: Joint = list(robot.movable_joints.values())[0]
        self.assertEqual(j.get_limits(), (-np.inf, np.inf))
        self.assertEqual(j.joint_type, 'revolute')

    def test_urdf_wrong_geometry(self):
        with self.assertRaises(NotImplementedError):
            URDFRobot(urdf_path=Path(os.path.realpath(__file__)).parent.joinpath('data/test_urdf_wrong_geometry.urdf'))

    def test_urdf_cyliner(self):
        robot = URDFRobot(urdf_path=Path(os.path.realpath(__file__)).parent.joinpath('data/test_urdf_cylinder.urdf'))
        link: Link = list(robot.links.values())[0]
        shape: Shape = link.actor.get_atached_shapes()[0]
        data = shape.get_shape_data()
        points = data.reshape(-1, 3)
        r = 0.1
        l = 0.6
        self.assertAlmostEqual(-r, points[:, 0].min())
        self.assertAlmostEqual(r, points[:, 0].max())
        self.assertAlmostEqual(-r, points[:, 1].min())
        self.assertAlmostEqual(r, points[:, 1].max())
        self.assertAlmostEqual(-l / 2, points[:, 2].min())
        self.assertAlmostEqual(l / 2, points[:, 2].max())

    def test_urdf_dae(self):
        robot = URDFRobot(urdf_path=Path(os.path.realpath(__file__)).parent.joinpath('data/test_urdf_dae.urdf'))
        link: Link = list(robot.links.values())[0]
        shape: Shape = link.actor.get_atached_shapes()[0]
        self.assertTrue('visual_mesh' in shape.get_user_data())

if __name__ == '__main__':
    unittest.main()
