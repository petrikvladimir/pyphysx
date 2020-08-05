#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 8/5/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import os
import unittest
from pyphysx_utils.urdf_robot_parser import *


class UrdfParserTestCase(unittest.TestCase):

    def test_urdf_load(self):
        robot = URDFRobot(urdf_path=Path(os.path.realpath(__file__)).parent.joinpath('test_urdf_crane.urdf'))
        self.assertEqual(4, len(robot.links))
        self.assertEqual(3, len(robot.movable_joints))

    def test_urdf_load_fixed_joint(self):
        robot = URDFRobot(urdf_path=Path(os.path.realpath(__file__)).parent.joinpath('test_urdf_fixed_joint.urdf'))
        self.assertEqual(2, len(robot.links))
        self.assertEqual(0, len(robot.movable_joints))
        self.assertEqual(None, robot.links['base'].parent)
        self.assertEqual(robot.links['base'], robot.links['l1'].parent)


if __name__ == '__main__':
    unittest.main()
