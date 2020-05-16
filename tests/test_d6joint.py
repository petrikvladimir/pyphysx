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
        j = D6Joint(a1, a2, local_pos0=(0., 3., 0.), local_pos1=(0., -5., 0.))
        self.assertAlmostEqual(j.get_local_pose(0)[0][1], 3.)
        self.assertAlmostEqual(j.get_local_pose(1)[0][1], -5.)
