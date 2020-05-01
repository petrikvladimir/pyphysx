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

    def test_simulation_free_fall(self):
        actor = RigidDynamic()
        scene = Scene()
        scene.add_actor(actor)
        scene.simulate(dt=0.5, num_substeps=480)
        expected_distance = -0.5 * 9.81 * scene.simulation_time ** 2
        self.assertAlmostEqual(actor.get_global_pose()[0][2], expected_distance, places=2)


if __name__ == '__main__':
    unittest.main()
