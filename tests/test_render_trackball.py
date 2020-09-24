#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 9/24/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>


import unittest
from pyphysx_render.pyrender_trackball import RoboticTrackball
import numpy as np


class TestRenderTrackball(unittest.TestCase):
    def test_spherical_coordinates(self):
        pos = RoboticTrackball.spherical_to_cartesian(1, np.pi / 2, np.pi / 4, target=np.zeros(3))
        d, a, e = RoboticTrackball.cartesian_to_spherical(pos, target=np.zeros(3))
        self.assertAlmostEqual(d, 1.)
        self.assertAlmostEqual(a, np.pi / 2)
        self.assertAlmostEqual(e, np.pi / 4)

    def test_spherical_coordinates_with_target(self):
        target = np.array([1, 2, 3])
        pos = RoboticTrackball.spherical_to_cartesian(1, np.pi / 2, np.pi / 4, target=target)
        d, a, e = RoboticTrackball.cartesian_to_spherical(pos, target=target)
        self.assertAlmostEqual(d, 1.)
        self.assertAlmostEqual(a, np.pi / 2)
        self.assertAlmostEqual(e, np.pi / 4)


if __name__ == '__main__':
    unittest.main()
