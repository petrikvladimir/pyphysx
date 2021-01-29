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

    def test_rotate(self):
        pose = np.eye(4)
        pose[:3, 3] = RoboticTrackball.spherical_to_cartesian(3., 0., np.deg2rad(45.), target=np.zeros(3))
        pose[:3, :3] = RoboticTrackball.look_at_rotation(eye=pose[:3, 3], target=np.zeros(3), up=[0, 0, 1])
        trackball = RoboticTrackball(pose, (640, 480), 1.)
        trackball.set_state(RoboticTrackball.STATE_ROTATE)
        trackball.drag([10, 20])  # 10px in azimuth, 20px in elevation
        npose = trackball._n_pose
        d, a, e = RoboticTrackball.cartesian_to_spherical(npose[:3, 3], trackball._target)
        self.assertAlmostEqual(d, 3.)
        self.assertAlmostEqual(a, 0. - 10 * 5e-1 / (0.3 * 480))
        self.assertAlmostEqual(e, np.deg2rad(45.) + 20 * 5e-1 / (0.3 * 480))

    def test_move_target0deg(self):
        pose = np.eye(4)
        pose[:3, 3] = RoboticTrackball.spherical_to_cartesian(3., np.deg2rad(0.), np.deg2rad(45.), target=np.zeros(3))
        pose[:3, :3] = RoboticTrackball.look_at_rotation(eye=pose[:3, 3], target=np.zeros(3), up=[0, 0, 1])
        trackball = RoboticTrackball(pose, (640, 480), 1.)
        trackball.move_target(np.array([1., 2., 3.]))
        self.assertAlmostEqual(trackball._n_target[0], 1.)
        self.assertAlmostEqual(trackball._n_target[1], 2.)
        self.assertAlmostEqual(trackball._n_target[2], 3.)

    def test_move_target90deg(self):
        pose = np.eye(4)
        pose[:3, 3] = RoboticTrackball.spherical_to_cartesian(3., np.deg2rad(90.), np.deg2rad(45.), target=np.zeros(3))
        pose[:3, :3] = RoboticTrackball.look_at_rotation(eye=pose[:3, 3], target=np.zeros(3), up=[0, 0, 1])
        trackball = RoboticTrackball(pose, (640, 480), 1.)
        trackball.move_target(np.array([1., 2., 3.]))
        self.assertAlmostEqual(trackball._n_target[0], -2.)
        self.assertAlmostEqual(trackball._n_target[1], 1.)
        self.assertAlmostEqual(trackball._n_target[2], 3.)


if __name__ == '__main__':
    unittest.main()
