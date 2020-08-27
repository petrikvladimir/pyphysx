#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 8/24/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import numpy as np
from pyrender.trackball import Trackball


class RoboticTrackball(Trackball):

    def __init__(self, pose, size, scale, target=np.array([0.0, 0.0, 0.0])):
        super().__init__(pose, size, scale, target)

    def drag(self, point):
        point = np.array(point, dtype=np.float32)
        dx, dy = point - self._pdown
        mindim = 0.3 * np.min(self._size)
        target = self._target
        eye = self._pose[:3, 3].flatten()

        if self._state == self.STATE_ROTATE:
            d, a, e = self.cartesian_to_spherical(eye, target)
            a -= dx * 5e-1 / mindim
            e += dy * 5e-1 / mindim
            e = np.clip(e, np.deg2rad(5), np.deg2rad(175))  # prevents singular position
            self._n_pose[:3, 3] = self.spherical_to_cartesian(d, a, e, target)
            self._n_pose[:3, :3] = self.look_at_rotation(eye=self._n_pose[:3, 3], target=target, up=[0, 0, 1])
        else:
            super().drag(point)

    @staticmethod
    def spherical_to_cartesian(d, a, e, target=np.zeros(3)):
        return np.array([d * np.sin(e) * np.cos(a), d * np.sin(e) * np.sin(a), d * np.cos(e)]) + target

    @staticmethod
    def cartesian_to_spherical(eye_pos, target):
        dp = eye_pos - target
        r = np.linalg.norm(dp)
        return r, np.arctan2(dp[1], dp[0]), np.arccos(dp[2] / r)

    @staticmethod
    def look_at_rotation(eye, target, up):
        f = (target - eye)
        f = f / np.linalg.norm(f)
        s = np.cross(f, up)
        s = s / np.linalg.norm(s)
        u = np.cross(s, f)
        u = u / np.linalg.norm(u)
        m = np.zeros((3, 3))
        m[0, :] = s
        m[1, :] = u
        m[2, :] = -f
        return m.T
