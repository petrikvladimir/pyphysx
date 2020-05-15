#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/2/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import random
import numpy as np
from pyglet.gl import GL_LINES, GLfloat, glMultMatrixf
import matplotlib.colors as mcolors
from scipy.spatial.transform import Rotation


def gl_color_from_matplotlib(color=None, alpha=None, return_rgba=False):
    """ Get color from matplotlib color. If color is none, use random from tab palette."""
    color = random.choice(list(mcolors.TABLEAU_COLORS.items()))[0] if color is None else color
    if return_rgba:
        return (np.array(mcolors.to_rgba(color, alpha=alpha)) * 255.).astype(np.int)
    return (np.array(mcolors.to_rgb(color)) * 255.).astype(np.int)


def add_ground_lines(batch, dist=1., min_v=-10., max_v=10., color='tab:gray'):
    """ Add ground lines into the given batch. """
    vals = np.arange(min_v, max_v + dist / 2., dist)
    c = gl_color_from_matplotlib(color)
    for v in vals:
        sv = [min_v, v, 0]
        ev = [max_v, v, 0]
        batch.add(2, GL_LINES, None, ('v3f', (*sv, *ev)), ('c3B', (*c, *c)))
    for v in vals:
        sv = [v, min_v, 0]
        ev = [v, max_v, 0]
        batch.add(2, GL_LINES, None, ('v3f', (*sv, *ev)), ('c3B', (*c, *c)))


def add_coordinate_system(batch, pos=None, quat=None, scale=1.):
    """ Add coordinate system into the pyglet batch. """
    if pos is None:
        pos = np.zeros(3)
    if quat is None:
        quat = np.array([0, 0, 0, 1.])
    rot = Rotation.from_quat(quat)
    gx = rot.apply(np.array([1, 0, 0])) * scale
    gy = rot.apply(np.array([0, 1, 0])) * scale
    gz = rot.apply(np.array([0, 0, 1])) * scale
    cx = gl_color_from_matplotlib('tab:red')
    cy = gl_color_from_matplotlib('tab:green')
    cz = gl_color_from_matplotlib('tab:blue')
    batch.add(2, GL_LINES, None, ('v3f', (*pos, *gx)), ('c3B', (*cx, *cx)))
    batch.add(2, GL_LINES, None, ('v3f', (*pos, *gy)), ('c3B', (*cy, *cy)))
    batch.add(2, GL_LINES, None, ('v3f', (*pos, *gz)), ('c3B', (*cz, *cz)))


def gl_transform(pos, quat, scale=None):
    """ Apply gl transform (glMultMatrixf) from position, quaternion, and optional xyz scale [sx,sy,sz] or [s] """
    mat = np.zeros((4, 4))
    mat[:3, :3] = np.array(Rotation.from_quat(quat).as_matrix())
    mat[:3, 3] = np.array(pos)
    mat[3, 3] = 1
    glMultMatrixf((GLfloat * 16)(*mat.T.flatten()))
    if scale is not None:
        mat = np.diag(np.append(np.ones(3) * scale, 1))
        glMultMatrixf((GLfloat * 16)(*mat.T.flatten()))
