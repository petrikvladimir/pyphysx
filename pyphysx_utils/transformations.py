#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from scipy.spatial.transform import Rotation


def multiply_transformations(pos1, quat1, pos2, quat2):
    """ Given two poses T_1 and T_2, represented by position and quaternion, compute T_1 * T_2. """
    rot1 = Rotation.from_quat(quat1)
    rot2 = Rotation.from_quat(quat2)
    return rot1.apply(pos2) + pos1, (rot1 * rot2).as_quat()


def inverse_transform(pos, quat):
    """ Inverse transformation. """
    rot = Rotation.from_quat(quat).inv()
    return -rot.apply(pos), rot.as_quat()
