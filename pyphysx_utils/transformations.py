#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from pyphysx import cast_transformation
import quaternion as npq


def multiply_transformations(pose1, pose2):
    """ Given two poses T_1 and T_2, represented by position and quaternion, compute T_1 * T_2. """
    if not (isinstance(pose1, tuple) and len(pose1) == 2):
        pose1 = cast_transformation(pose1)
    if not (isinstance(pose2, tuple) and len(pose2) == 2):
        pose2 = cast_transformation(pose2)
    return npq.rotate_vectors(pose1[1], pose2[0]) + pose1[0], pose1[1] * pose2[1]


def inverse_transform(pose):
    """ Inverse transformation. """
    if not (isinstance(pose, tuple) and len(pose) == 2):
        pose = cast_transformation(pose)
    qinv: npq.quaternion = pose[1].inverse()
    return -npq.rotate_vectors(qinv, pose[0]), qinv
