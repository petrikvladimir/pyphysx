#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from pyphysx import cast_transformation
import quaternion as npq
import numpy as np


def multiply_transformations(pose1, pose2):
    """ Given two poses T_1 and T_2, represented by position and quaternion, compute T_1 * T_2. """
    pose1 = pose_ensure_complete(pose1)
    pose2 = pose_ensure_complete(pose2)
    return npq.rotate_vectors(pose1[1], pose2[0]) + pose1[0], pose1[1] * pose2[1]


def inverse_transform(pose):
    """ Inverse transformation. """
    pose = pose_ensure_complete(pose)
    qinv: npq.quaternion = pose[1].inverse()
    return -npq.rotate_vectors(qinv, pose[0]), qinv


def pose_ensure_complete(pose):
    """ Ensure that the pose is complete tuple represented by (pos, quat), where pos is np.ndarray and quat is
        np quaternion."""
    if isinstance(pose, tuple) and len(pose) == 2 and isinstance(pose[0], np.ndarray) \
            and isinstance(pose[1], npq.quaternion):
        return pose
    return cast_transformation(pose)


def pose_to_transformation_matrix(pose):
    """ Convert given pose to 4x4 transformation matrix. """
    pose = pose_ensure_complete(pose)
    transformation = np.eye(4)
    transformation[:3, :3] = npq.as_rotation_matrix(pose[1])
    transformation[:3, 3] = pose[0]
    return transformation
