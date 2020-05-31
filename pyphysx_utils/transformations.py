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


def unit_pose():
    """ Return unit pose (0,0,0) (1,0,0,0) """
    return np.zeros(3), npq.one


def quat_from_euler(seq='xyz', angles=None):
    """ Compute quaternion from intrinsic (e.g. 'XYZ') or extrinsic (fixed axis, e.g. 'xyz') euler angles. """
    angles = np.atleast_1d(angles)
    q = npq.one
    for s, a in zip(seq, angles):
        axis = np.array([
            1 if s.capitalize() == 'X' else 0,
            1 if s.capitalize() == 'Y' else 0,
            1 if s.capitalize() == 'Z' else 0,
        ])
        if s.isupper():
            q = q * npq.from_rotation_vector(axis * a)
        else:
            q = npq.from_rotation_vector(axis * a) * q
    return q


def quat_between_two_vectors(v, u) -> npq.quaternion:
    """ Computes rotation (represented by quaternion) that transforms unit vector v into the unit vector u.
        Based on: https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/476311#476311
    """
    vec = np.cross(v, u)
    if np.all(np.isclose(vec, np.zeros(3))):
        return npq.one
    ang_sine = np.linalg.norm(vec)
    ang_cosine = np.dot(v, u)
    ang = np.arctan2(ang_sine, ang_cosine)
    return npq.from_rotation_vector(vec / ang_sine * ang)
