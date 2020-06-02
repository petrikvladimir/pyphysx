#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# Class used to represent robots (links connected by joints) in a tree like structure.
# Parallel mechanism are therefore not supported.
#
# Each robot has a unique root node (link). Other links are connected by a joint (fixed, revolute, or prismatic) to a
# parent. For serial manipulator:
#
# (root) -> (link_1) -> (link_2) ... -> (link_n)
#
# Root link can be optionally attached to an existing actor or to a static world pose.
#

from typing import Dict, Optional

import numpy as np
import quaternion as npq
import anytree

from pyphysx_utils.transformations import multiply_transformations, inverse_transform, unit_pose, quat_from_euler
from pyphysx import *


class KinematicPhysXJoint:
    """ Class that provides same functions as D6Joint but for kinematic robot. """

    def __init__(self, actor0, actor1, local_pose0=None, local_pose1=None) -> None:
        super().__init__()
        self.lower = -np.inf
        self.upper = np.inf
        self.local_pose0 = cast_transformation(local_pose0) if local_pose0 is not None else unit_pose()
        self.local_pose1 = cast_transformation(local_pose1) if local_pose1 is not None else unit_pose()
        self.motion = D6Motion.LOCKED

    def get_local_pose(self, actor_id):
        return self.local_pose0 if actor_id == 0 else self.local_pose1

    def set_motion(self, axis, motion):
        self.motion = motion

    def get_motion(self, axis):
        return self.motion

    def set_linear_limit(self, axis, lower_limit, upper_limit):
        self.lower = lower_limit
        self.upper = upper_limit

    def get_linear_limit(self, axis):
        return self.lower, self.upper

    def set_twist_limit(self, lower_limit, upper_limit):
        self.lower = lower_limit
        self.upper = upper_limit

    def get_twist_limit(self):
        return self.lower, self.upper

    def set_drive_position(self, pose):
        pass

    def set_drive_velocity(self, linear=None, angular=None):
        pass

    def set_drive(self, axis, stiffness=0, damping=0, force_limit=0, is_acceleration=False):
        pass


class Joint:
    """
    Defines a joint that connects two link together.
    Can be one of the types: (i) fixed, (ii) prismatic, or (iii) revolute.
    """

    def __init__(self, name, joint_type='fixed', null_value=0.) -> None:
        super().__init__()
        self.name = name
        self.joint_type: str = joint_type
        self.null_value: float = null_value
        self.commanded_joint_position = null_value
        self.commanded_joint_velocity = 0.
        self.physx_joint: Optional[D6Joint] = None

    def joint_transformation(self, joint_position=None):
        """ Get transformation of joint. For prismatic, this is defined as translation in x-axis.
            For revolute it is rotation about x-axis. """
        if self.joint_type == 'fixed':
            return unit_pose()
        if joint_position is None:
            joint_position = self.null_value
        if self.joint_type == 'prismatic':
            return np.array([joint_position, 0, 0]), npq.one
        elif self.joint_type == 'revolute':
            return np.zeros(3), quat_from_euler('x', [joint_position])
        else:
            raise NotImplementedError('Only fixed, prismatic and revolute joints are supported.')

    def transformation_from_parent_to_child_link(self, joint_position=None):
        """ Return transformation from parent to the child either for the null position or for the specified
            position of the joint. """
        t0 = self.physx_joint.get_local_pose(0)
        tj = self.joint_transformation(joint_position)
        t1 = inverse_transform(self.physx_joint.get_local_pose(1))
        return multiply_transformations(multiply_transformations(t0, tj), t1)

    @property
    def is_revolute(self):
        return self.joint_type == 'revolute'

    @property
    def is_fixed(self):
        return self.joint_type == 'fixed'

    @property
    def is_prismatic(self):
        return self.joint_type == 'prismatic'

    def create_physx_joint(self, actor0, actor1, local_pose0, local_pose1, lower_limit=None, upper_limit=None,
                           kinematic=False):
        """ Create physx joint that connects given actors at given poses. Free joint motion is used if limits are not
        specified and limited is used otherwise for prismatic and revolute joint. """
        if local_pose0 is None:
            local_pose0 = unit_pose()
        if local_pose1 is None:
            local_pose1 = unit_pose()
        jcls = KinematicPhysXJoint if kinematic else D6Joint
        self.physx_joint = jcls(actor0, actor1, local_pose0, local_pose1)
        is_limited = lower_limit is not None and upper_limit is not None
        if self.is_revolute:
            self.physx_joint.set_motion(D6Axis.TWIST, D6Motion.LIMITED if is_limited else D6Motion.FREE)
            if is_limited:
                self.physx_joint.set_twist_limit(lower_limit=lower_limit, upper_limit=upper_limit)
        elif self.is_prismatic:
            self.physx_joint.set_motion(D6Axis.X, D6Motion.LIMITED if is_limited else D6Motion.FREE)
            if is_limited:
                self.physx_joint.set_linear_limit(D6Axis.X, lower_limit=lower_limit, upper_limit=upper_limit)

    def set_joint_position(self, value):
        """ Set desired position of the joint. """
        value = value if value is not None else self.null_value
        if self.is_prismatic:
            self.physx_joint.set_drive_position((value, 0, 0))
        elif self.is_revolute:
            self.physx_joint.set_drive_position((np.zeros(3), quat_from_euler('x', value)))
        self.commanded_joint_position = value

    def set_joint_velocity(self, value):
        """ Set desired velocity of the joint. """
        if self.is_prismatic:
            self.physx_joint.set_drive_velocity(linear=(value, 0, 0))
        elif self.is_revolute:
            self.physx_joint.set_drive_velocity(angular=(value, 0, 0))
        self.commanded_joint_velocity = value

    def configure_drive(self, stiffness=1e7, damping=1e5, force_limit=1e5, is_acceleration=False):
        """ Configure drive for the joint. Drive is used to control joint position and velocity as PD controller. """
        if self.is_revolute:
            self.physx_joint.set_drive(D6Drive.TWIST, stiffness=stiffness, damping=damping, force_limit=force_limit,
                                       is_acceleration=is_acceleration)
        elif self.is_prismatic:
            self.physx_joint.set_drive(D6Drive.X, stiffness=stiffness, damping=damping, force_limit=force_limit,
                                       is_acceleration=is_acceleration)

    def get_limits(self):
        """ Get limits of the joint, return tuple consisting of lower and upper limit.
            If joint is not not limited returns -inf, inf. """
        if self.is_prismatic:
            if self.physx_joint.get_motion(D6Axis.X) == D6Motion.FREE:
                return -np.inf, np.inf
            return self.physx_joint.get_linear_limit(D6Axis.X)
        elif self.is_revolute:
            if self.physx_joint.get_motion(D6Axis.TWIST) == D6Motion.FREE:
                return -np.inf, np.inf
            return self.physx_joint.get_twist_limit()
        else:
            return 0, 0


class Link(anytree.Node):
    """
    Link is named and defines: (i) an actor that specifies dynamic and geometric properties and (ii) joint that connects
    this link to the parent link.
    """

    def __init__(self, name, actor=None):
        super().__init__(name)
        self.actor = actor
        self.joint_from_parent: Optional[Joint] = None


class TreeRobot:

    def __init__(self, kinematic=False) -> None:
        """ If robot is kinematic, then all actors are set to be kinematic. Actors poses are set from forward kinematic
            automatically. """
        super().__init__()
        self.kinematic = kinematic
        self.links = {}  # type: Dict[str, Link]
        self.movable_joints = {}  # type: Dict[str, Joint]
        self._root_node = None  # type: Optional[Link]
        self.world_attachment_actor = None

    @property
    def root_node(self) -> Link:
        """ Get the root node of the kinematic tree. Root node value is cached. """
        if self._root_node is None:
            roots = [node for node in self.links.values() if node.is_root]
            assert len(roots) == 1
            self._root_node = roots[0]
        return self._root_node

    def add_link(self, link: Link):
        """ Add new link to the structure. This invalidates previously computed root node. """
        self.links[link.name] = link
        if self.kinematic:
            link.actor.set_rigid_body_flag(RigidBodyFlag.KINEMATIC, True)
        self._root_node = None

    def add_joint(self, parent_name: str, child_name: str, joint: Joint = None, local_pose0=None, local_pose1=None,
                  lower_limit=None, upper_limit=None):
        """ Add new joint to the structure. This invalidates previously computed root node. """
        self.links[child_name].parent = self.links[parent_name]
        self.links[child_name].joint_from_parent = joint
        if joint is not None:
            self.links[child_name].joint_from_parent.create_physx_joint(
                self.links[parent_name].actor, self.links[child_name].actor,
                local_pose0, local_pose1, lower_limit, upper_limit, self.kinematic
            )
            if not joint.is_fixed:
                self.movable_joints[joint.name] = joint
        self._root_node = None

    def print_structure(self, from_link: Link = None):
        """ Print structure of the robot into the terminal. Useful only for debugging. """
        for pre, fill, node in anytree.RenderTree(self.root_node if from_link is None else from_link):
            print("%s%s" % (pre, node.name))

    @property
    def root_pose(self):
        """ Get the pose of the root link. If attached return parent pose otherwise unit pose. """
        return unit_pose() if self.world_attachment_actor is None else self.world_attachment_actor.get_global_pose()

    def compute_link_transformations(self, joint_values: Optional[Dict[str, float]] = None) -> Dict[str, tuple]:
        """ Compute transformations of all links for given joint values and return them in a dictionary in which link
        name serves as a key and link pose is a value. """
        if joint_values is None:
            joint_values = {}
        link_transforms = {}
        for link in anytree.LevelOrderIter(self.root_node):  # type: Link
            if link.is_root:
                link_transforms[link.name] = self.root_pose
                continue
            parent_transform = link_transforms[link.parent.name]
            joint_value = joint_values.get(link.joint_from_parent.name, None)
            relative_pose = link.joint_from_parent.transformation_from_parent_to_child_link(joint_value)
            link_transforms[link.name] = multiply_transformations(parent_transform, relative_pose)
        return link_transforms

    def get_joint_names(self):
        """ Get joint names for all movable joints, i.e. for prismatic and revolute joints. """
        return list(self.movable_joints.keys())

    def attach_root_node_to_pose(self, pose):
        """ Create attachment joint that connects root link to given world coordinates. """
        self.world_attachment_actor = RigidStatic()
        self.world_attachment_actor.set_global_pose(pose)
        return D6Joint(self.world_attachment_actor, self.root_node.actor)

    def attach_root_node_to_actor(self, actor, **kwargs):
        """ Create attachment joint that connects root link to the given actor. """
        self.world_attachment_actor = actor
        return D6Joint(self.world_attachment_actor, self.root_node.actor, **kwargs)

    def reset_pose(self, joint_values: Optional[Dict[str, float]] = None):
        """ Reset pose of every actor in the tree. The poses are computed for a given joint values. The commanded joint
            values are set to given values too."""
        if joint_values is None:
            joint_values = {}
        transformations = self.compute_link_transformations(joint_values)
        for link in self.links.values():
            link.actor.set_global_pose(transformations[link.name])
        for joint in self.movable_joints.values():
            joint.set_joint_position(joint_values.get(joint.name, None))

    def set_joints_position(self, joint_values: Dict[str, float]):
        """ Set desired position of every joint in a tree. """
        for joint in self.movable_joints.values():
            joint.set_joint_position(joint_values[joint.name])

    def set_joints_velocities(self, joint_values: Dict[str, float]):
        """ Set desired velocity of every joint in a tree. """
        for joint in self.movable_joints.values():
            joint.set_joint_velocity(joint_values[joint.name])

    def get_aggregate(self, enable_self_collision=False):
        """ Get aggregate of actors that can be included into the scene. """
        agg = Aggregate(enable_self_collision=enable_self_collision)
        if self.world_attachment_actor is not None:
            agg.add_actor(self.world_attachment_actor)
        for link in self.links.values():
            agg.add_actor(link.actor)
        return agg

    def disable_gravity(self):
        """ Disable gravity for all links. """
        for link in self.links.values():
            link.actor.disable_gravity()

    def update(self, dt):
        """
            Method should be call before each simulate command.
            It updates the commanded joint position based on the current commanded position and velocity.
            For kinematic robots, it computes and set kinematic target for each link.
        """
        for joint_name, joint in self.movable_joints.items():
            joint.set_joint_position(
                np.clip(joint.commanded_joint_position + joint.commanded_joint_velocity * dt, *joint.get_limits())
            )

        if self.kinematic:
            joint_values = dict()
            for joint_name, joint in self.movable_joints.items():
                joint_values[joint_name] = joint.commanded_joint_position
            link_poses = self.compute_link_transformations(joint_values)
            for link_name, link in self.links.items():
                link.actor.set_kinematic_target(link_poses[link_name])
