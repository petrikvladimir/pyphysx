#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# Class used to represent robots (links connected by joints) in a tree like structure.
# Parallel mechanism are therefore not supported.
# todo: this file needs significant refactor...
#

from pathlib import Path
import anytree
from xml.etree.ElementTree import ElementTree, parse
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from pyphysx_utils.transformations import multiply_transformations, inverse_transform
from pyphysx import *


class Link(anytree.Node):

    def __init__(self, name, actor=None, material=None):
        super().__init__(name)
        self.actor = actor
        self.material = material
        self.joint = None
        self.joint_name = None
        self.motion_axis = None

    def get_transform_from_parent(self, joint_value):
        t0 = self.joint.get_local_pose(0)
        jtran = self.get_transform_for_motion_axis(joint_value)
        t1 = self.joint.get_local_pose(1)
        return multiply_transformations(*multiply_transformations(*t0, *jtran), *t1)

    def get_transform_for_motion_axis(self, v):
        axis = self.motion_axis
        pos, quat = np.zeros(3), np.array([0, 0, 0, 1.])
        if axis == D6Axis.X:
            pos[0] = v
        elif axis == D6Axis.Y:
            pos[1] = v
        elif axis == D6Axis.Z:
            pos[2] = v
        elif axis == D6Axis.TWIST:
            quat = Rotation.from_euler('x', v).as_quat()
        elif axis == D6Axis.SWING1:
            quat = Rotation.from_euler('y', v).as_quat()
        elif axis == D6Axis.SWING2:
            quat = Rotation.from_euler('z', v).as_quat()
        return pos, quat


class TreeRobot:

    def __init__(self) -> None:
        super().__init__()
        self._root_node = None
        self.links = {}

    @property
    def root_node(self):
        if self._root_node is None:
            roots = [node for node in self.links.values() if node.is_root]
            assert len(roots) == 1
            self._root_node = roots[0]
        return self._root_node

    def add_link(self, link: Link):
        self.links[link.name] = link
        self._root_node = None

    def add_joint(self, parent_name: str, child_name: str, pos, quat, axis, init_value=0.):
        self.links[child_name].parent = self.links[parent_name]
        self.links[child_name].joint.pos = pos
        self.links[child_name].joint.quat = quat
        self.links[child_name].joint.axis = axis
        self.links[child_name].joint.val = init_value
        self._root_node = None

    def print_structure(self):
        for pre, fill, node in anytree.RenderTree(self.root_node):
            print("%s%s" % (pre, node.name))

    def compute_link_transformations(self, joint_values=None):
        """ Compute transformations of all links and return then in a dictionary. """
        if joint_values is None:
            joint_values = {}
        link_transforms = {}
        for link in anytree.LevelOrderIter(self.root_node):
            if link.is_root:
                link_transforms[link.name] = (np.zeros(3), np.array([0, 0, 0, 1]))
                continue
            parent_transform = link_transforms[link.parent.name]
            link_transforms[link.name] = multiply_transformations(
                *parent_transform, *link.get_transform_from_parent(joint_values.get(link.joint_name, 0.))
            )
        return link_transforms

    def get_joint_names(self):
        return [l.joint_name for l in self.links.values() if not l.is_root and l.motion_axis is not None]


class URDFRobot(TreeRobot):

    def __init__(self, urdf_path, mesh_path=None, attach_to_world_pos=None, attach_to_world_quat=(0, 0, 0, 1),
                 attach_to_actor=None, joints_drive_setup=None) -> None:
        super().__init__()
        self.urdf_path = Path(urdf_path)
        self.mesh_path = Path(mesh_path) if mesh_path is not None else self.urdf_path.parent

        if attach_to_world_pos is not None:
            self.world_attachment_actor = RigidStatic()
            self.world_attachment_actor.set_global_pose(attach_to_world_pos, attach_to_world_quat)
        elif attach_to_actor is not None:
            self.world_attachment_actor = attach_to_actor
        else:
            self.world_attachment_actor = None

        urdf = parse(self.urdf_path)
        self.load_links_from_urdf_etree(urdf, self.mesh_path)
        self.load_joint_from_urdf_etree(urdf)
        self.set_joints_drive(drives_setup=joints_drive_setup)
        if self.world_attachment_actor is not None:
            D6Joint(self.world_attachment_actor, self.root_node.actor)
        self.reset_actor_poses()

    @staticmethod
    def load_mesh_shapes(mesh_path, material):
        """ Load mesh obj file and return all shapes in an array. """
        obj = trimesh.load(mesh_path, split_object=True, group_material=False)
        if isinstance(obj, trimesh.scene.scene.Scene):
            return [Shape.create_convex_mesh_from_points(g.vertices, material) for g in obj.geometry.values()]
        else:
            return [Shape.create_convex_mesh_from_points(obj.vertices, material)]

    def _get_drive_axis_for_motion_axis(self, motion_axis: D6Axis):
        if motion_axis == D6Axis.X:
            return D6Drive.X
        elif motion_axis == D6Axis.Y:
            return D6Drive.Y
        elif motion_axis == D6Axis.Z:
            return D6Drive.Z
        elif motion_axis == D6Axis.TWIST:
            return D6Drive.TWIST
        else:
            return D6Drive.SWING

    @staticmethod
    def _get_physx_axis_from_urdf_prismatic_joint(element):
        axis = [float(f) for f in element.find('axis').get('xyz').split()]
        if np.isclose(axis, [1, 0, 0]).all():
            return D6Axis.X
        if np.isclose(axis, [0, 1, 0]).all():
            return D6Axis.Y
        if np.isclose(axis, [0, 0, 1]).all():
            return D6Axis.Z
        raise NotImplementedError("Only positive x,y, or z axis can be used for joint axis.")

    @staticmethod
    def _get_physx_axis_from_urdf_revolute_joint(element):
        axis = [float(f) for f in element.find('axis').get('xyz').split()]
        if np.isclose(axis, [1, 0, 0]).all():
            return D6Axis.TWIST
        if np.isclose(axis, [0, 1, 0]).all():
            return D6Axis.SWING1
        if np.isclose(axis, [0, 0, 1]).all():
            return D6Axis.SWING2
        raise NotImplementedError("Only positive x,y, or z axis can be used for joint axis.")

    @staticmethod
    def _get_origin_from_urdf_element(element):
        element_origin = element.find('origin')
        if element_origin is None:
            return [0, 0, 0], [0, 0, 0, 1]
        pos = [float(f) for f in element_origin.get('xyz', '0 0 0').split()]
        quat = Rotation.from_euler('xyz', [float(f) for f in element_origin.get('rpy', '0 0 0').split()]).as_quat()
        return pos, quat

    def load_links_from_urdf_etree(self, urdf: ElementTree, mesh_root_folder: Path):
        for ulink in urdf.iterfind('link'):
            link = Link(ulink.get('name'), RigidDynamic(), Material())

            geometry_element = ulink.find('collision/geometry')
            if geometry_element is not None:
                for geom in geometry_element:
                    if geom.tag == 'mesh':
                        mesh_path = mesh_root_folder.joinpath(geom.get('filename').replace('package://', ''))
                        [link.actor.attach_shape(s) for s in self.load_mesh_shapes(mesh_path, link.material)]
                    else:
                        raise NotImplementedError  # todo add other urdf geometries

            mass_element = ulink.find('inertial/mass')
            mass = float(mass_element.get('value')) if mass_element is not None else None
            if mass is None or mass < 0.1:
                mass = 0.1
                print('Mass of each link has to be set, otherwise unstable. Using {} kg mass.'.format(mass))
            link.actor.set_mass(mass)
            self.add_link(link)

    def load_joint_from_urdf_etree(self, urdf: ElementTree):
        for ujoint in urdf.iterfind('joint'):
            origin = self._get_origin_from_urdf_element(ujoint)

            jtype = ujoint.get('type', 'fixed')
            parent_link = self.links[ujoint.find('parent').get('link')]
            child_link = self.links[ujoint.find('child').get('link')]
            child_link.joint_name = ujoint.get('name')
            child_link.joint = D6Joint(parent_link.actor, child_link.actor, *origin)
            child_link.parent = parent_link
            if jtype == 'prismatic':
                axis = self._get_physx_axis_from_urdf_prismatic_joint(ujoint)
                limit_element = ujoint.find('limit')
                if limit_element is not None:
                    child_link.joint.set_motion(axis, D6Motion.LIMITED)
                    lower, upper = float(limit_element.get('lower')), float(limit_element.get('upper'))
                    child_link.joint.set_linear_limit(axis, lower, upper)
                else:
                    child_link.joint.set_motion(axis, D6Motion.FREE)
                child_link.motion_axis = axis
            elif jtype == 'revolute':
                axis = self._get_physx_axis_from_urdf_revolute_joint(ujoint)
                limit_element = ujoint.find('limit')
                if limit_element is not None:
                    child_link.joint.set_motion(axis, D6Motion.LIMITED)
                    lower, upper = float(limit_element.get('lower')), float(limit_element.get('upper'))
                    abs_max_limit = np.maximum(np.abs(lower), np.abs(upper))
                    if axis == D6Axis.TWIST:  # todo: the best approach would be to transfer all to x-axis joints
                        child_link.joint.set_twist_limit(axis, lower, upper)
                    elif axis == D6Axis.SWING1:
                        child_link.joint.set_swing_limit(abs_max_limit, 0.)
                    elif axis == D6Axis.SWING2:
                        child_link.joint.set_swing_limit(0., abs_max_limit)
                else:
                    child_link.joint.set_motion(axis, D6Motion.FREE)
                child_link.motion_axis = axis
            elif jtype == 'fixed':
                pass
            else:
                raise NotImplementedError("Only fixed, revolute, or prismatic joints are supported.")

    def set_joints_drive(self, drives_setup=None):
        drives_setup = drives_setup or {}
        for link in self.links.values():
            if link.motion_axis is not None:
                drive_setup = drives_setup.get(
                    link.joint_name,
                    dict(stiffness=1e7, damping=100., force_limit=1e5, is_acceleration=True)
                )
                link.joint.set_drive(self._get_drive_axis_for_motion_axis(link.motion_axis), **drive_setup)

    def get_aggregate(self):
        agg = Aggregate(enable_self_collision=False)
        if self.world_attachment_actor is not None:
            agg.add_actor(self.world_attachment_actor)
        for link in self.links.values():
            agg.add_actor(link.actor)
        return agg

    def set_joint_drive_values(self, values=None):
        if values is None:
            return
        for link in self.links.values():
            if link.motion_axis is not None:
                v = values.get(link.joint_name, 0.)
                link.joint.set_drive_position(*link.get_transform_for_motion_axis(v))

    def reset_actor_poses(self, base_pos=(0., 0., 0.), base_quat=(0., 0., 0., 1.), joint_values=None):
        if self.world_attachment_actor is not None:
            base_pos, base_quat = multiply_transformations(base_pos, base_quat,
                                                           *self.world_attachment_actor.get_global_pose())
        link_transforms = self.compute_link_transformations(joint_values)
        for link in self.links.values():
            link.actor.set_global_pose(*multiply_transformations(base_pos, base_quat, *link_transforms[link.name]))

        self.set_joint_drive_values(joint_values)
