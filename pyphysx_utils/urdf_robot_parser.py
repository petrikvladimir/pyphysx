#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/31/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# Parse URDF file into the tree robot.
#

from pathlib import Path

from pyphysx_utils.tree_robot import TreeRobot
from typing import Dict, Optional, List
import anytree
from xml.etree.ElementTree import ElementTree, parse, Element
import numpy as np
import trimesh
from pyphysx_utils.transformations import quat_between_two_vectors
from pyphysx import *
from pyphysx_utils.tree_robot import *
import quaternion as npq


class URDFRobot(TreeRobot):

    def __init__(self, urdf_path, mesh_path=None) -> None:
        super().__init__()
        self.urdf_path = Path(urdf_path)
        self.mesh_path = Path(mesh_path) if mesh_path is not None else self.urdf_path.parent
        urdf = parse(self.urdf_path)
        self.parse_links_from_urdf_etree(urdf, self.mesh_path)
        self.parse_joints_from_urdf_etree(urdf)

    def parse_links_from_urdf_etree(self, urdf: ElementTree, mesh_root_folder: Path):
        for link_element in urdf.iterfind('link'):
            link = Link(link_element.get('name'), RigidDynamic())

            visual_shapes = []
            for visual_element in link_element.iterfind('visual'):
                visual_shapes += self._parse_shapes(visual_element, mesh_root_folder=mesh_root_folder)
            collision_shapes = []
            for collision_element in link_element.iterfind('collision'):
                collision_shapes += self._parse_shapes(collision_element, mesh_root_folder=mesh_root_folder)

            # Ignore simulation of visual shape and rendering of collision if both are specified.
            if len(collision_shapes) != 0 and len(visual_shapes) != 0:
                for s in visual_shapes:
                    s.set_flag(ShapeFlag.SIMULATION_SHAPE, False)
                for s in collision_shapes:
                    s.set_flag(ShapeFlag.VISUALIZATION, False)

            for s in visual_shapes + collision_shapes:
                link.actor.attach_shape(s)

            link.actor.set_mass(self._parse_mass(link_element))
            self.add_link(link)

    def parse_joints_from_urdf_etree(self, urdf: ElementTree):
        for joint_element in urdf.iterfind('joint'):
            joint = Joint(joint_element.get('name'), joint_element.get('type', 'fixed'))
            origin = self._get_origin_from_urdf_element(joint_element)
            limit_element = joint_element.find('limit')

            axis = np.array([float(f) for f in joint_element.find('axis').get('xyz', '1 0 0').split()])
            alignment_transform = (np.zeros(3), quat_between_two_vectors(np.array([1., 0., 0.]), axis))

            self.add_joint(
                joint_element.find('parent').get('link'),
                joint_element.find('child').get('link'),
                joint,
                local_pose0=multiply_transformations(origin, alignment_transform),
                local_pose1=alignment_transform,
                lower_limit=float(limit_element.get('lower')) if limit_element is not None else None,
                upper_limit=float(limit_element.get('upper')) if limit_element is not None else None,
            )

    @staticmethod
    def load_mesh_shapes(mesh_path, material, scale: float) -> List[Shape]:
        """ Load mesh obj file and return all shapes in an array. """
        obj = trimesh.load(mesh_path, split_object=True, group_material=False)
        if isinstance(obj, trimesh.scene.scene.Scene):
            return [Shape.create_convex_mesh_from_points(g.vertices, material, scale) for g in obj.geometry.values()]
        else:
            return [Shape.create_convex_mesh_from_points(obj.vertices, material, scale)]

    @staticmethod
    def _get_origin_from_urdf_element(element):
        element_origin = element.find('origin')
        if element_origin is None:
            return unit_pose()
        pos = [float(f) for f in element_origin.get('xyz', '0 0 0').split()]
        quat = quat_from_euler('xyz', [float(f) for f in element_origin.get('rpy', '0 0 0').split()])
        return pos, quat

    @staticmethod
    def _parse_shapes(element, mesh_root_folder: Path):
        """ Get list of shapes specified in a given element. E.g. if you provide collision element, it will give you
        all collision geometry elements. """
        geometry_element = element.find('geometry')
        if geometry_element is None:
            return []

        for geom in geometry_element:
            if geom.tag == 'mesh':
                mesh_path = mesh_root_folder.joinpath(geom.get('filename').replace('package://', ''))
                scale = float(geom.get('scale', '1').split()[0])
                shapes = URDFRobot.load_mesh_shapes(mesh_path, material=Material(), scale=scale)
            elif geom.tag == 'box':
                shapes = [Shape.create_box(size=geom.get('size', '1 1 1').split(), material=Material())]
            elif geom.tag == 'sphere':
                shapes = [Shape.create_sphere(radius=float(geom.get('radius', '1')), material=Material())]
            else:
                raise NotImplementedError('Only sphere/box/mesh geometries are supported')
        local_pose = URDFRobot._get_origin_from_urdf_element(element)
        for s in shapes:
            s.set_local_pose(local_pose)
        return shapes

    @staticmethod
    def _parse_mass(link_element, min_mass=0.1, verbose=True) -> float:
        mass_element = link_element.find('inertial/mass')
        mass = float(mass_element.get('value')) if mass_element is not None else None
        if mass is None or mass < min_mass:
            if verbose:
                print('Mass of each link has to be set, otherwise unstable. Using mass {} kg instead of {} kg.'.format(
                    min_mass, mass))
            mass = min_mass
        return mass
