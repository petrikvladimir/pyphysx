#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/31/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# Parse URDF file into the tree robot.
#

from typing import List, Tuple
from pathlib import Path
from xml.etree.ElementTree import ElementTree, parse

import trimesh

from pyphysx_utils.tree_robot import *
from pyphysx_utils.transformations import quat_between_two_vectors


class URDFRobot(TreeRobot):

    def __init__(self, urdf_path, mesh_path=None, use_random_collision_colors=False, **kwargs) -> None:
        """
        Create a robot from urdf_path or urdf_text.
        :param urdf_path: path to the urdf file
        :param mesh_path: path to the directory with meshes that are referred from urdf
        """
        super().__init__(**kwargs)
        self.urdf_path = Path(urdf_path)
        self.mesh_path = Path(mesh_path) if mesh_path is not None else self.urdf_path.parent
        urdf = parse(self.urdf_path)
        self.materials = self._parse_materials(urdf)
        self.parse_links_from_urdf_etree(urdf, self.mesh_path, use_random_collision_colors=use_random_collision_colors)
        self.parse_joints_from_urdf_etree(urdf)

    def parse_links_from_urdf_etree(self, urdf: ElementTree, mesh_root_folder: Path, use_random_collision_colors=False):
        for link_element in urdf.iterfind('link'):
            link = Link(link_element.get('name'), RigidDynamic())

            collision_shapes = []
            for collision_element in link_element.iterfind('collision'):
                collision_shapes += self._parse_shapes(collision_element, mesh_root_folder=mesh_root_folder)

            visual_not_simulated = sum(1 for _ in link_element.iterfind('visual')) != 0 and len(collision_shapes) != 0

            visual_shapes = []
            for visual_element in link_element.iterfind('visual'):
                visual_shapes += self._parse_shapes(visual_element, mesh_root_folder=mesh_root_folder,
                                                    global_materials=self.materials, set_visual_mesh_userdata=True,
                                                    sphere_instead_of_mesh=visual_not_simulated)

            """ Color collision shapes based on the colors used in visual shapes. """
            if not use_random_collision_colors:
                visual_colors = [s.get_user_data()['color'] for s in visual_shapes if
                                 s.get_user_data() is not None and 'color' in s.get_user_data()]
                if len(visual_colors) > 0:
                    mean_visual_color = np.mean(visual_colors, axis=0)
                    for s in collision_shapes:
                        URDFRobot.shape_update_user_data(s, 'color', mean_visual_color)

            """ Ignore simulation of visual shape and rendering of collision if both are specified. """
            if visual_not_simulated:
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
            jtype = joint_element.get('type', 'fixed')
            if jtype == 'continuous':
                jtype = 'revolute'
            joint = Joint(joint_element.get('name'), jtype)
            origin = self._get_origin_from_urdf_element(joint_element)
            limit_element = joint_element.find('limit')

            axis = [1., 0., 0.] if joint_element.find('axis') == None \
                else np.array([float(f) for f in joint_element.find('axis').get('xyz', '1 0 0').split()])
            alignment_transform = (np.zeros(3), quat_between_two_vectors(np.array([1., 0., 0.]), axis))

            lower_limit_exists = limit_element is not None and 'lower' in limit_element.keys()
            upper_limit_exists = limit_element is not None and 'upper' in limit_element.keys()
            self.add_joint(
                joint_element.find('parent').get('link'),
                joint_element.find('child').get('link'),
                joint,
                local_pose0=multiply_transformations(origin, alignment_transform),
                local_pose1=alignment_transform,
                lower_limit=float(limit_element.get('lower')) if lower_limit_exists else None,
                upper_limit=float(limit_element.get('upper')) if upper_limit_exists else None,
            )

    @staticmethod
    def _parse_materials(element) -> Dict[str, List]:
        """ Parse materials into the dictionary. """
        materials = dict()

        """ First parse all materials in order to prevent bug where material hierarchy is not respected. """
        for material_element in element.findall('.//material'):
            name, color = URDFRobot._parse_material(material_element)
            if color is not None:
                materials[name] = color

        """ Replace previously obtained materials by top level materials """
        for material_element in element.iterfind('material'):
            name, color = URDFRobot._parse_material(material_element)
            materials[name] = color
        return materials

    @staticmethod
    def _parse_material(material_element) -> Tuple[str, List]:
        """ Given material element return tuple of name and color """
        name = str(material_element.get('name'))
        color_str = material_element.find('color')
        color = None if color_str is None else [float(v) for v in color_str.get('rgba', '1 1 1 1').split()]
        return name, color

    @staticmethod
    def shape_update_user_data(shape, name, value):
        if shape.get_user_data() is not None:
            shape.get_user_data().update({name: value})
        else:
            shape.set_user_data({name: value})

    @staticmethod
    def load_mesh_shapes(mesh_path, material, scale, set_visual_mesh_userdata=False, sphere_shape=False) -> List[Shape]:
        """ Load mesh obj file and return all shapes in an array. """
        mesh_resolver = trimesh.resolvers.FilePathResolver(mesh_path)
        obj = trimesh.load(mesh_path, split_object=True, group_material=False, resolver=mesh_resolver)
        transform = np.diag([*scale, 1])
        obj.apply_transform(transform)

        if isinstance(obj, trimesh.scene.scene.Scene):
            if sphere_shape:
                shapes = [Shape.create_sphere(radius=1., material=material) for _ in obj.geometry.values()]
            else:
                shapes = [Shape.create_convex_mesh_from_points(g.vertices, material) for g in obj.geometry.values()]
            if set_visual_mesh_userdata:
                for s, g in zip(shapes, obj.geometry.values()):
                    URDFRobot.shape_update_user_data(s, 'visual_mesh', g)
        else:
            if sphere_shape:
                shapes = [Shape.create_sphere(radius=1., material=material)]
            else:
                shapes = [Shape.create_convex_mesh_from_points(obj.vertices, material)]
            if set_visual_mesh_userdata:
                URDFRobot.shape_update_user_data(shapes[0], 'visual_mesh', obj)
        return shapes

    @staticmethod
    def _get_origin_from_urdf_element(element):
        element_origin = element.find('origin')
        if element_origin is None:
            return unit_pose()
        pos = [float(f) for f in element_origin.get('xyz', '0 0 0').split()]
        quat = quat_from_euler('xyz', [float(f) for f in element_origin.get('rpy', '0 0 0').split()])
        return pos, quat

    @staticmethod
    def _parse_shapes(element, mesh_root_folder: Path, global_materials=None, set_visual_mesh_userdata=False,
                      sphere_instead_of_mesh=False):
        """ Get list of shapes specified in a given element. E.g. if you provide collision element, it will give you
        all collision geometry elements. If global materials are specified, parse color as well. """
        geometry_element = element.find('geometry')
        if geometry_element is None:
            return []

        for geom in geometry_element:
            if geom.tag == 'mesh':
                mesh_path = mesh_root_folder.joinpath(geom.get('filename').replace('package://', ''))
                scale = [float(f) for f in geom.get('scale', '1 1 1').split()]
                shapes = URDFRobot.load_mesh_shapes(mesh_path, material=Material(), scale=scale,
                                                    set_visual_mesh_userdata=set_visual_mesh_userdata,
                                                    sphere_shape=sphere_instead_of_mesh)
            elif geom.tag == 'box':
                shapes = [Shape.create_box(size=geom.get('size', '1 1 1').split(), material=Material())]
            elif geom.tag == 'sphere':
                shapes = [Shape.create_sphere(radius=float(geom.get('radius', '1')), material=Material())]
            elif geom.tag == 'cylinder':
                mesh = trimesh.creation.cylinder(radius=float(geom.get('radius', '1')),
                                                 height=float(geom.get('length', '1')))
                scale = float(geom.get('scale', '1').split()[0])
                shapes = [Shape.create_convex_mesh_from_points(mesh.vertices, Material(), scale)]
            else:
                raise NotImplementedError(f'Only sphere/box/cylinder/mesh geometries are supported. Got {geom.tag}')
        local_pose = URDFRobot._get_origin_from_urdf_element(element)
        for s in shapes:
            s.set_local_pose(local_pose)

        if global_materials is not None:
            materials = URDFRobot._parse_materials(element)
            if len(materials) == 1:
                clr = list(materials.values())[0]
                if clr is None:
                    clr = global_materials.get(list(materials.keys())[0])
                assert clr is not None, f'Color value must be specified either inside robot or in material. Not found for material {materials}'
                for s in shapes:
                    URDFRobot.shape_update_user_data(s, 'color', clr)
        return shapes

    @staticmethod
    def _parse_mass(link_element, min_mass=0.1, verbose=True) -> float:
        mass_element = link_element.find('inertial/mass')
        mass = float(mass_element.get('value')) if mass_element is not None else None
        if mass is None or mass < min_mass:
            if verbose:
                print(f'Mass of a link is small - simulation unstable. Using {min_mass} kg instead of {mass} kg.')
            mass = min_mass
        return mass
