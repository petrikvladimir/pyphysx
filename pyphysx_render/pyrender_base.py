#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 1/28/21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from typing import List

import numpy as np
from trimesh import Trimesh
from trimesh.creation import uv_sphere, box

from pyrender.scene import Scene as PyRenderScene
from pyrender import PerspectiveCamera, Mesh, Primitive, GLTF
from pyrender import Node

from pyphysx_render.pyrender_trackball import RoboticTrackball
from pyphysx_render.render_base import ViewerBase
from pyphysx import ShapeFlag, Shape, RigidActor, GeometryType

from pyphysx_render.utils import gl_color_from_matplotlib
from pyphysx_utils.transformations import multiply_transformations, pose_to_transformation_matrix


class PyRenderBase(ViewerBase):

    def __init__(self, render_scene=None, plane_grid_spacing=1., plane_grid_num_of_lines=10,
                 spheres_count=(8, 8)) -> None:
        """ Base class for PyRender viewer and offscreen renderer. """
        ViewerBase.__init__(self)
        self.spheres_count = spheres_count
        self.plane_grid_num_of_lines = plane_grid_num_of_lines
        self.plane_grid_spacing = plane_grid_spacing
        if render_scene is None:
            render_scene = PyRenderScene()
            render_scene.bg_color = np.array([0.75] * 3)
        if render_scene.main_camera_node is None:
            cam = PerspectiveCamera(yfov=np.deg2rad(60), aspectRatio=1.414, znear=0.005)
            cam_pose = np.eye(4)
            cam_pose[:3, 3] = RoboticTrackball.spherical_to_cartesian(3., 0., np.deg2rad(45.), target=np.zeros(3))
            cam_pose[:3, :3] = RoboticTrackball.look_at_rotation(eye=cam_pose[:3, 3], target=np.zeros(3), up=[0, 0, 1])
            nc = Node(camera=cam, matrix=cam_pose)
            render_scene.add_node(nc)
            render_scene.main_camera_node = nc
        self.render_scene = render_scene
        self.nodes_and_actors = []

    def _acquire_lock(self, blocking=True):
        """ Used for underlying viewer to acquire lock if required. """
        return True

    def _release_lock(self):
        """ Used for underlying viewer to acquire lock if required. """
        pass

    @staticmethod
    def _get_actor_pose_matrix(actor, offset):
        """ Get actor transformation matrix with applied offset if not none. """
        pose = actor.get_global_pose()
        if offset is not None:
            pose = multiply_transformations(offset, pose)
        return pose_to_transformation_matrix(pose)

    def add_physx_scene(self, scene, render_shapes_with_one_of_flags=(ShapeFlag.VISUALIZATION,), offset=None):
        """ Call this function to create a renderer scene from physx scene. """
        actors = scene.get_dynamic_rigid_actors() + scene.get_static_rigid_actors()
        for i, actor in enumerate(actors):
            n = self.actor_to_node(actor, render_shapes_with_one_of_flags)
            if n is not None:
                self.nodes_and_actors.append((n, actor, offset))
                self._acquire_lock()
                self.render_scene.add_node(n)
                self.render_scene.set_pose(n, self._get_actor_pose_matrix(actor, offset))
                self._release_lock()

    def clear_physx_scenes(self):
        """ Remove all tracked actors and the corresponding nodes. """
        self._acquire_lock()
        for node, actor, offset in self.nodes_and_actors:
            self.render_scene.remove_node(node)
        self.nodes_and_actors.clear()
        self._release_lock()

    def update(self, blocking=False):
        """ Update scene if lock can be acquired. Otherwise do nothing. Set blocking to True in order to force it. """
        if self._acquire_lock(blocking=blocking):
            for node, actor, offset in self.nodes_and_actors:
                self.render_scene.set_pose(node, self._get_actor_pose_matrix(actor, offset))
            self._release_lock()

    def _trimesh_from_basic_shape(self, shape: Shape, vertex_colors=None):
        """ Get trimesh from shape for visual meshes, spheres, or boxes. Return None if it is not a basic shape. """
        if shape.get_user_data() is not None:
            visual_mesh = shape.get_user_data().get('visual_mesh', None)
            if visual_mesh is not None:
                if not isinstance(visual_mesh, Trimesh):
                    raise NotImplementedError('Only trimesh scene or trimesh instances are supported.')
                if hasattr(visual_mesh.visual, 'material'):
                    visual_mesh.visual.material.kwargs['Ns'] = np.abs(visual_mesh.visual.material.kwargs['Ns'])
                return visual_mesh
        gtype = shape.get_geometry_type()
        if not gtype in [GeometryType.SPHERE, GeometryType.BOX]:
            return None
        geom = uv_sphere(shape.get_sphere_radius(), self.spheres_count) if gtype == GeometryType.SPHERE else box(
            2 * shape.get_box_half_extents())
        if vertex_colors is not None:
            geom.visual.vertex_colors = vertex_colors
        return geom

    def _grid_lines(self):
        """ Get points for n lines [Nx3] representing grid structure. Grid is in y-z plane.
         However, PyRender renders it on xy plane. """
        d = self.plane_grid_spacing
        n = self.plane_grid_num_of_lines
        num_horizontal_lines = 2 * n + 1
        points = np.zeros((2 * 2 * num_horizontal_lines, 3))
        g = np.linspace(-n * d, n * d, num_horizontal_lines)
        points[:2 * num_horizontal_lines, 2] = g.repeat(2)
        points[:2 * num_horizontal_lines, 1] = np.tile([-n * d, n * d], num_horizontal_lines)
        points[2 * num_horizontal_lines:, 2] = np.tile([-n * d, n * d], num_horizontal_lines)
        points[2 * num_horizontal_lines:, 1] = g.repeat(2)
        return points

    def shape_to_mesh(self, shape: Shape):
        """ Convert pyphysx shape into the pyrender Mesh """
        clr_string = shape.get_user_data().get('color', None) if shape.get_user_data() is not None else None
        clr = gl_color_from_matplotlib(color=clr_string, return_rgba=True)
        basic_trimesh = self._trimesh_from_basic_shape(shape, clr)
        if basic_trimesh is not None:
            return Mesh.from_trimesh(basic_trimesh)
        elif shape.get_geometry_type() == GeometryType.CONVEXMESH:
            data = shape.get_shape_data()  # N x 9 - i.e. 3 triangles
            primitive = Primitive(data.reshape(-1, 3), normals=None, color_0=clr, mode=GLTF.TRIANGLES, poses=None)
            return Mesh(primitives=[primitive])
        elif shape.get_geometry_type() == GeometryType.PLANE:
            clr = [0.8] * 3 + [0.1] if clr_string is None else gl_color_from_matplotlib(clr_string, return_rgba=True)
            primitive = Primitive(positions=self._grid_lines(), normals=None, color_0=clr, mode=GLTF.LINES, poses=None)
            return Mesh(primitives=[primitive])
        else:
            raise NotImplementedError('Not supported type of the geometry.')

    def shape_to_node(self, shape):
        pose = pose_to_transformation_matrix(shape.get_local_pose())
        return Node(mesh=self.shape_to_mesh(shape=shape), matrix=pose)

    def actor_to_node(self, actor, flags):
        shapes = [s for s in actor.get_atached_shapes() if PyRenderBase.has_shape_any_of_flags(s, flags)]
        if len(shapes) == 0:
            return None
        return Node(children=[self.shape_to_node(s) for s in shapes])
