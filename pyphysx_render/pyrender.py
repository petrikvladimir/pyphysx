#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 8/24/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#    Details: Render PhysX using PyRender package.

import os
import imageio
import pyglet
import pyrender
import trimesh
import numpy as np
from typing import List

from pyrender import Viewer, Primitive, GLTF, TextAlign

from pyphysx import ShapeFlag, Shape, RigidActor, GeometryType
from pyphysx_render.pyrender_trackball import RoboticTrackball
from pyphysx_render.utils import gl_color_from_matplotlib
from pyphysx_utils.transformations import pose_to_transformation_matrix, multiply_transformations


class PyPhysxViewer(Viewer):
    def __init__(self, render_scene=None, viewport_size=None, render_flags=None, viewer_flags=None,
                 registered_keys=None, run_in_thread=True, video_filename=None, **kwargs):
        """
            Use render_scene to specify camera or lighting or additional geometries if required.
            Additional viewer flags:
                - axes_scale - size of coordinate axes
        """

        if render_scene is None:
            render_scene = pyrender.Scene()
            render_scene.bg_color = np.array([0.75] * 3)
        if render_scene.main_camera_node is None:
            cam = pyrender.PerspectiveCamera(yfov=np.deg2rad(60), aspectRatio=1.414, znear=0.005)
            cam_pose = np.eye(4)
            cam_pose[:3, 3] = RoboticTrackball.spherical_to_cartesian(3., 0., np.deg2rad(45.), target=np.zeros(3))
            cam_pose[:3, :3] = RoboticTrackball.look_at_rotation(eye=cam_pose[:3, 3], target=np.zeros(3), up=[0, 0, 1])
            nc = pyrender.Node(camera=cam, matrix=cam_pose)
            render_scene.add_node(nc)
            render_scene.main_camera_node = nc

        _viewer_flags = {
            'view_center': np.zeros(3),
            'window_title': 'PyPhysX Scene Viewer',
            'show_world_axis': True,
            'show_mesh_axes': False,
            'axes_scale': 0.5,
            'use_raymond_lighting': True,
            'plane_grid_spacing': 1.,
            'plane_grid_num_of_lines': 10,
            'spheres_count': [8, 8],
        }
        if viewer_flags is not None:
            _viewer_flags.update(viewer_flags)

        super().__init__(render_scene, viewport_size, render_flags, _viewer_flags, registered_keys, run_in_thread,
                         **kwargs)
        self.nodes_and_actors = []
        fps = self.viewer_flags['refresh_rate']
        self.video_writer = imageio.get_writer(video_filename, fps=fps) if video_filename is not None else None

    def _reset_view(self):
        super()._reset_view()
        self._trackball = RoboticTrackball(self._default_camera_pose, self.viewport_size, self._trackball._scale)

    @staticmethod
    def has_shape_any_of_flags(shape: Shape, flags: List[ShapeFlag]):
        """ Return true if shape contains any of the flag from the list. """
        for show_flag in flags:
            if shape.get_flag_value(show_flag):
                return True
        return False

    def shape_to_meshes(self, shape: Shape):
        clr_string = shape.get_user_data().get('color', None) if shape.get_user_data() is not None else None
        clr = gl_color_from_matplotlib(color=clr_string, return_rgba=True)
        visual_mesh = shape.get_user_data().get('visual_mesh', None) if shape.get_user_data() is not None else None

        meshes = []
        if visual_mesh is not None:
            if isinstance(visual_mesh, trimesh.Trimesh):
                if hasattr(visual_mesh.visual, 'material'):
                    visual_mesh.visual.material.kwargs['Ns'] = np.abs(visual_mesh.visual.material.kwargs['Ns'])
                meshes.append(pyrender.Mesh.from_trimesh(visual_mesh))
            else:
                raise NotImplementedError('Only trimesh scene or trimesh instances are supported.')
        elif shape.get_geometry_type() == GeometryType.SPHERE:
            geom = trimesh.creation.uv_sphere(radius=shape.get_sphere_radius(),
                                              count=self.viewer_flags['spheres_count'])
            geom.visual.vertex_colors = clr
            meshes.append(pyrender.Mesh.from_trimesh(geom))
        elif shape.get_geometry_type() == GeometryType.BOX:
            geom = trimesh.creation.box(extents=2 * shape.get_box_half_extents())
            geom.visual.vertex_colors = clr
            meshes.append(pyrender.Mesh.from_trimesh(geom))
        elif shape.get_geometry_type() == GeometryType.PLANE:
            d = self.viewer_flags['plane_grid_spacing']
            n = self.viewer_flags['plane_grid_num_of_lines']
            num_horizontal_lines = 2 * n + 1
            points = np.zeros((2 * 2 * num_horizontal_lines, 3))
            g = np.linspace(-n * d, n * d, num_horizontal_lines)
            points[:2 * num_horizontal_lines, 2] = g.repeat(2)
            points[:2 * num_horizontal_lines, 1] = np.tile([-n * d, n * d], num_horizontal_lines)
            points[2 * num_horizontal_lines:, 2] = np.tile([-n * d, n * d], num_horizontal_lines)
            points[2 * num_horizontal_lines:, 1] = g.repeat(2)
            clr = [0.8] * 3 + [0.1] if clr_string is None else gl_color_from_matplotlib(clr_string, return_rgba=True)
            primitive = Primitive(positions=points, normals=None, color_0=clr, mode=GLTF.LINES, poses=None)
            meshes.append(pyrender.Mesh(primitives=[primitive]))
        elif shape.get_geometry_type() == GeometryType.CONVEXMESH:
            data = shape.get_shape_data()  # N x 9 - i.e. 3 triangles
            primitive = Primitive(positions=data.reshape(-1, 3), normals=None, color_0=clr, mode=GLTF.TRIANGLES,
                                  poses=None)
            meshes.append(pyrender.Mesh(primitives=[primitive]))
        else:
            raise NotImplementedError('Not supported type of the geometry.')
        return meshes

    def shape_to_nodes(self, shape):
        pose = pose_to_transformation_matrix(shape.get_local_pose())
        return [pyrender.Node(mesh=mesh, matrix=pose) for mesh in self.shape_to_meshes(shape=shape)]

    def actor_to_node(self, actor, flags):
        shapes = [s for s in actor.get_atached_shapes() if PyPhysxViewer.has_shape_any_of_flags(s, flags)]
        if len(shapes) == 0:
            return None
        all_nodes: List[pyrender.Node] = []
        for s in shapes:
            all_nodes += self.shape_to_nodes(s)
        return pyrender.Node(children=all_nodes)

    def add_physx_scene(self, scene, render_shapes_with_one_of_flags=(ShapeFlag.VISUALIZATION,), offset=np.zeros(3)):
        """ Call this function to create a renderer scene from physx scene. """
        actors = scene.get_dynamic_rigid_actors() + scene.get_static_rigid_actors()
        for i, actor in enumerate(actors):
            n = self.actor_to_node(actor, render_shapes_with_one_of_flags)
            if n is not None:
                self.nodes_and_actors.append((n, actor, offset))
                self.render_lock.acquire()
                self.scene.add_node(n)
                pose = multiply_transformations(offset, actor.get_global_pose())
                self.scene.set_pose(n, pose_to_transformation_matrix(pose))
                self.render_lock.release()

    def clear_physx_scenes(self):
        """ Remove all tracked actors and the corresponding nodes. """
        self.render_lock.acquire()
        for node, actor, offset in self.nodes_and_actors:  # type: pyrender.Node, RigidActor
            self.scene.remove_node(node)
        self.nodes_and_actors.clear()
        self.render_lock.release()

    def update(self, blocking=False):
        """ Update scene if lock can be acquired. Otherwise do nothing. Set blocking to True in order to force it. """
        if self.render_lock.acquire(blocking=blocking):
            for node, actor, offset in self.nodes_and_actors:  # type: pyrender.Node, RigidActor
                pose = multiply_transformations(offset, actor.get_global_pose())
                self.scene.set_pose(node, pose_to_transformation_matrix(pose))
            self.render_lock.release()

    def _set_axes(self, world, mesh):
        """ Use fixed scale for the axes instead of automatic. """
        scale = self.viewer_flags['axes_scale']
        if world:
            if 'scene' not in self._axes:
                n = pyrender.Node(mesh=self._axis_mesh, scale=np.ones(3) * scale)
                self.scene.add_node(n)
                self._axes['scene'] = n
        else:
            if 'scene' in self._axes:
                self.scene.remove_node(self._axes['scene'])
                self._axes.pop('scene')

        if mesh:
            old_nodes = []
            existing_axes = set([self._axes[k] for k in self._axes])
            for node in self.scene.mesh_nodes:
                if node not in existing_axes:
                    old_nodes.append(node)

            for node in old_nodes:
                if node in self._axes:
                    continue
                n = pyrender.Node(mesh=self._axis_mesh, scale=np.ones(3) * scale)
                self.scene.add_node(n, parent_node=node)
                self._axes[node] = n
        else:
            to_remove = set()
            for main_node in self._axes:
                if main_node in self.scene.mesh_nodes:
                    self.scene.remove_node(self._axes[main_node])
                    to_remove.add(main_node)
            for main_node in to_remove:
                self._axes.pop(main_node)

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        """Record a mouse drag. """
        self._trackball.drag(np.array([x, y]))
        self._trackball.down(np.array([x, y]))

    def save_gif(self, filename=None):
        """ Fix to allow mp4 video storing. """
        if filename is None:
            filename = self._get_save_filename(['gif', 'all'])
        if filename is not None:
            self.viewer_flags['save_directory'] = os.path.dirname(filename)
            if filename.endswith('.gif'):
                imageio.mimwrite(filename, self._saved_frames, fps=self.viewer_flags['refresh_rate'],
                                 palettesize=128, subrectangles=True)
            else:
                imageio.mimwrite(filename, self._saved_frames, fps=self.viewer_flags['refresh_rate'])
        self._saved_frames = []

    def add_label(self, text, location=TextAlign.CENTER, font_name='OpenSans-Regular', font_pt=40, color=None,
                  scale=1.0):
        """ Add label on the screen. Returns the label object that can be used to update the text. """
        if self.viewer_flags['caption'] is None:
            self.viewer_flags['caption'] = []

        self.render_lock.acquire()
        self.viewer_flags['caption'].append(
            dict(text=text, location=location, font_name=font_name, font_pt=font_pt,
                 color=gl_color_from_matplotlib(color, return_rgba=True),
                 scale=scale))
        self.render_lock.release()
        return self.viewer_flags['caption'][-1]

    def update_label_text(self, label, new_text):
        self.render_lock.acquire()
        label['text'] = new_text
        self.render_lock.release()

    def _location_to_x_y(self, location):
        """ Workaround to be able to select arbitrary text location by user. """
        if isinstance(location, tuple):
            return location
        return super()._location_to_x_y(location)

    def on_draw(self):
        super().on_draw()
        if self.video_writer is not None:
            data = self._renderer.read_color_buf()
            if not np.all(data == 0.0):
                self.video_writer.append_data(data)

    def close(self):
        super().close()
        if self.video_writer is not None:
            self.video_writer.close()
            self.video_writer = None

    def on_key_press(self, symbol, modifiers):
        move_keys = {
            pyglet.window.key.UP: np.array([-0.1, 0.0, 0.]),
            pyglet.window.key.DOWN: np.array([0.1, 0.0, 0.]),
            pyglet.window.key.LEFT: np.array([0.0, -0.1, 0.]),
            pyglet.window.key.RIGHT: np.array([0.0, 0.1, 0.]),
        }
        if symbol in move_keys.keys():
            self._trackball.move_target(move_keys[symbol])
        elif symbol == pyglet.window.key.PAGEUP:
            self._trackball.scroll(1)
        elif symbol == pyglet.window.key.PAGEDOWN:
            self._trackball.scroll(-1)

        super().on_key_press(symbol, modifiers)
