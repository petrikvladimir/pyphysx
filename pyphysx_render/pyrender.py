#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 8/24/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#    Details: Render PhysX using PyRender package.

import os
import imageio
import pyglet
import numpy as np

from pyrender import Viewer, TextAlign, Node

from pyphysx_render.pyrender_base import PyRenderBase
from pyphysx_render.pyrender_trackball import RoboticTrackball
from pyphysx_render.utils import gl_color_from_matplotlib


class PyPhysxViewer(PyRenderBase, Viewer):
    def __init__(self, render_scene=None, viewport_size=None, render_flags=None, viewer_flags=None,
                 registered_keys=None, run_in_thread=True, video_filename=None, **kwargs):
        """ Use render_scene to specify camera or lighting or additional geometries if required. """
        _viewer_flags = {
            'view_center': np.zeros(3), 'window_title': 'PyPhysX Scene Viewer', 'show_world_axis': True,
            'show_mesh_axes': False, 'axes_scale': 0.5, 'use_raymond_lighting': True, 'plane_grid_spacing': 1.,
            'plane_grid_num_of_lines': 10, 'spheres_count': [8, 8], 'refresh_rate': 30.0,
        }
        if viewer_flags is not None:
            _viewer_flags.update(viewer_flags)
        fps = _viewer_flags['refresh_rate']
        self.video_writer = imageio.get_writer(video_filename, fps=fps) if video_filename is not None else None
        PyRenderBase.__init__(self, render_scene=render_scene,
                              plane_grid_spacing=_viewer_flags['plane_grid_spacing'],
                              plane_grid_num_of_lines=_viewer_flags['plane_grid_num_of_lines'],
                              spheres_count=_viewer_flags['spheres_count'])
        Viewer.__init__(self, scene=self.render_scene, viewport_size=viewport_size, render_flags=render_flags,
                        viewer_flags=_viewer_flags, registered_keys=registered_keys, run_in_thread=run_in_thread,
                        **kwargs)

    @property
    def is_active(self):
        """ Use viewer is active property to check if window is active. """
        return self._is_active

    def _acquire_lock(self, blocking=True):
        return self.render_lock.acquire(blocking=blocking)

    def _release_lock(self):
        self.render_lock.release()

    def _reset_view(self):
        super()._reset_view()
        self._trackball = RoboticTrackball(self._default_camera_pose, self.viewport_size, self._trackball._scale)

    def _set_axes(self, world, mesh):
        """ Use fixed scale for the axes instead of automatic. """
        scale = self.viewer_flags['axes_scale']
        if world:
            if 'scene' not in self._axes:
                n = Node(mesh=self._axis_mesh, scale=np.ones(3) * scale)
                self.render_scene.add_node(n)
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
                n = Node(mesh=self._axis_mesh, scale=np.ones(3) * scale)
                self.render_scene.add_node(n, parent_node=node)
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
                imageio.mimwrite(filename, self._saved_frames, duration=1000 // self.viewer_flags['refresh_rate'],
                                 palettesize=128, subrectangles=True)
            else:
                imageio.mimwrite(filename, self._saved_frames, duration=1000 // self.viewer_flags['refresh_rate'])
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
