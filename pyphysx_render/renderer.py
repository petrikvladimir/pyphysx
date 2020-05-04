#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/3/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import pyglet
from pyphysx_render.utils import *
from pyglet.gl import *


class PyPhysXWindow(pyglet.window.Window):
    def __init__(self, scene, **kwargs):
        super(PyPhysXWindow, self).__init__(**kwargs)
        self.cam_pos_azimuth = np.deg2rad(10)
        self.cam_pos_elevation = np.deg2rad(45)
        self.cam_pos_distance = 2.
        self.look_at = np.zeros(3)
        self.view_up = np.array([0., 0., 1.])
        self.background_color_rgba = np.array([0.75] * 3 + [1.])

        self.static_batch = pyglet.graphics.Batch()
        add_ground_lines(self.static_batch, color=[0.8] * 3)
        add_coordinate_system(self.static_batch)

        self.scene = scene

        self.actors = self.scene.get_dynamic_rigid_actors()
        self.actor_batches_and_poses = []
        for actor in self.actors:
            self.actor_batches_and_poses.append(self.batches_and_local_poses_from_actor(actor))

    def on_resize(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(90, width / height, .01, 100)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glClearColor(*self.background_color_rgba)
        return pyglet.event.EVENT_HANDLED

    def on_draw(self):
        self.clear()
        glLoadIdentity()
        eye = Rotation.from_euler('ZY', [self.cam_pos_azimuth, -self.cam_pos_elevation]).apply(
            [self.cam_pos_distance, 0, 0])
        gluLookAt(*eye, *self.look_at, *self.view_up)
        self.static_batch.draw()

        for actor, batches_and_poses in zip(self.actors, self.actor_batches_and_poses):
            glPushMatrix()
            gl_transform(*actor.get_global_pose())
            for (batch, pose) in batches_and_poses:
                glPushMatrix()
                gl_transform(*pose)
                batch.draw()
                glPopMatrix()
            glPopMatrix()

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if buttons & pyglet.window.mouse.LEFT:
            self.cam_pos_azimuth -= dx * 1e-2
            self.cam_pos_elevation -= dy * 1e-2

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        self.cam_pos_distance = np.clip(self.cam_pos_distance - scroll_y * 1e-1, 0.015, 100.)

    def on_key_press(self, symbol, modifiers):
        super().on_key_press(symbol, modifiers)
        if symbol & pyglet.window.key.S:
            self.scene.simulate(0.1, 10)

    @staticmethod
    def batches_and_local_poses_from_actor(actor, color=None):
        """ Get batches that can be used to draw actor together with the local poses for all batches. """
        batches_and_poses = []
        for shape in actor.get_atached_shapes():
            batches_and_poses.append(
                (PyPhysXWindow.batch_from_shape(shape, color), shape.get_local_pose())
            )
        return batches_and_poses

    @staticmethod
    def batch_from_shape(shape, color=None):
        batch = pyglet.graphics.Batch()
        data = shape.get_shape_data()

        n = data.shape[1] // 3
        rtype = GL_QUADS if n == 4 else GL_TRIANGLES
        c = np.tile(gl_color_from_matplotlib(color), n)
        for d in data:
            batch.add(n, rtype, None, ('v3f', d), ('c3B', c))
        return batch
