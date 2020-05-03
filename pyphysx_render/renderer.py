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
        #
        # sphr = Shape.create_sphere(0.1, Material())
        # print(sphr.get_shape_data())
        # self.shape = Shape.create_box([0.1] * 3, Material())
        # self.actor = RigidDynamic()
        # self.actor.set_global_pose((0, 0, 1))
        # self.actor.attach_shape(self.shape)
        # self.scene = Scene()
        # self.scene.add_actor(self.actor)
        #
        # self.shape_batch = batch_from_shape(self.shape)

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

        # glPushMatrix()
        # gl_transform(*self.actor.get_global_pose())
        # self.shape_batch.draw()
        # glPopMatrix()

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
