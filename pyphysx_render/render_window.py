#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/12/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import pyglet
from pyglet.gl import *
from queue import Empty
from multiprocessing import Queue
import imageio

from pyphysx_render.render_windows_interface import PyPhysXWindowInterface
from pyphysx_render.utils import *
from pyphysx_utils.transformations import quat_from_euler


class PyPhysXWindow(pyglet.window.Window, PyPhysXWindowInterface):
    def __init__(self, queue: Queue, fps=25, video_filename=None, coordinates_scale=1., coordinate_lw=10.,
                 cam_pos_azimuth=np.deg2rad(10), cam_pos_elevation=np.deg2rad(45), cam_pos_distance=2., **kwargs):
        super(PyPhysXWindow, self).__init__(**kwargs)
        self.cam_pos_azimuth = cam_pos_azimuth
        self.cam_pos_elevation = cam_pos_elevation
        self.cam_pos_distance = cam_pos_distance
        self.look_at = np.zeros(3)
        self.view_up = np.array([0., 0., 1.])
        self.background_color_rgba = np.array([0.75] * 3 + [1.])
        self.fps = fps

        self.static_batch = pyglet.graphics.Batch()
        add_ground_lines(self.static_batch, color=[0.8] * 3)

        self.static_coordinate_system_batch = pyglet.graphics.Batch()
        if coordinates_scale > 0:
            add_coordinate_system(self.static_coordinate_system_batch, scale=coordinates_scale)
        self.coordinate_lw = coordinate_lw

        self.queue = queue
        pyglet.clock.schedule_interval(self.update, 1 / self.fps)
        self.actors = {}

        self.plot_frames = False
        self.plot_geometry = True
        self.plot_labels = True

        self.labels = []

        self.video_filename = video_filename
        self.vid_imgs = []

        glEnable(GL_DEPTH_TEST)
        glEnable(pyglet.gl.GL_BLEND)
        glBlendFunc(pyglet.gl.GL_SRC_ALPHA, pyglet.gl.GL_ONE_MINUS_SRC_ALPHA)

    def on_resize(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(90, width / height, .01, 100)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glClearColor(*self.background_color_rgba)
        return pyglet.event.EVENT_HANDLED

    def get_eye_pos(self):
        v = [self.cam_pos_distance, 0, 0]
        return npq.rotate_vectors(quat_from_euler('ZY', [self.cam_pos_azimuth, -self.cam_pos_elevation]), v)

    def plot_coordinate_system(self):
        glLineWidth(self.coordinate_lw)
        self.static_coordinate_system_batch.draw()
        glLineWidth(1)

    def on_draw(self):
        self.clear()
        glLoadIdentity()
        gluLookAt(*self.get_eye_pos(), *self.look_at, *self.view_up)
        self.static_batch.draw()
        self.plot_coordinate_system()

        if self.plot_labels:
            for (pose, scale, batch) in self.labels:
                glPushMatrix()
                gl_transform(pose, scale)
                batch.draw()
                glPopMatrix()

        for actor in self.actors.values():
            if actor['pose'] is None:
                continue
            glPushMatrix()
            gl_transform(actor['pose'])
            if self.plot_frames:
                self.plot_coordinate_system()
            if self.plot_geometry:
                for batch, local_pose in zip(actor['batches'], actor['local_poses']):
                    glPushMatrix()
                    gl_transform(local_pose)
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
        if symbol is pyglet.window.key.F:
            self.plot_frames = not self.plot_frames
        elif symbol is pyglet.window.key.G:
            self.plot_geometry = not self.plot_geometry
        elif symbol is pyglet.window.key.L:
            self.plot_labels = not self.plot_labels

    def update(self, dt):
        try:
            while True:  # get all data from the queue before rendering
                cmd, data = self.queue.get(block=False)
                getattr(self, cmd)(*data[0], **data[1])
        except Empty:
            pass

        self.on_draw()
        if self.video_filename is not None:
            ibar = pyglet.image.get_buffer_manager().get_color_buffer().get_image_data()
            np_img = np.flip(np.asanyarray(ibar.get_data()).reshape(ibar.height, ibar.width, 4), axis=0)
            self.vid_imgs.append(np_img)

    @staticmethod
    def batch_from_shape(shape, color=None):
        return PyPhysXWindow.batch_from_shape_data(shape.get_shape_data(), color)

    @staticmethod
    def batch_from_shape_data(data, color=None):
        batch = pyglet.graphics.Batch()
        n = data.shape[1] // 3
        rtype = GL_QUADS if n == 4 else GL_TRIANGLES
        c = np.tile(gl_color_from_matplotlib(color, return_rgba=True), n)
        for d in data:
            batch.add(n, rtype, None, ('v3f', d), ('c4B', c))
        return batch

    def add_label(self, pose=None, scale=1., text='', font_name=None, font_size=None,
                  bold=False, italic=False, color='green', alpha=1., x=0, y=0, width=None, height=None, anchor_x='left',
                  anchor_y='baseline', align='left'):
        label = pyglet.text.Label(text, font_name, font_size, bold, italic,
                                  tuple(gl_color_from_matplotlib(color, return_rgba=True, alpha=alpha)),
                                  x, y, width, height, anchor_x, anchor_y, align)
        self.labels.append((pose if pose is not None else unit_pose(), np.array(scale) * 1e-2, label))

    def clear_labels(self):
        self.labels.clear()

    def update_labels_text(self, texts):
        assert len(texts) == len(self.labels)
        label: pyglet.text.Label
        for (pose, scale, label), text in zip(self.labels, texts):
            if text is not None:
                label.text = text

    def close(self):
        super().close()
        if self.video_filename is not None:
            print("Saving {}-frames video into: {}".format(len(self.vid_imgs), self.video_filename))
            imageio.mimsave(self.video_filename, self.vid_imgs, fps=self.fps)

    def add_actor_geometry(self, actor_id, geometry_data, local_pose, color=None):
        if actor_id not in self.actors:
            self.actors[actor_id] = dict(batches=[], local_poses=[], pose=None)
        self.actors[actor_id]['batches'].append(self.batch_from_shape_data(geometry_data, color))
        self.actors[actor_id]['local_poses'].append(local_pose)

    def clear_actors(self):
        self.actors.clear()

    def set_actor_pose(self, actor_id, pose):
        if actor_id in self.actors:
            self.actors[actor_id]['pose'] = pose
