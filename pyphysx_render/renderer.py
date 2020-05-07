#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/3/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from queue import Empty
from multiprocessing import Process, Queue
import pyglet
from pyglet.gl import *
from pyphysx_render.utils import *
import imageio


class PyPhysXWindow(pyglet.window.Window):
    def __init__(self, queue: Queue, fps=25, video_filename=None, coordinates_scale=1., coordinate_lw=10.,
                 cam_pos_azimuth=np.deg2rad(10), cam_pos_elevation=np.deg2rad(45), cam_pos_distance=2.,
                 **kwargs):
        super(PyPhysXWindow, self).__init__(**kwargs)
        self.cam_pos_azimuth, self.cam_pos_elevation = cam_pos_azimuth, cam_pos_elevation,
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
        self.actors_global_pose, self.actors_batches_and_poses = [], []

        self.plot_frames = False
        self.plot_geometry = True

        self.video_filename = video_filename
        self.vid_imgs = []

        glEnable(GL_DEPTH_TEST)

    def on_close(self):
        super().on_close()
        if self.video_filename is not None:
            print("Saving {}-frames video into: {}".format(len(self.vid_imgs), self.video_filename))
            imageio.mimsave(self.video_filename, self.vid_imgs, fps=self.fps)

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
        return Rotation.from_euler('ZY', [self.cam_pos_azimuth, -self.cam_pos_elevation]).apply(v)

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

        if len(self.actors_global_pose) == len(self.actors_batches_and_poses):
            for global_pose, batches_and_poses in zip(self.actors_global_pose, self.actors_batches_and_poses):
                glPushMatrix()
                gl_transform(*global_pose)
                if self.plot_frames:
                    self.plot_coordinate_system()
                if self.plot_geometry:
                    for (batch, local_pose) in batches_and_poses:
                        glPushMatrix()
                        gl_transform(*local_pose)
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
        if symbol is pyglet.window.key.G:
            self.plot_geometry = not self.plot_geometry

    def update(self, dt):
        try:
            while True:  # get all data from the queue before rendering
                cmd, data = self.queue.get(block=False)
                if cmd == 'geometry':
                    self.actors_batches_and_poses = []
                    for actor_shapes_and_poses in data:
                        self.actors_batches_and_poses.append(
                            [(self.batch_from_shape_data(data), local_pose) for data, local_pose in
                             actor_shapes_and_poses]
                        )
                elif cmd == 'poses':
                    self.actors_global_pose = data
                elif cmd == 'close':
                    self.on_close()
                    self.close()
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
        c = np.tile(gl_color_from_matplotlib(color), n)
        for d in data:
            batch.add(n, rtype, None, ('v3f', d), ('c3B', c))
        return batch


class PyPhysXParallelRenderer:

    def __init__(self, autostart=True, render_window_cls=PyPhysXWindow, render_window_kwargs=None) -> None:
        super().__init__()
        self.queue = Queue()
        self.process = Process(target=self.start_rendering_f,
                               args=(self.queue, render_window_cls, render_window_kwargs))
        self.actors = None
        self.actors_shapes_data_and_local_poses = None
        if autostart:
            self.start()

    def is_running(self):
        return self.process.is_alive()

    def wait_for_finnish(self):
        self.process.join()

    def start(self):
        self.process.start()

    @staticmethod
    def start_rendering_f(queue, render_window_cls, render_window_kwargs):
        default_render_window_kwargs = dict(fps=25, caption='PyPhysX Rendering', resizable=True)
        default_render_window_kwargs.update(render_window_kwargs)
        r = render_window_cls(queue, **default_render_window_kwargs)
        pyglet.app.run()

    def close(self):
        self.queue.put(('close', None))

    def render_scene(self, scene, recompute_actors=False):
        if recompute_actors or self.actors is None:
            self.actors = scene.get_dynamic_rigid_actors()
            actors_shapes_data_and_local_poses = []
            for actor in self.actors:
                actors_shapes_data_and_local_poses.append(
                    [(shape.get_shape_data(), shape.get_local_pose()) for shape in actor.get_atached_shapes()]
                )
            self.queue.put(('geometry', actors_shapes_data_and_local_poses))

        actors_global_pose = [a.get_global_pose() for a in self.actors]
        self.queue.put(('poses', actors_global_pose))
