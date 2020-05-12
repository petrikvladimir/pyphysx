#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/3/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from multiprocessing import Process, Queue


class PyPhysXParallelRenderer:

    def __init__(self, autostart=True, render_window_cls=None, render_window_kwargs=None) -> None:
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
        from pyphysx_render.render_window import PyPhysXWindow
        import pyglet
        if render_window_cls is None:
            render_window_cls = PyPhysXWindow
        default_render_window_kwargs = dict(fps=25, caption='PyPhysX Rendering', resizable=True)
        default_render_window_kwargs.update(render_window_kwargs)
        r = render_window_cls(queue, **default_render_window_kwargs)
        pyglet.app.run()

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
