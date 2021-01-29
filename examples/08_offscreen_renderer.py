#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 1/28/21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import imageio
import numpy as np

from pyrender import PointLight, Node

from pyphysx import *
from pyphysx_render.pyrender_offscreen_renderer import PyPhysxOffscreenRenderer
from pyphysx_utils.rate import Rate

scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material(static_friction=0.1, dynamic_friction=0.1, restitution=0.5)))

actor = RigidDynamic()
actor.attach_shape(Shape.create_box([0.2] * 3, Material(restitution=1.)))
actor.set_global_pose([0.5, 0.5, 1.0])
actor.set_mass(1.)
scene.add_actor(actor)

render = PyPhysxOffscreenRenderer()

# Offscreen render has no lighting by default - that is opposite to the pyrender viewer. Therefore, we create some
# lighting to get nice rendered pictures.
render.render_scene.ambient_light = [0.1] * 3
render.render_scene.add_node(Node(light=PointLight(color=[0.2, 0.2, 1.0], intensity=3.0), matrix=np.eye(4)))

render.add_physx_scene(scene)  # add PyPhysX scene

frames = []
rate = Rate(60)
for _ in range(20):  # render 20 frames at 10 fps and store images into the list
    for _ in range(6):
        scene.simulate(rate.period())  # simulation runs at 60 Hz
    render.update()
    rgb, depth = render.get_rgb_and_depth()
    depth = np.clip((depth * 20), 0, 255).astype(dtype=np.uint8)
    frames.append(np.concatenate([rgb, np.repeat(depth[:, :, np.newaxis], 3, axis=-1)], axis=1))

imageio.mimwrite('videos/08_offscreen_renderer.gif', frames, fps=10)
