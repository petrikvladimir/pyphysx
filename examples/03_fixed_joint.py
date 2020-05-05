#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/5/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from pyphysx_render.rate import Rate
from pyphysx_render.renderer import PyPhysXParallelRenderer
from pyphysx import *

scene = Scene()
scene.add_actor(RigidStatic.create_plane(mat=Material()))

for dy in [0., 0.5]:
    a1 = RigidDynamic()
    a1.attach_shape(Shape.create_box([0.2] * 3, Material()))
    a1.set_global_pose([0.5, 0.0 + dy, 1.0])
    a1.set_mass(1.)
    scene.add_actor(a1)

    a2 = RigidDynamic()
    a2.attach_shape(Shape.create_box([0.1] * 3, Material()))
    a2.set_global_pose([0.5, 0.0 + dy, 1.5])
    a2.set_mass(1.)
    scene.add_actor(a2)

joint = D6Joint(a1, a2, local_pos0=[0., 0., 0.5])

render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='fixed_joint.gif'))
rate = Rate(25)
while render.is_running():
    scene.simulate(rate.period(), 1)
    render.render_scene(scene)
    rate.sleep()
