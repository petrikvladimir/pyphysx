#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/4/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>


from pyphysx_render.renderer import PyPhysXParallelRenderer
from pyphysx_utils.rate import Rate
from pyphysx import *

scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material(static_friction=0.1, dynamic_friction=0.1, restitution=0.5)))

actor = RigidDynamic()
actor.attach_shape(Shape.create_box([0.2] * 3, Material(restitution=1.)))
actor.set_global_pose([0.5, 0.5, 1.0])
actor.set_mass(1.)
scene.add_actor(actor)

render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='free_fall.gif'))

rate = Rate(240)
while render.is_running():
    scene.simulate(rate.period())
    render.render_scene(scene)
    rate.sleep()
