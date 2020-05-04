#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/4/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>


from pyphysx_render.renderer import PyPhysXParallelRenderer
from pyphysx_render.rate import Rate
from pyphysx import *

scene = Scene()
scene.add_actor(RigidStatic.create_plane(mat=Material(static_friction=0.1, dynamic_friction=0.1)))

actor = RigidDynamic()
actor.attach_shape(Shape.create_box([0.2] * 3, Material(restitution=1.)))
actor.set_global_pose([0., 0.1, 1.0])
scene.add_actor(actor)

actor = RigidDynamic()
actor.attach_shape(Shape.create_box([0.1, 0.1, 0.5], Material()))
scene.add_actor(actor)

render = PyPhysXParallelRenderer()

rate = Rate(25)
for i in range(100):
    scene.simulate(rate.period(), 10)
    render.render_scene(scene)
    rate.sleep()
