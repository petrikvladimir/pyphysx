#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/5/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import itertools
from pyphysx_render.rate import Rate
from pyphysx_render.renderer import PyPhysXParallelRenderer
from pyphysx import *

scene = Scene()
scene.add_actor(RigidStatic.create_plane(mat=Material()))

# Create actors and add them to the scene
actors_bottom = [RigidDynamic() for _ in range(3)]
actors_upper = [RigidDynamic() for _ in range(3)]
for a in itertools.chain(actors_bottom, actors_upper):
    scene.add_actor(a)

# attach shapes and set mass
for a in actors_bottom:
    a.attach_shape(Shape.create_box([0.2] * 3, Material()))
    a.set_mass(1.)
for a in actors_upper:
    a.attach_shape(Shape.create_box([0.2] * 3, Material()))
    a.set_mass(1.)

# set poses
for i, a in enumerate(actors_bottom):
    a.set_global_pose([0.5, 0.5 * i - 0.5, 1.0])
for i, a in enumerate(actors_upper):
    a.set_global_pose([0.5, 0.5 * i - 0.5, 1.5])

# create joints
# 1. No joint
j1 = None
# 2. Fixed joint
j2 = D6Joint(actors_bottom[1], actors_upper[1], local_pos0=[0., 0., 0.5])
# 3. Prismatic joint with drive damping
j3 = D6Joint(actors_bottom[2], actors_upper[2], local_pos0=[0., 0., 0.5])
j3.set_motion(D6Axis.Z, D6Motion.FREE)  # unlock z axis linear motion
j3.set_drive(D6Drive.Z, stiffness=0., damping=100., force_limit=100)  # add drive with damping

render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='joint.gif', coordinates_scale=0))
rate = Rate(25)
while render.is_running():
    scene.simulate(rate.period(), 1)
    render.render_scene(scene)
    rate.sleep()
