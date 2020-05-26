#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/4/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import trimesh
from pyphysx_render.renderer import PyPhysXParallelRenderer
from pyphysx_utils.rate import Rate
from pyphysx import *

actor = RigidDynamic()
mesh_mat = Material()
obj: trimesh.Scene = trimesh.load('spade.obj', split_object=True, group_material=False)
for g in obj.geometry.values():
    actor.attach_shape(Shape.create_convex_mesh_from_points(g.vertices, mesh_mat, scale=1e-3))
actor.set_global_pose([0.5, 0.5, 1.0])
actor.set_mass(1.)

# Add custom coloring to the shapes
for i, s in enumerate(actor.get_atached_shapes()):
    if i == 10:
        s.set_user_data(dict(color='tab:blue'))
    elif i == 9:
        s.set_user_data(dict(color='tab:grey'))
    elif i in [0, 1, 4, 5]:
        s.set_user_data(dict(color='tab:green'))
    else:
        s.set_user_data(dict(color='green'))

scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material(static_friction=0.1, dynamic_friction=0.1, restitution=0.5)))
scene.add_actor(actor)

render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='spade.gif'))

rate = Rate(240)
while render.is_running():
    scene.simulate(rate.period())
    render.render_scene(scene)
    rate.sleep()
