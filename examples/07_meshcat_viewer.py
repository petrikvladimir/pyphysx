#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 1/21/21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from pyphysx import *

from pyphysx_render.meshcat_render import MeshcatViewer
from pyphysx_utils.rate import Rate
import trimesh
import numpy as np

from pyphysx_utils.urdf_robot_parser import URDFRobot

""" Construct a scene with various objects """
scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material(static_friction=0.1, dynamic_friction=0.1, restitution=0.5)))

actor = RigidDynamic()
actor.attach_shape(Shape.create_box([0.2] * 3, Material(restitution=1.)))
actor.set_global_pose([0.5, 0.5, 1.0])
actor.set_mass(1.)
scene.add_actor(actor)

actor = RigidDynamic()
sphere = Shape.create_sphere(0.1, Material(restitution=1.))
sphere.set_user_data(dict(color='tab:blue'))
actor.attach_shape(sphere)
actor.set_global_pose([-0.5, 0.5, 1.0])
actor.set_mass(1.)
scene.add_actor(actor)

actor = RigidDynamic()
obj: trimesh.Scene = trimesh.load('spade.obj', split_object=True, group_material=False)
for g in obj.geometry.values():
    actor.attach_shape(Shape.create_convex_mesh_from_points(g.vertices, Material(restitution=1.), scale=1e-3))
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

actor.set_global_pose([0.5, -0.5, 1.0])
actor.set_mass(1.)
scene.add_actor(actor)

robot = URDFRobot("franka_panda/panda.urdf", kinematic=False, use_random_collision_colors=True)
robot.attach_root_node_to_pose([0.0, 0.0, 1.0])
q = dict()
for i, value in enumerate([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]):
    q['panda_joint{}'.format(i + 1)] = value
robot.reset_pose(q)  # reset pose of all links based on default values for joints (0 m or 0 deg)
scene.add_aggregate(robot.get_aggregate())  # add robot into the scene, robot is an aggregate of actors with self


""" Create a viewer and add the scene into it. """
render = MeshcatViewer(wait_for_open=True, open_meshcat=True, show_frames=False)
render.add_physx_scene(scene)

""" Simulate forever. Note, that meschat viewer is always active. """
rate = Rate(120)
while render.is_active:
    robot.update(rate.period())
    scene.simulate(rate.period())
    render.update()
    rate.sleep()
