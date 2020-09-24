#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 9/24/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# Warning: This example requires a franka_panda urdf file that is not part of this repository.
#          You can download zip file containing urdf e.g. from:
#          https://drive.google.com/file/d/10OX-9UvbGtO63gnd_AoTmQog5xOdrirO/view?usp=sharing
#          Extract the zip file into examples folder.
import time

import numpy as np

from pyphysx import *
from pyphysx_utils.rate import Rate
from pyphysx_utils.urdf_robot_parser import URDFRobot
from pyphysx_render.pyrender import PyPhysxViewer
from pyphysx_render.utils import gl_color_from_matplotlib

""" Create scene """
scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material(static_friction=0.2)))

""" Add cubes into the scene """
for i in range(7):
    actor = RigidDynamic()
    shape = Shape.create_box([0.1] * 3, Material(static_friction=0.05, dynamic_friction=0.05))
    shape.set_user_data({'color': gl_color_from_matplotlib(None, alpha=0.75, return_rgba=True).astype(np.float) / 255})
    actor.attach_shape(shape)
    actor.set_global_pose([0.3, 0.0, 0.05 + 0.1 * i])
    actor.set_mass(1.)
    scene.add_actor(actor)

""" Load robot """
robot = URDFRobot("franka_panda/panda.urdf", kinematic=True, use_random_collision_colors=True)
robot.attach_root_node_to_pose((0, 0, 0))  # attach base of the robot to the world origin
q = dict()
for i, value in enumerate([-np.pi / 2, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]):
    q['panda_joint{}'.format(i + 1)] = value
robot.reset_pose(q)  # reset pose of all links based on default values for joints (0 m or 0 deg)
scene.add_aggregate(robot.get_aggregate())  # add robot into the scene, robot is an aggregate of actors with self


""" Configure renderer. """
render = PyPhysxViewer(video_filename='videos/05b_panda_cubes.gif', viewer_flags={'show_world_axis': False})
render.scene.bg_color = np.array([0.3] * 3)
render._trackball.scroll(6)
render.add_physx_scene(scene)

rate = Rate(120)
while render.is_active:
    robot.movable_joints['panda_joint1'].set_joint_velocity(np.deg2rad(20))

    """ Perform simulation.  """
    robot.update(rate.period())  # Robot update is necessary before simulation! It updates kinematic target and command.
    scene.simulate(rate.period())

    """ Render in real time. """
    render.update()
    rate.sleep()
