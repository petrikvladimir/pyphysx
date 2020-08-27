#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/16/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# Warning: This example requires a franka_panda urdf file that is not part of this repository.
#          You can download zip file containing urdf e.g. from:
#          https://drive.google.com/file/d/10OX-9UvbGtO63gnd_AoTmQog5xOdrirO/view?usp=sharing
#          Extract the zip file into examples folder.

import numpy as np

from pyphysx import *
from pyphysx_utils.rate import Rate
from pyphysx_utils.urdf_robot_parser import URDFRobot
from pyphysx_render.pyrender import PyPhysxViewer

""" Create scene """
scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material()))

""" Load robot """
robot = URDFRobot("franka_panda/panda.urdf", kinematic=True, use_random_collision_colors=True)
robot.attach_root_node_to_pose((0, 0, 0))  # attach base of the robot to the world origin
q = dict()
for i, value in enumerate([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]):
    q['panda_joint{}'.format(i + 1)] = value
robot.reset_pose(q)  # reset pose of all links based on default values for joints (0 m or 0 deg)
scene.add_aggregate(robot.get_aggregate())  # add robot into the scene, robot is an aggregate of actors with self

""" Configure renderer. """
render = PyPhysxViewer()
render.add_physx_scene(scene, offset=[0., -0.5, 0.])
render.add_physx_scene(scene, offset=[0., 0.5, 0.], render_shapes_with_one_of_flags=(ShapeFlag.SIMULATION_SHAPE,))

rate = Rate(30)
while render.is_active:
    robot.movable_joints['panda_joint1'].set_joint_velocity(np.deg2rad(45))

    """ Perform simulation.  """
    robot.update(rate.period())  # Robot update is necessary before simulation! It updates kinematic target and command.
    scene.simulate(rate.period())

    """ Render in real time. """
    render.update()
    rate.sleep()
