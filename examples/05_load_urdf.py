#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/16/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# Example showing how to load URDF robot and how to control it in position or velocity mode.

import numpy as np
from pyrender import TextAlign
from pyphysx import *
from pyphysx_utils.rate import Rate
from pyphysx_utils.urdf_robot_parser import URDFRobot
from pyphysx_render.pyrender import PyPhysxViewer

position_control = True  # Select position control (if True) or velocity control (if False)
kinematic_robot = False  # Dynamic robot simulation (if False) or only kinematic robot simulation (if True)

""" Create scene """
scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material()))

""" Load robot """
robot = URDFRobot("crane_robot.urdf", kinematic=kinematic_robot)  # load the robot and set if it is kinematic or dynamic
robot.attach_root_node_to_pose((0, 0, 0))  # attach base of the robot to the world origin
robot.reset_pose()  # reset pose of all links based on default values for joints (0 m or 0 deg)
scene.add_aggregate(robot.get_aggregate())  # add robot into the scene, robot is an aggregate of actors with self
# collisions turned off by default, use get_aggregate(enable_self_collision=True) if you want self collision checking.

""" Set all joints PD controller. This setting is ignored for kinematic robot because it is not needed. """
robot.movable_joints['rz'].configure_drive(stiffness=1e6, damping=1e5, force_limit=1e5, is_acceleration=False)
robot.movable_joints['tz'].configure_drive(stiffness=1e6, damping=1e5, force_limit=1e5, is_acceleration=False)
robot.movable_joints['ty'].configure_drive(stiffness=1e6, damping=1e5, force_limit=1e5, is_acceleration=False)

""" Configure renderer and add labels. """
render = PyPhysxViewer(video_filename='videos/05_load_urdf.gif')
render.add_physx_scene(scene)

label_shapes = render.add_label('Rendering visual shapes.', TextAlign.BOTTOM_LEFT, scale=0.5, color='tab:blue')
label_joints = render.add_label('', TextAlign.BOTTOM_RIGHT, scale=0.5, color='tab:green')

rate = Rate(120)
while render.is_active:
    """ Set robot commands based on the simulation time. Positions and velocity control should follow the same path. """
    t = scene.simulation_time
    if position_control:
        if 1 < t < 2:
            robot.movable_joints['tz'].set_joint_position((t - 1) * 0.5)
        if 2 < t < 3:
            robot.movable_joints['rz'].set_joint_position((t - 2) * np.deg2rad(180))
        if 3 < t < 4:
            robot.movable_joints['ty'].set_joint_position((t - 3) * 0.3)
    else:
        for joint in robot.movable_joints.values():
            joint.set_joint_velocity(0.)
        if 1 < t < 2:
            robot.movable_joints['tz'].set_joint_velocity(0.5)
        if 2 < t < 3:
            robot.movable_joints['rz'].set_joint_velocity(np.deg2rad(180))
        if 3 < t < 4:
            robot.movable_joints['ty'].set_joint_velocity(0.3)

    """ Render collision shapes instead of visual shapes if time is larger than 5 seconds. """
    if 5. < t < 5. + rate.period():
        render.clear_physx_scenes()
        render.add_physx_scene(scene, render_shapes_with_one_of_flags=(ShapeFlag.SIMULATION_SHAPE,))
        render.update_label_text(label_shapes, 'Rendering collision shapes')

    render.update_label_text(label_joints, 'Command: rz: {:.2f}, tz: {:.2f}, ty: {:.2f}'.format(
        robot.movable_joints['rz'].commanded_joint_position,
        robot.movable_joints['tz'].commanded_joint_position,
        robot.movable_joints['ty'].commanded_joint_position
    ))

    """ Perform simulation.  """
    robot.update(rate.period())  # Robot update is necessary before simulation! It updates kinematic target and command.
    scene.simulate(rate.period())

    """ Render in real time. """
    render.update()
    rate.sleep()
