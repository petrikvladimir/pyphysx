#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/16/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from pyphysx_render.renderer import PyPhysXParallelRenderer
from pyphysx_utils.rate import Rate
from pyphysx_utils.urdf_robot_parser import URDFRobot, quat_from_euler
from pyphysx import *
import numpy as np

scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material()))

robot = URDFRobot("crane_robot.urdf")
robot.attach_root_node_to_pose((0, 0, 0))
robot.reset_pose()
scene.add_aggregate(robot.get_aggregate())

robot.movable_joints['rz'].configure_drive(stiffness=1e6, damping=1e5, force_limit=1e5, is_acceleration=False)
robot.movable_joints['tz'].configure_drive(stiffness=1e6, damping=1e5, force_limit=1e5, is_acceleration=False)
robot.movable_joints['ty'].configure_drive(stiffness=1e6, damping=1e5, force_limit=1e5, is_acceleration=False)

render = PyPhysXParallelRenderer(render_window_kwargs=dict(
    # video_filename='load_urdf.mp4'
))
render.add_label(pose=((0.5, 0, 0), quat_from_euler('z', np.deg2rad(90))), color='tab:blue',
                 text='Rendering visual shapes.')
rate = Rate(120)

while render.is_running():
    t = scene.simulation_time
    if 1 < t < 2:
        robot.movable_joints['tz'].set_joint_position((t - 1) * 0.5)
    if 2 < t < 3:
        robot.movable_joints['rz'].set_joint_position((t - 2) * np.deg2rad(180))
    if 3 < t < 4:
        robot.movable_joints['ty'].set_joint_position((t - 3) * 0.3)

    if 5. < t < 5. + rate.period():
        render.update_labels_text(['Rendering collision shapes'])
        render.render_scene(scene, recompute_actors=True, render_shapes_with_one_of_flags=[ShapeFlag.SIMULATION_SHAPE])

    scene.simulate(rate.period())
    render.render_scene(scene)
    rate.sleep()
