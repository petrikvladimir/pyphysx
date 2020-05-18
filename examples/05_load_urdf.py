#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/16/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from pyphysx_render.rate import Rate
from pyphysx_render.renderer import PyPhysXParallelRenderer
from pyphysx_utils.tree_robot import *

scene = Scene()
scene.add_actor(RigidStatic.create_plane(mat=Material()))

# URDF do not specify controller (drive) parameters, you need to tune and provide them by yourself.
drive_setup = {
    'panda_joint1': dict(stiffness=10000000, damping=1000, force_limit=1000, is_acceleration=False),
    'panda_joint2': dict(stiffness=10000000, damping=1000, force_limit=1000, is_acceleration=False),
    'panda_joint3': dict(stiffness=10000000, damping=1000, force_limit=1000, is_acceleration=False),
    'panda_joint4': dict(stiffness=1000000,  damping=1000, force_limit=1000, is_acceleration=False),
    'panda_joint5': dict(stiffness=100000,   damping=1000, force_limit=1000, is_acceleration=False),
    'panda_joint6': dict(stiffness=100000,   damping=1000, force_limit=1000, is_acceleration=False),
    'panda_joint7': dict(stiffness=100000,   damping=1000, force_limit=1000, is_acceleration=False),
}
robot = URDFRobot("franka_panda/panda_tmp.urdf", attach_to_world_pos=(0, 0, 0), joints_drive_setup=drive_setup)
scene.add_aggregate(robot.get_aggregate())

q = {}
for name in robot.get_joint_names():
    q[name] = 0.0
q['panda_joint4'] = np.deg2rad(-90.)
q['panda_joint6'] = np.deg2rad(90.)
robot.reset_actor_poses(joint_values=q)
robot.set_joint_drive_values(q)

render = PyPhysXParallelRenderer(render_window_kwargs=dict(
    video_filename='load_urdf.mp4'
))
render.render_scene(scene)

rate = Rate(120)
while render.is_running():
    scene.simulate(rate.period())
    q['panda_joint1'] += np.deg2rad(120.) * rate.period()  # rotate 120deg per second
    robot.set_joint_drive_values(q)
    render.render_scene(scene)
    rate.sleep()
