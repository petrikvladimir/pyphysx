import xml.etree.ElementTree as ET
from pathlib import Path

from scipy.spatial.transform import Rotation as R
import numpy as np
from pyphysx_utils.transformations import *


class LinkUrdf:
    def __init__(self, name, mesh_path):
        self.name = name
        self.mesh_path = mesh_path


class JointUrdf:
    def __init__(self, name, parent, child, xyz, rpy, axis):
        self.name = name
        self.parent = parent
        self.child = child
        self.xyz = xyz
        self.rpy = rpy
        self.quat = R.from_euler('xyz', rpy).as_quat()
        self.axis = axis
        self.val = 0.


class Robot:
    def __init__(self, urdf_path):
        self.root = ET.parse(urdf_path).getroot()
        self.links = Robot.find_links(self.root)
        self.joints = Robot.find_joints(self.root)

    @staticmethod
    def find_links(root):
        #                 return {child.attrib['name']: child.find('visual').find('geometry').find('mesh').attrib['filename']
        #                                         if (child.find('visual') is not None) else None
        #                 for child in root if (child.tag == 'link')}
        return [LinkUrdf(child.attrib['name'], child.find('collision').find('geometry').find('mesh').attrib['filename']
        if (child.find('visual') is not None) else None)
                for child in root if (child.tag == 'link')]

    @staticmethod
    def find_joints(root):
        return [JointUrdf(child.attrib['name'], child.find('parent').attrib['link']
                          , child.find('child').attrib['link']
                          , [float(x) for x in child.find('origin').attrib['xyz'].split(' ')]
                          , [float(x) for x in child.find('origin').attrib['rpy'].split(' ')]
                          # , child.find('origin').attrib['rpy']
                          , child.find('axis').attrib['xyz'] if (child.find('axis') is not None) else None)
                for child in root if (child.tag == 'joint')]


import itertools
from pyphysx_render.rate import Rate
from pyphysx_render.renderer import PyPhysXParallelRenderer
from pyphysx import *
import trimesh
from trimesh.exchange.dae import load_collada

urdf_path = Path("franka_panda/panda.urdf")
robot = Robot(urdf_path)

scene = Scene()
scene.add_actor(RigidStatic.create_plane(mat=Material()))

# find root link (i.e. link without parent)

from pyphysx_utils.tree_robot import *

trobot = TreeRobot()
[trobot.add_link(Link(l.name)) for l in robot.links]
[trobot.add_joint(j.parent, j.child, j.xyz, j.quat, j.axis, 0.) for j in robot.joints]

trobot.print_structure()
link_transforms = trobot.compute_link_transformations()

links_dict = {}
for link in robot.links:
    if link.mesh_path is not None:
        actor = RigidDynamic()
        mesh_mat = Material()
        obj = trimesh.load(link.mesh_path.replace("package://", str(urdf_path.parent) + '/'), split_object=True,
                           group_material=False)
        if type(obj) == trimesh.scene.scene.Scene:
            shapes = [actor.attach_shape(Shape.create_convex_mesh_from_points(g.vertices, mesh_mat)) for g
                      in obj.geometry.values()]
        else:
            actor.attach_shape(Shape.create_convex_mesh_from_points(obj.vertices, mesh_mat))
        actor.set_global_pose(link_transforms[link.name][0], link_transforms[link.name][1])
        actor.set_mass(1.)
        # actor.disable_gravity()
    else:
        actor = RigidDynamic()
        mesh_mat = Material()
    scene.add_actor(actor)
    links_dict[link.name] = actor

render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='robot.mp4'))
render.render_scene(scene)

exit(1)

rate = Rate(25)
while render.is_running():
    scene.simulate(rate.period(), 1)
    render.render_scene(scene)
    rate.sleep()

exit(1)


def get_t_matrix(pos, quat, inverse=False):
    r = R.from_quat(quat).as_matrix()
    if inverse:
        r = r.T
        pos = -r.dot(pos)
    t = np.append(r, pos.reshape((3, 1)), axis=1)
    return np.append(t, np.array([0, 0, 0, 1]).reshape(1, 4), axis=0)


def multiplyTransforms(pos1, quat1, pos2, quat2):
    t1 = get_t_matrix(pos1, quat1)
    t2 = get_t_matrix(pos2, quat2)
    t = t1.dot(t2)
    pos = t[:3, 3]
    quat = R.from_matrix(t[:3, :3]).as_quat()
    return pos, quat


first_link_pos = [[0., 0., 0.], [0., 0., 0., 1.]]
parents = [joint.parent for joint in robot.joints]
children = [joint.child for joint in robot.joints]
main_parent = max(set(parents) - set(children))
position_dict = {}
position_dict[main_parent] = first_link_pos
joints_list_copy = robot.joints.copy()
joints_list_copy_copy = robot.joints.copy()
import numpy as np

while len(joints_list_copy_copy) > 0:
    for joint in joints_list_copy_copy:
        if joint.parent in position_dict:
            pos, quat = multiplyTransforms(np.array(position_dict[joint.parent][0]), position_dict[joint.parent][1],
                                           np.array(joint.xyz), joint.quat)
            print("Parent {} pose = {} {}, joint pose = {} {}, child {} pose = {} {}" \
                  .format(joint.parent, position_dict[joint.parent][0],
                          R.from_quat(position_dict[joint.parent][1]).as_euler("xyz", degrees=True),
                          joint.xyz, R.from_quat(joint.quat).as_euler("xyz", degrees=True), joint.child,
                          pos, R.from_quat(quat).as_euler("xyz", degrees=True)))
            position_dict[joint.child] = [pos, quat]
            joints_list_copy.remove(joint)
    joints_list_copy_copy = joints_list_copy.copy()

print(position_dict)
# count_actors = 0
# positions = [[0., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 0.], [1., 0., 1.], [1., 1., 0.],
#              [1., 0.5, 0.], [0.5, 1., 0.], [0.5, 0., 1.], [0., 0., 0.5], [0., 0.5, 0.], [0.5, 0., 0.],]
links_dict = {}
for link in robot.links:
    if link.mesh_path is not None:
        actor = RigidDynamic()
        mesh_mat = Material()
        # x = load_collada(link.mesh_path.replace("package://", "/home/kzorina/PandaRobot.jl/deps/"))
        # obj: trimesh.Scene = load_collada(link.mesh_path.replace("package://", "/home/kzorina/PandaRobot.jl/deps/"))
        obj = trimesh.load(link.mesh_path.replace("package://", str(urdf_path.parent) + '/'), split_object=True,
                           group_material=False)
        # trimesh.load(link.mesh_path.replace("package://", "/home/kzorina/PandaRobot.jl/deps/"), split_object=True, group_material=False)
        if type(obj) == trimesh.scene.scene.Scene:
            shapes = [actor.attach_shape(Shape.create_convex_mesh_from_points(g.vertices, mesh_mat)) for g
                      in obj.geometry.values()]
        else:
            actor.attach_shape(Shape.create_convex_mesh_from_points(obj.vertices, mesh_mat))
        actor.set_global_pose(position_dict[link.name][0], position_dict[link.name][1])
        actor.set_mass(1.)
        # actor.disable_gravity()
    else:
        actor = RigidDynamic()
        mesh_mat = Material()
    scene.add_actor(actor)
    links_dict[link.name] = actor
    # count_actors += 1
    # Shape.create_convex_mesh_from_points()
print(len(robot.joints))
# for joint in robot.joints:
#     joint_add = D6Joint(links_dict[joint.parent], links_dict[joint.child],
#                         local_pos0=robot.joints[0].xyz,
#                         local_quat0=robot.joints[0].quat)

# j2 = D6Joint(actors_bottom[1], actors_upper[1], local_pos0=[0., 0., 0.5])


render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='robot.gif'))

rate = Rate(25)
import time

while render.is_running():
    scene.simulate(rate.period(), 10)
    render.render_scene(scene)
    rate.sleep()
    time.sleep(10)
exit(1)
