#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 1/29/21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import numpy as np
import os
import unittest
from trimesh.creation import uv_sphere

from pyphysx import *
from pyphysx_render.pyrender_base import PyRenderBase
from pyphysx_render.pyrender_offscreen_renderer import PyPhysxOffscreenRenderer

os.environ['PYOPENGL_PLATFORM'] = 'osmesa'  # first set backend to headless mode


class TestPyRender(unittest.TestCase):
    def test_base_scene_creation(self):
        r = PyRenderBase()
        """ Default bg color """
        self.assertEqual(r.render_scene.bg_color[0], 0.75)

        """ Default camera position """
        pose = r.render_scene.get_pose(r.render_scene.main_camera_node)
        self.assertAlmostEqual(pose[0, 3], 3. * np.sin(np.deg2rad(45)))
        self.assertAlmostEqual(pose[1, 3], 0.)
        self.assertAlmostEqual(pose[2, 3], 3. * np.cos(np.deg2rad(45)))

        """ no actors """
        self.assertEqual(len(r.nodes_and_actors), 0)

    def test_scene_size(self):
        scene = Scene()
        actor = RigidDynamic()
        actor.attach_shape(Shape.create_box([0.2] * 3, Material()))
        scene.add_actor(actor)

        r = PyRenderBase()
        r.add_physx_scene(scene)
        self.assertEqual(len(r.nodes_and_actors), 1)
        r.clear_physx_scenes()
        self.assertEqual(len(r.nodes_and_actors), 0)

    def test_actor_pose_matrix(self):
        scene = Scene()
        actor = RigidDynamic()
        actor.attach_shape(Shape.create_box([0.2] * 3, Material()))
        actor.set_global_pose([1, 2, 3])
        scene.add_actor(actor)
        p1 = PyRenderBase._get_actor_pose_matrix(actor, offset=None)
        p2 = PyRenderBase._get_actor_pose_matrix(actor, offset=[3., 2., 1.])
        self.assertEqual(p1.shape, (4, 4))
        self.assertEqual(p2.shape, (4, 4))
        self.assertAlmostEqual(p1[0, 3], 1.)
        self.assertAlmostEqual(p1[1, 3], 2.)
        self.assertAlmostEqual(p1[2, 3], 3.)
        self.assertAlmostEqual(p2[0, 3], 4.)
        self.assertAlmostEqual(p2[1, 3], 4.)
        self.assertAlmostEqual(p2[2, 3], 4.)

    def test_update(self):
        scene = Scene()
        actor = RigidDynamic()
        actor.attach_shape(Shape.create_box([0.2] * 3, Material()))
        actor.set_global_pose([1, 2, 3])
        scene.add_actor(actor)
        r = PyRenderBase()
        r.add_physx_scene(scene)
        r.update()
        p1 = r.render_scene.get_pose(r.nodes_and_actors[0][0])
        self.assertAlmostEqual(p1[0, 3], 1.)
        self.assertAlmostEqual(p1[1, 3], 2.)
        self.assertAlmostEqual(p1[2, 3], 3.)
        actor.set_global_pose([4, 5, 6])
        p1 = r.render_scene.get_pose(r.nodes_and_actors[0][0])
        self.assertAlmostEqual(p1[0, 3], 1.)
        self.assertAlmostEqual(p1[1, 3], 2.)
        self.assertAlmostEqual(p1[2, 3], 3.)
        r.update()
        p1 = r.render_scene.get_pose(r.nodes_and_actors[0][0])
        self.assertAlmostEqual(p1[0, 3], 4.)
        self.assertAlmostEqual(p1[1, 3], 5.)
        self.assertAlmostEqual(p1[2, 3], 6.)

    def test_shape_any_of_flags(self):
        s = Shape.create_box([0.2] * 3, Material())
        s.set_flag(ShapeFlag.VISUALIZATION, True)
        s.set_flag(ShapeFlag.SIMULATION_SHAPE, True)
        s.set_flag(ShapeFlag.SCENE_QUERY_SHAPE, False)
        self.assertTrue(PyRenderBase.has_shape_any_of_flags(s, [ShapeFlag.VISUALIZATION]))
        self.assertTrue(PyRenderBase.has_shape_any_of_flags(s, [ShapeFlag.SIMULATION_SHAPE]))
        self.assertTrue(PyRenderBase.has_shape_any_of_flags(s, [ShapeFlag.SIMULATION_SHAPE, ShapeFlag.VISUALIZATION]))
        self.assertTrue(PyRenderBase.has_shape_any_of_flags(s, [ShapeFlag.VISUALIZATION, ShapeFlag.SCENE_QUERY_SHAPE]))
        self.assertFalse(PyRenderBase.has_shape_any_of_flags(s, [ShapeFlag.SCENE_QUERY_SHAPE]))
        self.assertFalse(PyRenderBase.has_shape_any_of_flags(s, []))

    def test_grid_lines(self):
        r = PyRenderBase()
        lines = r._grid_lines()
        self.assertEqual(lines.shape, (21 * 4, 3))
        self.assertAlmostEqual(lines[0, 1], -10.)
        self.assertAlmostEqual(lines[1, 1], 10.)
        self.assertAlmostEqual(lines[-2, 2], -10.)
        self.assertAlmostEqual(lines[-1, 2], 10.)

    def test_basic_shape_trimesh(self):
        r = PyRenderBase(spheres_count=(4, 4))
        s = Shape.create_box([0.2] * 3, Material())
        m = r._trimesh_from_basic_shape(s, vertex_colors=[10, 20, 30, 250])
        self.assertEqual(m.visual.vertex_colors.shape, (8, 4))  # box has 8 vertices
        self.assertEqual(m.visual.vertex_colors[5, 0], 10)
        self.assertEqual(m.visual.vertex_colors[5, 1], 20)
        self.assertEqual(m.visual.vertex_colors[5, 2], 30)
        self.assertEqual(m.visual.vertex_colors[5, 3], 250)

        s = Shape.create_sphere(0.15, Material())
        m = r._trimesh_from_basic_shape(s)
        err = np.max(np.abs(np.linalg.norm(m.vertices, axis=-1) - 0.15))  # error of sphere creation
        self.assertAlmostEqual(err, 0.)

        s.set_user_data({'visual_mesh': uv_sphere(1.)})  # set user data - should return this mesh instead of shape
        m = r._trimesh_from_basic_shape(s)
        err = np.max(np.abs(np.linalg.norm(m.vertices, axis=-1) - 1.))  # error of sphere creation
        self.assertAlmostEqual(err, 0.)

    def test_actor_to_node(self):
        r = PyRenderBase()

        actor = RigidDynamic()
        self.assertEqual(r.actor_to_node(actor, [ShapeFlag.VISUALIZATION]), None)

        actor.attach_shape(Shape.create_box([0.2] * 3, Material()))
        n = r.actor_to_node(actor, [ShapeFlag.VISUALIZATION])
        self.assertEqual(len(n.children), 1)

        actor.attach_shape(Shape.create_box([0.2] * 3, Material()))
        n = r.actor_to_node(actor, [ShapeFlag.VISUALIZATION])
        self.assertEqual(len(n.children), 2)

    def test_offscreen_renderer_shapes(self):
        r = PyPhysxOffscreenRenderer(viewport_size=(320, 240))
        rgb, depth = r.get_rgb_and_depth()
        self.assertEqual(rgb.shape, (240, 320, 3))
        self.assertEqual(depth.shape, (240, 320))
        rgba, depth = r.get_rgba_and_depth()
        self.assertEqual(rgba.shape, (240, 320, 4))
        self.assertEqual(depth.shape, (240, 320))
        depth = r.get_depth()
        self.assertEqual(depth.shape, (240, 320))


if __name__ == '__main__':
    unittest.main()
