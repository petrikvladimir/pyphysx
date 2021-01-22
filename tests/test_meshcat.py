#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 1/22/21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>


import numpy as np
import unittest
import sys

from pyphysx_render.meshcat_render import MeshcatViewer

sys.path.append('lib')

from pyphysx import *


class MeschatTest(unittest.TestCase):

    def test_vis_group(self):
        viewer = MeshcatViewer(object_prefix='asdf')
        self.assertEqual(viewer.vis_group.path.entries[-1], 'asdf')

    def test_scene_size(self):
        scene = Scene()
        actor = RigidDynamic()
        actor.attach_shape(Shape.create_box([0.2] * 3, Material()))
        scene.add_actor(actor)

        viewer = MeshcatViewer()
        viewer.add_physx_scene(scene)
        self.assertEqual(len(viewer.actors_and_offsets), 1)
        viewer.clear_physx_scenes()
        self.assertEqual(len(viewer.actors_and_offsets), 0)

    def test_geometry_material(self):
        viewer = MeshcatViewer()
        box = Shape.create_box([0.2] * 3, Material())
        box.set_user_data(dict(color='#ff0000ff'))
        mat = viewer._get_shape_material(box)
        self.assertEqual(mat.color, int('ff0000', base=16))
        self.assertAlmostEqual(mat.opacity, 1.)

    def test_geometry_material2(self):
        viewer = MeshcatViewer()
        box = Shape.create_box([0.2] * 3, Material())
        box.set_user_data(dict(color='#00ff0000'))
        mat = viewer._get_shape_material(box)
        self.assertEqual(mat.color, int('00ff00', base=16))
        self.assertAlmostEqual(mat.opacity, 0.)

    def test_geometry_shape_box(self):
        viewer = MeshcatViewer()
        s = Shape.create_box([0.2, 0.1, 0.05], Material())
        g = viewer._get_shape_geometry(s)
        self.assertAlmostEqual(g.lengths[0], 0.2)
        self.assertAlmostEqual(g.lengths[1], 0.1)
        self.assertAlmostEqual(g.lengths[2], 0.05)

    def test_geometry_shape_sphere(self):
        viewer = MeshcatViewer()
        s = Shape.create_sphere(0.5, Material())
        g = viewer._get_shape_geometry(s)
        self.assertAlmostEqual(g.radius, 0.5)

    def test_geometry_shape_convex_shape(self):
        viewer = MeshcatViewer()
        points = np.random.randn(10, 3)
        s = Shape.create_convex_mesh_from_points(points, Material())
        g = viewer._get_shape_geometry(s)
        pmin = np.min(points, axis=0)
        pmax = np.max(points, axis=0)
        gmin = np.min(g.vertices, axis=0)
        gmax = np.max(g.vertices, axis=0)
        self.assertAlmostEqual(pmin[0], gmin[0], places=5)
        self.assertAlmostEqual(pmin[1], gmin[1], places=5)
        self.assertAlmostEqual(pmin[2], gmin[2], places=5)
        self.assertAlmostEqual(pmax[0], gmax[0], places=5)
        self.assertAlmostEqual(pmax[1], gmax[1], places=5)
        self.assertAlmostEqual(pmax[2], gmax[2], places=5)


if __name__ == '__main__':
    unittest.main()
