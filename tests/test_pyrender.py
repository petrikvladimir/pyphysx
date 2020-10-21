#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 10/16/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
from pyphysx_render.pyrender import PyPhysxViewer
from pyphysx import *


class TestPyRender(unittest.TestCase):
    def test_sphere_count(self):
        m = 32
        n = 16
        render = PyPhysxViewer(viewer_flags={'spheres_count': [m, n]})
        self.assertEqual(32, render.viewer_flags['spheres_count'][0])
        self.assertEqual(16, render.viewer_flags['spheres_count'][1])

        mesh = render.shape_to_meshes(Shape.create_sphere(radius=0.1, material=Material()))
        self.assertEqual(1, len(mesh))
        self.assertEqual(1, len(mesh[0].primitives))
        self.assertEqual(2 * m * n - m, mesh[0].primitives[0].positions.shape[0])
        render.close_external()

    def test_clear_scene(self):
        scene = Scene()
        actor = RigidDynamic()
        actor.attach_shape(Shape.create_box([0.2] * 3, Material(restitution=1.)))
        scene.add_actor(actor)
        actor1 = RigidDynamic()
        actor1.attach_shape(Shape.create_box([0.2] * 3, Material(restitution=1.)))
        scene.add_actor(actor1)

        render = PyPhysxViewer()
        render.add_physx_scene(scene)
        self.assertEqual(2, len(render.nodes_and_actors))
        render.clear_physx_scenes()
        self.assertEqual(0, len(render.nodes_and_actors))
        render.close_external()


if __name__ == '__main__':
    unittest.main()
