#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 10/16/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
from pyphysx_render.pyrender import PyPhysxViewer
import pyphysx


class TestPyRender(unittest.TestCase):
    def test_sphere_count(self):
        m = 32
        n = 16
        render = PyPhysxViewer(viewer_flags={'spheres_count': [m, n]})
        self.assertEqual(32, render.viewer_flags['spheres_count'][0])
        self.assertEqual(16, render.viewer_flags['spheres_count'][1])

        mesh = render.shape_to_meshes(pyphysx.Shape.create_sphere(radius=0.1, material=pyphysx.Material()))
        self.assertEqual(1, len(mesh))
        self.assertEqual(1, len(mesh[0].primitives))
        self.assertEqual(2 * m * n - m, mesh[0].primitives[0].positions.shape[0])


if __name__ == '__main__':
    unittest.main()
