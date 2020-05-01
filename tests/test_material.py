#!/usr/bin/env python

# Copyright (c) CTU -- All Rights Reserved
# Created on: 2020-05-1
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
import sys

sys.path.append('lib')

from pyphysx import *


class MaterialTest(unittest.TestCase):

    def test_creation(self):
        mat = Material()
        self.assertAlmostEqual(mat.get_static_friction(), 0.)
        self.assertAlmostEqual(mat.get_dynamic_friction(), 0.)
        self.assertAlmostEqual(mat.get_restitution(), 0.)

        mat = Material(static_friction=0.2, dynamic_friction=0.3, restitution=0.5)
        self.assertAlmostEqual(mat.get_static_friction(), 0.2)
        self.assertAlmostEqual(mat.get_dynamic_friction(), 0.3)
        self.assertAlmostEqual(mat.get_restitution(), 0.5)

    def test_setters(self):
        mat = Material()
        mat.set_static_friction(0.2)
        mat.set_dynamic_friction(0.3)
        mat.set_restitution(0.5)

        self.assertAlmostEqual(mat.get_static_friction(), 0.2)
        self.assertAlmostEqual(mat.get_dynamic_friction(), 0.3)
        self.assertAlmostEqual(mat.get_restitution(), 0.5)


if __name__ == '__main__':
    unittest.main()
