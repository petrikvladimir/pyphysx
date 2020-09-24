#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 9/24/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>


import unittest
from pyphysx_render.utils import gl_color_from_matplotlib


class TestRenderUtils(unittest.TestCase):
    def test_color(self):
        c = gl_color_from_matplotlib(color='black')
        self.assertEqual(len(c), 3)
        self.assertEqual(c[0], 0)
        self.assertEqual(c[1], 0)
        self.assertEqual(c[2], 0)

        c = gl_color_from_matplotlib(color='white', alpha=0.5, return_rgba=True)
        self.assertEqual(len(c), 4)
        self.assertEqual(c[0], 255)
        self.assertEqual(c[1], 255)
        self.assertEqual(c[2], 255)
        self.assertEqual(c[3], 127)


if __name__ == '__main__':
    unittest.main()
