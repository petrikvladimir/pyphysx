#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 8/5/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
from queue import Queue
import numpy as np

from pyglet.window.key import LEFT, RIGHT, UP, DOWN
from pyphysx_render.render_window import PyPhysXWindow


class RenderWindowTestCase(unittest.TestCase):

    def test_render_look_at_key_modifications(self):
        self.queue = Queue()
        window = PyPhysXWindow(queue=self.queue, no_gl=True)
        self.assertAlmostEqual(0., np.linalg.norm(window.look_at))
        window.on_key_press(LEFT, None)
        self.assertAlmostEqual(1e-1, np.linalg.norm(window.look_at))
        self.assertNotAlmostEqual(0., np.linalg.norm(window.look_at))
        window.on_key_press(RIGHT, None)
        self.assertAlmostEqual(0., np.linalg.norm(window.look_at))

        self.assertAlmostEqual(0., np.linalg.norm(window.look_at))
        window.on_key_press(UP, None)
        self.assertAlmostEqual(1e-1, np.linalg.norm(window.look_at))
        self.assertNotAlmostEqual(0., np.linalg.norm(window.look_at))
        window.on_key_press(DOWN, None)
        self.assertAlmostEqual(0., np.linalg.norm(window.look_at))


if __name__ == '__main__':
    unittest.main()
