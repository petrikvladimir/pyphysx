#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 1/28/21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from pyrender import OffscreenRenderer, RenderFlags
from pyphysx_render.pyrender_base import PyRenderBase


class PyPhysxOffscreenRenderer(PyRenderBase, OffscreenRenderer):

    def __init__(self, render_scene=None, viewport_size=None) -> None:
        viewport_width, viewport_height = viewport_size if viewport_size is not None else (640, 480)
        PyRenderBase.__init__(self, render_scene=render_scene)
        OffscreenRenderer.__init__(self, viewport_width=viewport_width, viewport_height=viewport_height, point_size=1.0)

    def get_rgb_and_depth(self):
        return self.render(self.render_scene)

    def get_depth(self):
        return self.render(self.render_scene, RenderFlags.DEPTH_ONLY)

    def get_rgba_and_depth(self):
        return self.render(self.render_scene, RenderFlags.RGBA)
