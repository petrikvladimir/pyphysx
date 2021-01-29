#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 1/21/21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from abc import abstractmethod
from typing import List

from pyphysx import ShapeFlag, Shape
from pyphysx_render.utils import gl_color_from_matplotlib


class ViewerBase:

    @property
    def is_active(self):
        """ Return true if viewer is active. """
        return True

    @abstractmethod
    def add_physx_scene(self, scene, render_shapes_with_one_of_flags=(ShapeFlag.VISUALIZATION,), offset=None):
        """
        Add actors from the given scene into the viewer.
        Actors are filtered based on the given flags and poses are moved by given offset if specified.
        """
        raise NotImplementedError("")

    @abstractmethod
    def update(self, blocking=False):
        """
        Update rendered poses based on the simulated actor positions.
        Update is forced if blocking is true - otherwise it depends on mutex acquire statues.
        """
        raise NotImplementedError("")

    @abstractmethod
    def clear_physx_scenes(self):
        """ Remove all tracked actors and the corresponding nodes. """
        raise NotImplementedError("")

    @staticmethod
    def get_shape_color(shape: Shape):
        """ Return color of the shape specified by user_data or random if not specified. """
        clr_string = shape.get_user_data().get('color', None) if shape.get_user_data() is not None else None
        return gl_color_from_matplotlib(color=clr_string, return_rgba=True)

    @staticmethod
    def has_shape_any_of_flags(shape: Shape, flags: List[ShapeFlag]):
        """ Return true if shape contains any of the flag from the list. """
        for show_flag in flags:
            if shape.get_flag_value(show_flag):
                return True
        return False
