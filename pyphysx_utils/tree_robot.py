#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# Class used to represent robots (links connected by joints) in a tree like structure.
# Parallel mechanism are therefore not supported.
#
import anytree
import numpy as np
from pyphysx_utils.transformations import multiply_transformations


class Joint:
    def __init__(self) -> None:
        super().__init__()
        self.value = 0.
        self.axis = [0, 0, 1]
        self.pos = [0, 0, 0]
        self.quat = [0, 0, 0, 1]


class Link(anytree.Node):

    def __init__(self, name):
        super().__init__(name)
        self.joint = Joint()
        self.actor = None


class TreeRobot:

    def __init__(self) -> None:
        super().__init__()
        self._root_node = None
        self.links = {}

    @property
    def root_node(self):
        if self._root_node is None:
            roots = [node for node in self.links.values() if node.is_root]
            assert len(roots) == 1
            self._root_node = roots[0]
        return self._root_node

    def add_link(self, link: Link):
        self.links[link.name] = link
        self._root_node = None

    def add_joint(self, parent_name: str, child_name: str, pos, quat, axis, init_value=0.):
        self.links[child_name].parent = self.links[parent_name]
        self.links[child_name].joint.pos = pos
        self.links[child_name].joint.quat = quat
        self.links[child_name].joint.axis = axis
        self.links[child_name].joint.val = init_value
        self._root_node = None

    def print_structure(self):
        for pre, fill, node in anytree.RenderTree(self.root_node):
            print("%s%s" % (pre, node.name))

    def compute_link_transformations(self):
        """ Compute transformations of all links and return then in a dictionary. """
        link_transforms = {}
        for link in anytree.LevelOrderIter(self.root_node):
            if link.is_root:
                link_transforms[link.name] = (np.zeros(3), np.array([0, 0, 0, 1]))
                continue
            parent_transform = link_transforms[link.parent.name]
            link_transforms[link.name] = multiply_transformations(*parent_transform, link.joint.pos, link.joint.quat)
            # todo add joint transformation
        return link_transforms
