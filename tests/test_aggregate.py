#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/16/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>


import numpy as np
import unittest
import sys

sys.path.append('lib')

from pyphysx import *


class AggregateTestCase(unittest.TestCase):

    def test_add_remove_get_actors(self):
        actors = [RigidDynamic() for _ in range(6)]
        for i in range(6):
            actors[i].set_user_data(i)
        agg = Aggregate(5)
        self.assertEqual(0, len(agg.get_actors()))
        agg.add_actor(actors[0])
        self.assertEqual(1, len(agg.get_actors()))
        agg.add_actor(actors[1])
        agg.add_actor(actors[2])
        agg.add_actor(actors[3])
        agg.add_actor(actors[4])
        self.assertEqual(5, len(agg.get_actors()))

        self.assertAlmostEqual(3., agg.get_actors()[3].get_user_data())
        agg.remove_actor(actors[3])
        self.assertEqual(4, len(agg.get_actors()))
        self.assertAlmostEqual(4., agg.get_actors()[3].get_user_data())

        agg.add_actor(actors[3])
        agg.add_actor(actors[5])
        self.assertEqual(5, len(agg.get_actors()))


if __name__ == '__main__':
    unittest.main()
