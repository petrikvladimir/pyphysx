#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/1/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import numpy as np
import unittest
import sys

sys.path.append('lib')

from pyphysx import *


class SceneTestCase(unittest.TestCase):

    def test_simulation_free_fall(self):
        actor = RigidDynamic()
        scene = Scene()
        scene.add_actor(actor)
        for _ in range(480):
            scene.simulate(dt=0.5 / 480)
        expected_distance = -0.5 * 9.81 * scene.simulation_time ** 2
        self.assertAlmostEqual(actor.get_global_pose()[0][2], expected_distance, places=2)

    def test_get_actors(self):
        scene = Scene()
        r1 = RigidDynamic()
        r2 = RigidDynamic()
        r3 = RigidDynamic()
        r1.set_mass(1.)
        r2.set_mass(2.)
        r3.set_mass(3.)
        scene.add_actor(r1)
        scene.add_actor(r2)
        scene.add_actor(r3)
        actors = scene.get_dynamic_rigid_actors()
        self.assertEqual(3, len(actors))
        self.assertAlmostEqual(1., actors[0].get_mass())
        self.assertAlmostEqual(2., actors[1].get_mass())
        self.assertAlmostEqual(3., actors[2].get_mass())

    def test_get_aggregates(self):
        scene = Scene()
        agg = Aggregate()
        agg.add_actor(RigidDynamic())
        agg.add_actor(RigidDynamic())
        agg.add_actor(RigidDynamic())
        scene.add_aggregate(agg)
        agg = Aggregate()
        agg.add_actor(RigidDynamic())
        agg.add_actor(RigidDynamic())
        agg.add_actor(RigidDynamic())
        scene.add_aggregate(agg)

        self.assertEqual(2, len(scene.get_aggregates()))


if __name__ == '__main__':
    unittest.main()
