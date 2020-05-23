#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/22/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
from pyphysx_render.rate import Rate
import time


class TestRate(unittest.TestCase):
    def test_period_frequency(self):
        r = Rate(25)
        self.assertAlmostEqual(r.period(), 1 / 25)
        self.assertAlmostEqual(r.frequency(), 25.)

    def test_sleep(self):
        r = Rate(10)
        creation_time = time.time()
        r.sleep()
        after_sleep_time = time.time()
        time.sleep(0.09)  # majority of time in this part
        r.sleep()  # this should take just remaining time, s.t. after_sleep2_time - after_sleep is 0.1
        after_sleep2_time = time.time()
        self.assertAlmostEqual(after_sleep_time - creation_time, 0.1, places=2)
        self.assertAlmostEqual(after_sleep2_time - after_sleep_time, 0.1, places=2)


if __name__ == '__main__':
    unittest.main()
