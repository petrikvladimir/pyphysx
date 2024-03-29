#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/22/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
from pyphysx_utils.rate import Rate
import time


class TestRate(unittest.TestCase):
    def test_period_frequency(self):
        r = Rate(25)
        self.assertAlmostEqual(r.period(), 1 / 25)
        self.assertAlmostEqual(r.frequency(), 25.)

    def test_sleep(self):
        r = Rate(0.2)
        creation_time = time.time()
        r.sleep()
        after_sleep_time = time.time()
        time.sleep(3.)  # majority of time in this part
        r.sleep()  # this should take just remaining time, s.t. after_sleep2_time - after_sleep is 5.
        after_sleep2_time = time.time()
        self.assertAlmostEqual(after_sleep_time - creation_time, 5.0, places=0)
        self.assertAlmostEqual(after_sleep2_time - after_sleep_time, 5.0, places=0)


if __name__ == '__main__':
    unittest.main()
