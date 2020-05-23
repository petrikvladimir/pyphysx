#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/22/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import unittest
from pyphysx_render.rate import Rate


class TestRate(unittest.TestCase):
    def test_period_frequency(self):
        r = Rate(25)
        self.assertAlmostEqual(r.period(), 1 / 25)
        self.assertAlmostEqual(r.frequency(), 25.)


if __name__ == '__main__':
    unittest.main()
