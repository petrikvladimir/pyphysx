#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 3/25/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import time


class Rate:
    def __init__(self, frequency) -> None:
        self._frequency = frequency
        self.last_time = time.time()

    def frequency(self):
        return self._frequency

    def period(self):
        return 1. / self._frequency

    def sleep(self):
        elapsed = time.time() - self.last_time
        remaining = self.period() - elapsed
        time.sleep(max(remaining, 0))
        self.last_time = time.time()
