#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 3/25/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import time


class Rate:
    def __init__(self, freq) -> None:
        self.sleep_duration = 1. / freq
        self.last_time = time.time()

    def period(self):
        return self.sleep_duration

    def sleep(self):
        elapsed = time.time() - self.last_time
        remaining = self.sleep_duration - elapsed
        time.sleep(max(remaining, 0))
        self.last_time = time.time()
