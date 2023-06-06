#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 3/13/23
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from skbuild import setup
import os

if __name__ == '__main__':
    setup(
        cmake_args=['-DENABLE_COVERAGE=On'] if os.environ.get('PYPHYSX_COV') == 'true' else ()
    )
