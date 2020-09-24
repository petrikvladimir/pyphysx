#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/2/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import random
import numpy as np
import matplotlib.colors as mcolors


def gl_color_from_matplotlib(color=None, alpha=None, return_rgba=False):
    """ Get color from matplotlib color. If color is none, use random from tab palette."""
    color = random.choice(list(mcolors.TABLEAU_COLORS.items()))[0] if color is None else color
    if return_rgba:
        return (np.array(mcolors.to_rgba(color, alpha=alpha)) * 255.).astype(np.int)
    return (np.array(mcolors.to_rgb(color)) * 255.).astype(np.int)
