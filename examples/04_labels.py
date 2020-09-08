#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import time
from pyrender import TextAlign
from pyphysx_render.pyrender import PyPhysxViewer

viewer = PyPhysxViewer(viewer_flags={'show_world_axis': False}, video_filename='videos/04_labels.gif')

# Add label to the center of the screen. Will be kept in the center during window resizing.
viewer.add_label('Hello world!', TextAlign.CENTER, color='tab:blue', scale=1)

# Add label at the specific location given in pixels. Note that for location given in pixels bottom-left anchor is used.
loc = viewer._location_to_x_y(TextAlign.CENTER)
counter_label = viewer.add_label('i=0', (loc[0] - 20, loc[1] - 50), color='tab:green')

for i in range(100):
    viewer.update_label_text(counter_label, 'i={}'.format(i))  # update label text, for updating other parameters,
    # acquire viewer lock and change them in the label dictionary directly
    time.sleep(0.1)
    if not viewer.is_active:
        break
