#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import time

from pyphysx_render.renderer import PyPhysXParallelRenderer

render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='labels.gif', coordinates_scale=0))

render.add_label(text='Hello world!', color='tab:blue', scale=2., anchor_x='center')
render.add_label(text='', pos=(0, -0.5, 0), color='tab:green', alpha=0.6)
for i in range(100):
    render.update_labels_text(
        [
            None,  # labels with none will not be updated, i.e. it should render Hello world!
            'i={}'.format(i),  # updates label current counter number
        ]
    )
    time.sleep(0.01)

render.close()
