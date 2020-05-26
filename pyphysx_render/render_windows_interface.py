#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/15/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
# Interface used to communicate between parallel renderer window and main process.
#

from multiprocessing import Queue


def add_to_queue(fn):
    def wrapped(self, *args, **kwargs):
        if self.sending_queue is not None:
            fn_name = fn.__name__
            self.sending_queue.put((fn_name, (args, kwargs)))
        return None

    return wrapped


class PyPhysXWindowInterface:

    def __init__(self, sending_queue: Queue = None) -> None:
        """ If queue is given, it is assumed that interface is sending the function signature into the queue. """
        super().__init__()
        self.sending_queue = sending_queue

    @add_to_queue
    def add_label(self, pose=None, scale=1., text='',
                  font_name=None, font_size=None, bold=False, italic=False,
                  color='green', alpha=1.,
                  x=0, y=0, width=None, height=None,
                  anchor_x='left', anchor_y='baseline',
                  align='left'):
        """ Add new label on the given pose. Scale text by a given factor. """
        pass

    @add_to_queue
    def clear_labels(self):
        """ Remove all labels from the scene. """
        pass

    @add_to_queue
    def update_labels_text(self, texts):
        """ Update text on all labels that are in the scene. """
        pass

    @add_to_queue
    def close(self):
        """ Close window. """
        pass

    @add_to_queue
    def add_actor_geometry(self, actor_id, geometry_data, local_pose, color=None):
        """ Add actor geometry into the internal rendering buffer. Actor is identified by actor_id, new is created if
            id was not used before. """
        pass

    @add_to_queue
    def clear_actors(self):
        """ Remove all actors from the rendering buffer. """
        pass

    @add_to_queue
    def set_actor_pose(self, actor_id, pose):
        """ Set actor pose based on the actor_id """
        pass
