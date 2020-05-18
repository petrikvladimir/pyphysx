#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/16/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import itertools
import multiprocessing

from pyphysx_render.rate import Rate
from pyphysx import *
import numpy as np
import pandas as pd
import time
import trimesh


def do_sim(i, return_dict, prop):
    """ Perform simulation in separate process. That ensures that the physics is initialized for each set of
    properties. There is always just one Physics for the process. """
    if prop['gpu']:
        Physics.init_gpu()
    Physics.set_num_cpu(prop['num_cpus'])
    scene = Scene() if not prop['gpu'] else Scene(
        scene_flags=[SceneFlag.ENABLE_PCM, SceneFlag.ENABLE_GPU_DYNAMICS, SceneFlag.ENABLE_STABILIZATION],
        broad_phase_type=BroadPhaseType.GPU,
        gpu_max_num_partitions=8, gpu_dynamic_allocation_scale=8.,
    )
    obj: trimesh.Scene = trimesh.load('spade.obj', split_object=True, group_material=False)
    mat = Material(static_friction=0.1, dynamic_friction=0.1, restitution=0.5)
    scene.add_actor(RigidStatic.create_plane(mat=mat))
    for _ in range(prop['num_objects']):
        o = RigidDynamic()
        if prop['obj_type'] == 'sphere':
            o.attach_shape(Shape.create_sphere(0.05, mat))
        elif prop['obj_type'] == 'spade':
            for g in obj.geometry.values():
                o.attach_shape(Shape.create_convex_mesh_from_points(g.vertices, mat, scale=1e-3))
        o.set_global_pose(np.random.uniform(0.5, 10., size=3))
        o.set_mass(1.)
        scene.add_actor(o)

    start_time = time.time()
    rate = Rate(240)
    for _ in range(1000):
        scene.simulate(rate.period())
    end_time = time.time()
    return_dict[i] = end_time - start_time


performance = pd.DataFrame()

manager = multiprocessing.Manager()
return_dict = manager.dict()

dicts = {
    'obj_type': ['sphere', 'spade'],
    'num_objects': [1, 10, 100, 1000, 5000, 10000, 25000, 50000],
    'gpu': [True, False],
    'num_cpus': [16, 8, 4, 2, 1, 0],
}

properties = list(dict(zip(dicts, x)) for x in itertools.product(*dicts.values()))
gpu_initialised = False

for i, prop in enumerate(properties):
    if prop['obj_type'] == 'spade' and prop['num_objects'] > 10000:
        break
    print(i, prop)
    p = multiprocessing.Process(target=do_sim, args=(i, return_dict, prop))
    p.start()
    p.join()
    performance = performance.append({**prop, **{'computation_time': return_dict[i]}}, ignore_index=True)
    performance.to_csv('06_performance.csv')
    print('Computation time: ', return_dict[i])
