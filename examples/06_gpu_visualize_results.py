#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 5/17/20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('06_performance.csv')

fig, axes = plt.subplots(2, 8, squeeze=False, sharex=True, sharey=False, figsize=(6.4 * 3., 4.8 * 1.5))
for (obj_type, obj_grp), axrow in zip(df.groupby('obj_type'), axes):
    for (num_objects, grp), ax in zip(obj_grp.groupby('num_objects'), axrow):
        for (gname, ggrp), clr in zip(grp.groupby('gpu'), ['tab:blue', 'tab:orange']):
            width = 0.4
            indv = np.flip(ggrp['num_cpus'].to_numpy(), 0)
            ctime = np.flip(ggrp['computation_time'].to_numpy(), 0)
            ind = np.arange(5 - len(ggrp['num_cpus']), 5)
            if gname < 0.5:  # CPU
                ax.bar(ind - width / 2, ctime, width=width, color='tab:blue', label='No GPU')
            else:  # GPU
                ax.bar(ind + width / 2, ctime, width=width, color='tab:orange', label='GPU V100')
            ax.set_xticks(ind)
            ax.set_xticklabels([str(int(v)) for v in indv])
        ax.set_title('{} x {}'.format(int(num_objects), obj_type))

for ax in axes.flatten():
    ax.legend()
    ax.grid(False)
for ax in axes[-1, :]:
    ax.set_xlabel('Number of CPU')
for ax in axes[:, 0]:
    ax.set_ylabel('Computation time [s]')
fig.delaxes(axes[0, -1])
fig.delaxes(axes[0, -2])
fig.subplots_adjust(left=0.03, wspace=0.25)
fig.subplots_adjust(right=0.99)

fig.savefig('06_gpu_performance.png')
plt.show()
