#!/bin/bash
# Compress all example videos
for vid in 01_free_fall 02_spade 03_joints 04_labels 05_load_urdf 05_load_urdf_kinematic; do
  gifsicle -i $vid.gif -O3 --resize-width 256 --colors 32 --lossy -o anim_$vid.gif
done
for vid in 05a_load_panda; do
  gifsicle -i $vid.gif -O3 --resize-width 256 --colors 128 --lossy -o anim_$vid.gif
done