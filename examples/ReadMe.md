# Examples
The examples show usage of the wrapper. After running the code, you should see output similar to the animations shown bellow.

## Free fall of the cube
![](anim_fall_small.gif)

## Mesh loading example
![](anim_spade.gif)

## Joints example
Two rigid bodies connected by joints: 
(i) no joint on the left,
(ii) Fixed joint in the middle,
and (iii) Linear joint with damping on the right.

![](anim_joints.gif)

## Labels example
Shows how to add/update labels into the scene.

![](anim_labels.gif)


## GPU computation example
The example is split into two parts: (i) compute execution time for cpu and gpu for various scenes and (ii) plot results.
For turning on gpu computation you need to init gpu (the function initialize GPU context) and you can pass (optional) GPU settings into the scene constructor:
```
Physics.init_gpu()
Scene(scene_flags=[SceneFlag.ENABLE_PCM, SceneFlag.ENABLE_GPU_DYNAMICS, SceneFlag.ENABLE_STABILIZATION],
    broad_phase_type=BroadPhaseType.GPU, gpu_max_num_partitions=8, gpu_dynamic_allocation_scale=1.,
)
```
![](06_gpu_performance.png)

# Notes
Rendered videos were compressed using:
```
gifsicle -i labels.gif -O3 --resize-width 256 --colors 32 --lossy -o anim_labels.gif
```