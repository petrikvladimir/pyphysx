# Examples
The examples show usage of the wrapper. After running the code, you should see output similar to the animations shown bellow.

## Free fall of the cube
![](videos/anim_01_free_fall.gif)

## Mesh loading example
![](videos/anim_02_spade.gif)

## Joints example
Two rigid bodies connected by joints: 
(i) no joint on the left,
(ii) Fixed joint in the middle,
and (iii) Linear joint with damping on the right.

![](videos/anim_03_joints.gif)

## Labels example
Shows how to add/update labels into the scene.

![](videos/anim_04_labels.gif)


## Load URDF example
Example shows how to load simple urdf model into the physx and how to specify the joint commands in position or velocity mode.
Left example shows dynamic simulation of robot while the right one shows kinematic simulation.
The type of simulation can be specified when loading the robot.

![](videos/anim_05_load_urdf.gif)
![](videos/anim_05_load_urdf_kinematic.gif)

## Load URDF Franka Emika Panda robot
Example shows both visual and collision model of the robot.

![](videos/anim_05a_load_panda.gif)

## Panda scene interaction
Example shows interaction between cubes and panda robot. In addition it shows how to change background color.

![](videos/anim_05b_panda_cubes.gif)

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

## Meshcat Viewer example
Example shows how to use MeshCat 3D web visualizer with PyPhysx.
```python
# ... create scene as usual and then create viewer
render = MeshcatViewer(wait_for_open=True, open_meshcat=True) # this will open tab in your browser with empty scene
render.add_physx_scene(scene)  # add pyphysx scene into the meshcat viewer
# ... simulate etc.
render.update() # update poses of actors

```
![](videos/anim_07_meshcat.gif)

## Offscreen render utilizing PyRender library
Shows how to create offscreen renderer. It shares the api with other PyPhysX viewer, i.e. has `add_physx_scene` and `update` functions.
RGB and Depth data are obtained by calling `get_rgb_and_depth()` member function.
Concatenation of these values are saved into the gif and shown bellow.
Note, that different backend need to be used if running in headless mode (e.g. on computational cluster). 
See [PyRender documentation](https://pyrender.readthedocs.io/en/latest/examples/offscreen.html) for details.
```python
render = PyPhysxOffscreenRenderer()
render.add_physx_scene(scene)  # add pyphysx scene into the offscreen renderer
render.render_scene.ambient_light = [0.1] * 3  # add some light
render.update()
rgb, depth = render.get_rgb_and_depth()
```
![](videos/anim_08_offscreen_renderer.gif)
