# PyPhysX - python wrapper for PhysX Nvidia
[![Build Status](https://travis-ci.com/petrikvladimir/pyphysx.svg?branch=master)](https://travis-ci.com/petrikvladimir/pyphysx) 
[![codecov](https://codecov.io/gh/petrikvladimir/pyphysx/branch/master/graph/badge.svg)](https://codecov.io/gh/petrikvladimir/pyphysx)

## Installation
To install for the first time:
```
pip install git+https://github.com/petrikvladimir/pyphysx.git@master
```
To update if its already installed: 
```
pip install --upgrade git+https://github.com/petrikvladimir/pyphysx.git@master
```
If pip install fails, build it from source:
```
git clone https://github.com/petrikvladimir/pyphysx.git
python setup.py install --user
```
Optionally, you can install in anaconda
```
conda create -n py38 python=3.8
conda activate py38
conda install -c anaconda gcc_linux-64 gxx_linux-64
pip install git+https://github.com/petrikvladimir/pyphysx.git@master
```

## Trivial example
For more advanced examples, have a look into the folder [examples](examples/).
```python
from pyphysx_render.renderer import PyPhysXParallelRenderer
from pyphysx_utils.rate import Rate
from pyphysx import *

scene = Scene()
scene.add_actor(RigidStatic.create_plane(material=Material(static_friction=0.1, dynamic_friction=0.1, restitution=0.5)))

actor = RigidDynamic()
actor.attach_shape(Shape.create_box([0.2] * 3, Material(restitution=1.)))
actor.set_global_pose([0.5, 0.5, 1.0])
actor.set_mass(1.)
scene.add_actor(actor)

render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='out.mp4'))
rate = Rate(120)
for i in range(100):
    scene.simulate(rate.period())
    render.render_scene(scene)
    rate.sleep()
```
The code will render and simulate the scene and automatically generate video like this:

![](examples/anim_fall.gif)


# Features
## PhysX interface
- physics
  - PhysX allows only one instance of Physics object per process - we enforce it in PyPhysX by using singleton that is initialized on the first use
  - parallel computation control:
    - `Physics.set_num_cpu(N)` - creates CPU dispatcher with N threads
    - `Physics.init_gpu()` - initialize GPU computation
- scene
  - create scene and actors that will be simulated
  - multiple scenes can be created in parallel
- rigid actors (both static and dynamic)
  - create/attach/detach geometries (box, sphere, convex mesh)
  - create/update materials
  - set/update flags or actor properties (velocity, kinematic target, mass)
- D6Joint
  - specify per axis limits and drives
- transformations
  - automatic transformation casting between pxTransform and tuple of position and numpy quaternion (see [Transformation](doc/transformation.md))

## Rendering
- the library uses `pyglet` library to render 3d scene in parallel process
- you decide when you want to update your scene by using function `render_scene(scene)`
- renderer can save video automatically after the window is closed `render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='out.mp4', fullscreen=True))`
- window control:
  - mouse:
    - right button drag to rotate the scene
  - keys:
    - esc to close the window
    - f to show on/off coordinates frame
    - g to show on/off geometries
    - l to show on/off text labels
    - p to take screenshot of the rendered scene `~/Pictures/pyphysx_screenshot_[datetime].png`

## URDF parser
- parse robot from `URDF` file
- specify joint controller and command robot