# PyPhysX - python wrapper for PhysX Nvidia
[![Build Status](https://travis-ci.com/petrikvladimir/pyphysx.svg?branch=master)](https://travis-ci.com/petrikvladimir/pyphysx) 
[![codecov](https://codecov.io/gh/petrikvladimir/pyphysx/branch/master/graph/badge.svg)](https://codecov.io/gh/petrikvladimir/pyphysx)

## Installation
```
pip install git+https://github.com/petrikvladimir/pyphysx.git@master
pip install --upgrade git+https://github.com/petrikvladimir/pyphysx.git@master
```


# Features
## Automatic video saving
Record the rendered screen and save it as a video automatically after the window is closed.
```
render = PyPhysXParallelRenderer(render_window_kwargs=dict(video_filename='out.mp4', fullscreen=True))
```