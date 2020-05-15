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


# Notes
Rendered videos were compressed using:
```
gifsicle -i labels.gif -O3 --resize-width 256 --colors 32 --lossy -o anim_labels.gif
```