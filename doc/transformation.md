# Transformation

All transformation in this wrapper are represented by a tuple that consists of position and rotation (quaternion).
The tuple transformation is automatically casted into physx::PxTransform c++ type.

## Transformations returned from PhysX
All transformation returned from physx are in form `(pos, quat)`, e.g.:
```python
pos, quat = actor.get_global_pose()
```
Position is a numpy array of shape `(3,)` and quaternion is numpy-quaternion.

## Creating transformations
There are multiple possibilities how to specify transformation that is passed to physx:

### Pure translation (i.e. Identity orientation)
- array of 3 values: `[x, y, z]` or `(x,y,z)` or `np.array([x,y,z])`
- 1D tuple of buffer of 3 values: `([x, y, z],)` or `((x,y,z),)` or `(np.array([x,y,z]),)`
 or `((x,y,z),)` creates transformation with identity rotation

### Specifying rotation
 - tuple of position and quaternion `((x,y,z), quaternion(1,0,0,0))`
 - tuple of position and quaternion specified as array `((x,y,z), (1,0,0,0))`
 - array of size 7: `[x,y,z,qw,qx,qy,qz]`

### Casting transformations
You can use function `pyphysx.cast_transformation()` to convert above mentioned types into `(pos, quat)` format.

## Manipulation with transformations
```
t1 = (pos1, quat1)
t2 = cast_transformation([0,0,0.2])

print(multiply_transformations(t1, t2))
print(inverse_transformation(t1))
```