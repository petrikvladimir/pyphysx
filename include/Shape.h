/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 5/1/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef PYPHYSX_SHAPE_H
#define PYPHYSX_SHAPE_H

#include <Eigen/Eigen>
#include <PxPhysicsAPI.h>
#include <BasePhysxPointer.h>
#include <Material.h>
#include <Physics.h>

class Shape : public BasePhysxPointer<physx::PxShape> {

public:
//    todo:  functions for enabling/disabling flags

    static Shape create_box(const Eigen::Vector3f &sz, Material mat, bool is_exclusive) {
        return Shape::from_geometry(physx::PxBoxGeometry(0.5 * sz[0], 0.5 * sz[1], 0.5 * sz[2]), mat, is_exclusive);
    }

    static Shape create_sphere(float radius, Material mat, bool is_exclusive) {
        return Shape::from_geometry(physx::PxSphereGeometry(radius), mat, is_exclusive);
    }

private:
    static Shape from_geometry(const physx::PxGeometry &geometry, Material mat, bool is_exclusive) {
        Shape shape;
        shape.set_physx_ptr(Physics::get_physics()->createShape(geometry, *mat.get_physx_ptr(), is_exclusive,
                                                                physx::PxShapeFlag::eSIMULATION_SHAPE |
                                                                physx::PxShapeFlag::eVISUALIZATION));
        return shape;
    }

};


#endif //PYPHYSX_SHAPE_H
