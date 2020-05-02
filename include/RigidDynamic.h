/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_RIGIDDYNAMIC_H
#define SIM_PHYSX_RIGIDDYNAMIC_H

#include <Physics.h>
#include <BasePhysxPointer.h>
#include <Shape.h>
#include <transformation_utils.h>

class RigidDynamic : public BasePhysxPointer<physx::PxRigidDynamic> {

public:
    RigidDynamic() : BasePhysxPointer() {
        set_physx_ptr(Physics::get_physics()->createRigidDynamic(physx::PxTransform(physx::PxIdentity)));
    }

    void attach_shape(const Shape &shape) {
        get_physx_ptr()->attachShape(*shape.get_physx_ptr());
    }

    auto get_atached_shapes() {
        std::vector<physx::PxShape *> shapes_ptr(get_physx_ptr()->getNbShapes());
        get_physx_ptr()->getShapes(&shapes_ptr[0], shapes_ptr.size());
        std::vector<Shape> shapes(shapes_ptr.size());
        for (size_t i = 0; i < shapes.size(); ++i) {
            shapes[i].set_physx_ptr(shapes_ptr[i]);
        }
        return shapes;
    }

    void set_mass_and_update_inertia(float mass) {
        physx::PxRigidBodyExt::setMassAndUpdateInertia(*get_physx_ptr(), mass);
    }

    auto get_mass() {
        return get_physx_ptr()->getMass();
    }

    auto get_global_pose() {
        return transform_to_eigen(get_physx_ptr()->getGlobalPose());
    }

    void set_global_pose(const Eigen::Vector3f &pos, const Eigen::Vector4f &quat) {
        get_physx_ptr()->setGlobalPose(eigen_to_transform(pos, quat));
    }

};

#endif //SIM_PHYSX_RIGIDDYNAMIC_H
