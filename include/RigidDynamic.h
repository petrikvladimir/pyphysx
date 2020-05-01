/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_RIGIDDYNAMIC_H
#define SIM_PHYSX_RIGIDDYNAMIC_H

#include <transformation_utils.h>
#include <PxPhysicsAPI.h>
#include <BasePhysxPointer.h>
#include <Shape.h>

class RigidDynamic : public BasePhysxPointer<physx::PxRigidDynamic> {

public:
    RigidDynamic() : BasePhysxPointer() {
        set_physx_ptr(Physics::get_physics()->createRigidDynamic(physx::PxTransform(physx::PxIdentity)));
    }

    void attach_shape(const Shape &shape) {
        get_physx_ptr()->attachShape(*shape.get_physx_ptr());
    }

    void set_mass_and_update_inertia(float mass) {
        physx::PxRigidBodyExt::setMassAndUpdateInertia(*get_physx_ptr(), mass);
    }

    void set_global_pose(const Eigen::Vector3f &sz) {

    }

    void get_global_pose() {

    }

};


#endif //SIM_PHYSX_RIGIDDYNAMIC_H
