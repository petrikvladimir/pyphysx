/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_RIGIDDYNAMIC_H
#define SIM_PHYSX_RIGIDDYNAMIC_H

#include <Eigen/src/Core/Matrix.h>
#include <PxPhysicsAPI.h>
#include <BasePhysxPointer.h>
#include <transformation_utils.h>

class RigidDynamic : public BasePhysxPointer<physx::PxRigidDynamic> {

public:
    RigidDynamic() : BasePhysxPointer() {
        set_physx_ptr(PxGetPhysics().createRigidDynamic(physx::PxTransform(physx::PxIdentity)));
    }

    void attach_box(const Eigen::Vector3f &sz, float mass, Material mat) {
        physx::PxRigidActorExt::createExclusiveShape(*get_physx_ptr(),
                                                     physx::PxBoxGeometry(0.5 * sz[0], 0.5 * sz[1], 0.5 * sz[2]),
                                                     *mat.get_physx_ptr(), physx::PxShapeFlag::eSIMULATION_SHAPE);
        physx::PxRigidBodyExt::setMassAndUpdateInertia(*get_physx_ptr(), mass);
    }

    void set_global_pose(const Eigen::Vector3f &sz) {

    }

    void get_global_pose() {

    }

};


#endif //SIM_PHYSX_RIGIDDYNAMIC_H
