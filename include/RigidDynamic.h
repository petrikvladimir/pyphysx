/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_RIGIDDYNAMIC_H
#define SIM_PHYSX_RIGIDDYNAMIC_H

#include <Physics.h>
#include <RigidActor.h>

class RigidDynamic : public RigidActor {

private:
    /** @brief Cast physx base pointer into physx RigidDynamic pointer. */
    auto get_dyn_ptr() {
        return reinterpret_cast<physx::PxRigidDynamic *>(get_physx_ptr());
    }

public:
    RigidDynamic() : RigidDynamic(Physics::get_physics()->createRigidDynamic(physx::PxTransform(physx::PxIdentity))) {
    }

    explicit RigidDynamic(physx::PxRigidDynamic *physxPtr) :
            RigidActor(reinterpret_cast<RigidActor::type_physx *>(physxPtr)) {
    }

    void set_mass_and_update_inertia(float mass) {
        physx::PxRigidBodyExt::setMassAndUpdateInertia(*get_dyn_ptr(), mass);
    }

    auto get_mass() {
        return get_dyn_ptr()->getMass();
    }

};

#endif //SIM_PHYSX_RIGIDDYNAMIC_H
