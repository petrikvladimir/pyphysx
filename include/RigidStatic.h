/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 5/4/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef PYPHYSX_RIGIDSTATIC_H
#define PYPHYSX_RIGIDSTATIC_H

#include <Physics.h>
#include <RigidActor.h>

class RigidStatic : public RigidActor {

private:
    /** @brief Cast physx base pointer into physx RigidStatic pointer. */
    auto get_static_ptr() {
        return reinterpret_cast<physx::PxRigidStatic *>(get_physx_ptr());
    }

public:
    RigidStatic() : RigidActor(Physics::get_physics()->createRigidStatic(physx::PxTransform(physx::PxIdentity))) {}

    explicit RigidStatic(physx::PxRigidStatic *physxPtr) :
            RigidActor(reinterpret_cast<RigidActor::type_physx *>(physxPtr)) {
    }
	
	~RigidStatic() {
		get_physx_ptr()->release();
	}

    static RigidStatic create_plane(Material mat, float nx, float ny, float nz, float distance) {
        return RigidStatic(physx::PxCreatePlane(*Physics::get_physics(), physx::PxPlane(nx, ny, nz, distance),
                                         *mat.get_physx_ptr()));
    }
};

#endif //PYPHYSX_RIGIDSTATIC_H
