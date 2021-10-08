/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 *
 *     Wrapper around the PhysX material.
 */

#ifndef SIM_PHYSX_MATERIAL_H
#define SIM_PHYSX_MATERIAL_H

#include <Physics.h>
#include <BasePhysxPointer.h>

class Material : public BasePhysxPointer<physx::PxMaterial> {

public:
    /** @brief Create new material using global physics instance. */
    Material(float static_friction, float dynamic_friction, float restitution) :
            BasePhysxPointer(Physics::get_physics()->createMaterial(static_friction, dynamic_friction, restitution)) {
    }

    explicit Material(physx::PxMaterial *pref) : BasePhysxPointer<physx::PxMaterial>(pref) {}

    void set_static_friction(float static_friction) {
        get_physx_ptr()->setStaticFriction(static_friction);
    }

    auto get_static_friction() const {
        return get_physx_ptr()->getStaticFriction();
    }

    void set_dynamic_friction(float dynamic_friction) {
        get_physx_ptr()->setDynamicFriction(dynamic_friction);
    }

    auto get_dynamic_friction() const {
        return get_physx_ptr()->getDynamicFriction();
    }

    void set_restitution(float restitution) {
        get_physx_ptr()->setRestitution(restitution);
    }

    auto get_restitution() const {
        return get_physx_ptr()->getRestitution();
    }
};

#endif //SIM_PHYSX_MATERIAL_H
