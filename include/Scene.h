/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_SCENE_H
#define SIM_PHYSX_SCENE_H

#include <Physics.h>
#include <BasePhysxPointer.h>
#include <RigidDynamic.h>

class Scene : public BasePhysxPointer<physx::PxScene> {
public:
    Scene() : BasePhysxPointer() {
        physx::PxSceneDesc sceneDesc(Physics::get().physics->getTolerancesScale());
        sceneDesc.cpuDispatcher = Physics::get().dispatcher;
        sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
        sceneDesc.gravity = physx::PxVec3(0.0f, 0.0f, -9.81f);
        set_physx_ptr(Physics::get().physics->createScene(sceneDesc));
    }

    /** @brief Simulate scene for given amount of time dt.
     * Num steps can be used to control accuracy by using smaller delta time internally. */
    void simulate(float dt, int num_substeps) {
        const auto ddt = dt / float(num_substeps);
        for (size_t i = 0; i < num_substeps; ++i) {
            get_physx_ptr()->simulate(ddt);
            get_physx_ptr()->fetchResults(true);
            simulation_time += ddt;
        }
    }

    void add_actor(const RigidDynamic &actor) {
        get_physx_ptr()->addActor(*actor.get_physx_ptr());
    }

    auto get_static_rigid_actors() {
        auto n = get_physx_ptr()->getNbActors(physx::PxActorTypeFlag::eRIGID_STATIC);
        std::vector<physx::PxRigidActor *> actors(n);
        get_physx_ptr()->getActors(physx::PxActorTypeFlag::eRIGID_STATIC,
                                   reinterpret_cast<physx::PxActor **>(&actors[0]), n);
        return from_vector_of_physx_ptr<RigidActor>(actors);
    }

    auto get_dynamic_rigid_actors() {
        auto n = get_physx_ptr()->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC);
        std::vector<physx::PxRigidDynamic *> actors(n);
        get_physx_ptr()->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC,
                                   reinterpret_cast<physx::PxActor **>(&actors[0]), n);
        return from_vector_of_physx_ptr<RigidDynamic, physx::PxRigidDynamic>(actors);
    }

public:
    double simulation_time = 0.;
};

#endif //SIM_PHYSX_SCENE_H
