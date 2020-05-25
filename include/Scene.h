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
#include "RigidStatic.h"
#include "Aggregate.h"

class Scene : public BasePhysxPointer<physx::PxScene> {
public:
    Scene(const physx::PxFrictionType::Enum &friction_type,
          const physx::PxBroadPhaseType::Enum &broad_phase_type,
          const std::vector<physx::PxSceneFlag::Enum> &scene_flags,
          size_t gpu_max_num_partitions,
          float gpu_dynamic_allocation_scale
    ) : BasePhysxPointer() {
        physx::PxSceneDesc sceneDesc(Physics::get().physics->getTolerancesScale());
        sceneDesc.cpuDispatcher = Physics::get().dispatcher;
        sceneDesc.cudaContextManager = Physics::get().cuda_context_manager;
        sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
        sceneDesc.gravity = physx::PxVec3(0.0f, 0.0f, -9.81f);
        for (const auto &flag : scene_flags) {
            sceneDesc.flags |= flag;
        }
        sceneDesc.frictionType = friction_type;
        sceneDesc.broadPhaseType = broad_phase_type;
        sceneDesc.gpuMaxNumPartitions = gpu_max_num_partitions;
        sceneDesc.gpuDynamicsConfig.patchStreamSize *= gpu_dynamic_allocation_scale;
        sceneDesc.gpuDynamicsConfig.forceStreamCapacity *= gpu_dynamic_allocation_scale;
        sceneDesc.gpuDynamicsConfig.contactBufferCapacity *= gpu_dynamic_allocation_scale;
        sceneDesc.gpuDynamicsConfig.contactStreamSize *= gpu_dynamic_allocation_scale;
        sceneDesc.gpuDynamicsConfig.foundLostPairsCapacity *= gpu_dynamic_allocation_scale;
        sceneDesc.gpuDynamicsConfig.constraintBufferCapacity *= gpu_dynamic_allocation_scale;
        sceneDesc.gpuDynamicsConfig.heapCapacity *= gpu_dynamic_allocation_scale;
        sceneDesc.gpuDynamicsConfig.tempBufferCapacity *= gpu_dynamic_allocation_scale;

        set_physx_ptr(Physics::get().physics->createScene(sceneDesc));
    }

    /** @brief Simulate scene for given amount of time dt and fetch results with blocking. */
    void simulate(float dt) {
        get_physx_ptr()->simulate(dt);
        get_physx_ptr()->fetchResults(true);
        simulation_time += dt;
    }

    void add_actor(RigidActor actor) {
        get_physx_ptr()->addActor(*actor.get_physx_ptr());
    }

    auto get_static_rigid_actors() {
        const auto n = get_physx_ptr()->getNbActors(physx::PxActorTypeFlag::eRIGID_STATIC);
        std::vector<physx::PxRigidActor *> actors(n);
        get_physx_ptr()->getActors(physx::PxActorTypeFlag::eRIGID_STATIC,
                                   reinterpret_cast<physx::PxActor **>(&actors[0]), n);
        return from_vector_of_physx_ptr<RigidActor>(actors);
    }

    auto get_dynamic_rigid_actors() {
        const auto n = get_physx_ptr()->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC);
        std::vector<physx::PxRigidDynamic *> actors(n);
        get_physx_ptr()->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC,
                                   reinterpret_cast<physx::PxActor **>(&actors[0]), n);
        return from_vector_of_physx_ptr<RigidDynamic, physx::PxRigidDynamic>(actors);
    }

    void add_aggregate(Aggregate agg) {
        get_physx_ptr()->addAggregate(*agg.get_physx_ptr());
    }

    auto get_aggregates() {
        const auto n = get_physx_ptr()->getNbAggregates();
        std::vector<physx::PxAggregate *> aggs(n);
        get_physx_ptr()->getAggregates(&aggs[0], n);
        return from_vector_of_physx_ptr<Aggregate>(aggs);
    }

public:
    double simulation_time = 0.;
};

#endif //SIM_PHYSX_SCENE_H
