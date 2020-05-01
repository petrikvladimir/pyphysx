/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 *
 *     Wrapper for physics, foundation, cooking, and dispatcher for scenes workload management.
 */

#ifndef SIM_PHYSX_PHYSICS_H
#define SIM_PHYSX_PHYSICS_H

#include "PxPhysicsAPI.h"
#include "Scene.h"

class Physics {

public:
    explicit Physics(int num_cpu) {
        using namespace physx;
        foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, error_callback);
        physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());
        cooking = PxCreateCooking(PX_PHYSICS_VERSION, *foundation, PxCookingParams(PxTolerancesScale()));
        dispatcher = PxDefaultCpuDispatcherCreate(num_cpu);
    }

    /** @brief Create standard scene with gravity in -z direction.  */
    Scene create_scene() {
        physx::PxSceneDesc sceneDesc(physics->getTolerancesScale());
        sceneDesc.cpuDispatcher = dispatcher;
        sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
        sceneDesc.gravity = physx::PxVec3(0.0f, 0.0f, -9.81f);
        Scene scene;
        scene.set_physx_ptr(physics->createScene(sceneDesc));
        return scene;
    }

    virtual ~Physics() {
//        Release all scenes first
        auto num_scenes = physics->getNbScenes();
        std::vector<physx::PxScene *> scenes(num_scenes);
        physics->getScenes(&scenes[0], scenes.size());
        for (auto &scene : scenes) {
            scene->release();
        }
        #define SAFE_RELEASE(x)    if(x)    { x->release(); x = nullptr;    }
        SAFE_RELEASE(dispatcher);
        SAFE_RELEASE(cooking);
        SAFE_RELEASE(physics);
        SAFE_RELEASE(foundation);
    }

private:

    physx::PxDefaultAllocator allocator;
    physx::PxDefaultErrorCallback error_callback;
    physx::PxFoundation *foundation = nullptr;
    physx::PxPhysics *physics = nullptr;
    physx::PxCooking *cooking = nullptr;

    physx::PxDefaultCpuDispatcher *dispatcher = nullptr;

};


#endif //SIM_PHYSX_PHYSICS_H
