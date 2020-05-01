/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_SCENE_H
#define SIM_PHYSX_SCENE_H

#include <Physics.h>
#include <BasePhysxPointer.h>
#include <PxScene.h>

class Scene : public BasePhysxPointer<physx::PxScene> {
public:
    Scene() : BasePhysxPointer() {
        physx::PxSceneDesc sceneDesc(Physics::get().physics->getTolerancesScale());
        sceneDesc.cpuDispatcher = Physics::get().dispatcher;
        sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
        sceneDesc.gravity = physx::PxVec3(0.0f, 0.0f, -9.81f);
        set_physx_ptr(Physics::get().physics->createScene(sceneDesc));
    }
};

#endif //SIM_PHYSX_SCENE_H
