/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_SCENE_H
#define SIM_PHYSX_SCENE_H

#include <BasePhysxPointer.h>
#include <PxScene.h>

class Scene : public BasePhysxPointer<physx::PxScene> {
};


#endif //SIM_PHYSX_SCENE_H
