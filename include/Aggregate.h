/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 5/16/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef PYPHYSX_AGGREGATE_H
#define PYPHYSX_AGGREGATE_H

#include <Physics.h>
#include <BasePhysxPointer.h>
#include "RigidActor.h"

class Aggregate : public BasePhysxPointer<physx::PxAggregate> {
public:
    Aggregate(size_t max_size, bool enable_self_collision) :
            BasePhysxPointer<physx::PxAggregate>(
                    Physics::get_physics()->createAggregate(max_size, enable_self_collision)) {}


    void add_actor(RigidActor actor) {
        get_physx_ptr()->addActor(*actor.get_physx_ptr());
    }

    void remove_actor(RigidActor actor) {
        get_physx_ptr()->removeActor(*actor.get_physx_ptr());
    }

    auto get_actors() {
        auto n = get_physx_ptr()->getNbActors();
        std::vector<physx::PxRigidActor *> actors(n);
        get_physx_ptr()->getActors(reinterpret_cast<physx::PxActor **>(&actors[0]), n);
        return from_vector_of_physx_ptr<RigidActor>(actors);
    }
};


#endif //PYPHYSX_AGGREGATE_H
