/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 5/3/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef PYPHYSX_RIGIDACTOR_H
#define PYPHYSX_RIGIDACTOR_H

#include <Physics.h>
#include <BasePhysxPointer.h>
#include <Shape.h>
#include <transformation_utils.h>

class RigidActor : public BasePhysxPointer<physx::PxRigidActor> {
public:
    explicit RigidActor(physx::PxRigidActor *physxPtr) : BasePhysxPointer(physxPtr) {}

    auto get_global_pose() {
        return get_physx_ptr()->getGlobalPose();
    }

    void set_global_pose(const physx::PxTransform& pose) {
        get_physx_ptr()->setGlobalPose(pose);
    }

    void attach_shape(const Shape &shape) {
        get_physx_ptr()->attachShape(*shape.get_physx_ptr());
    }

    void detach_shape(const Shape &shape) {
        get_physx_ptr()->detachShape(*shape.get_physx_ptr());
    }

    auto get_atached_shapes() {
        std::vector<physx::PxShape *> shapes_ptr(get_physx_ptr()->getNbShapes());
        get_physx_ptr()->getShapes(&shapes_ptr[0], shapes_ptr.size());
        return from_vector_of_physx_ptr<Shape>(shapes_ptr);
    }

    void disable_gravity() {
        get_physx_ptr()->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, true);
    }

    void enable_gravity() {
        get_physx_ptr()->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, false);
    }

    void set_user_data(const pybind11::object &o) {
        if (get_physx_ptr()->userData != nullptr) { // release the old object first
            pybind11::handle(static_cast<PyObject *>(get_physx_ptr()->userData)).dec_ref();
        }
        o.inc_ref(); // make object safe from garbage collector
        get_physx_ptr()->userData = o.ptr();
    }

    auto get_user_data() {
        if (get_physx_ptr()->userData == nullptr) {
            return pybind11::reinterpret_borrow<pybind11::object>(pybind11::none());
        }
        return pybind11::reinterpret_borrow<pybind11::object>(
                pybind11::handle(
                        static_cast<PyObject *>(get_physx_ptr()->userData)
                )
        );
    }

    auto overlaps(RigidActor other) {
        for (const auto shape : get_atached_shapes()) {
            for (const auto other_shape: other.get_atached_shapes()) {
                if (shape.overlaps(other_shape, this->get_global_pose(), other.get_global_pose())) {
                    return true;
                }
            }
        }
        return false;
    }
};


#endif //PYPHYSX_RIGIDACTOR_H
