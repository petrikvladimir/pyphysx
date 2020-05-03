/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_BASEPHYSXPOINTER_H
#define SIM_PHYSX_BASEPHYSXPOINTER_H


#include <cstdint>

template<class T>
class BasePhysxPointer {
public:
    typedef T type_physx;

    BasePhysxPointer() : ref(0) {}

    explicit BasePhysxPointer(T *physx_ptr) : ref(reinterpret_cast<uintptr_t>(physx_ptr)) {}

    /** @brief Set physx pointer to the object created elsewhere. */
    void set_physx_ptr(T *physx_ptr) {
        ref = reinterpret_cast<uintptr_t>(physx_ptr);
    }

    /** @brief Get pointer to the created material in a PhysX format. */
    auto get_physx_ptr() const {
        return reinterpret_cast<T *>(ref);
    }

private:
    /** @brief Pointer to the memory, processable by pybind11. */
    uintptr_t ref;
};


#endif //SIM_PHYSX_BASEPHYSXPOINTER_H
