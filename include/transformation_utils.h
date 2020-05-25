/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_TRANSFORMATION_UTILS_H
#define SIM_PHYSX_TRANSFORMATION_UTILS_H

#include <Eigen/Eigen>
#include <foundation/PxTransform.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

namespace pybind11 {
    namespace detail {
        template<>
        struct type_caster<physx::PxQuat> {
            /**
             * @brief Automatic casting of PxQuat type into the numpy quaternion type.
             */
        public:
        PYBIND11_TYPE_CASTER(physx::PxQuat, _("quaternion"));

            bool load(handle src, bool) {
                if (!src) { return false; }
                if (pybind11::hasattr(src, "x") && pybind11::hasattr(src, "y") && pybind11::hasattr(src, "z") &&
                    pybind11::hasattr(src, "w")) {
                    value.x = src.attr("x").cast<float>();
                    value.y = src.attr("y").cast<float>();
                    value.z = src.attr("z").cast<float>();
                    value.w = src.attr("w").cast<float>();
                    value.normalize();
                    return true;
                }
                auto arr = src.cast<pybind11::array_t<float>>();
                if (!arr) {
                    return false;
                }
                value.w = arr.at(0);
                value.x = arr.at(1);
                value.y = arr.at(2);
                value.z = arr.at(3);
                value.normalize();
                return true;
            }

            static handle cast(const physx::PxQuat &q, return_value_policy /* policy */, handle /* parent */) {
                pybind11::object pyquat = pybind11::module::import("quaternion").attr("quaternion")();
                pyquat.attr("x") = pybind11::cast(q.x);
                pyquat.attr("y") = pybind11::cast(q.y);
                pyquat.attr("z") = pybind11::cast(q.z);
                pyquat.attr("w") = pybind11::cast(q.w);
                return pyquat.release();
            }
        };

        template<>
        struct type_caster<physx::PxVec3> {
            /**
             * @brief Automatic casting of PxVec3 type into the numpy array.
             */
        public:
        PYBIND11_TYPE_CASTER(physx::PxVec3, _("pxvec3"));

            bool load(handle src, bool) {
                auto arr = src.cast<pybind11::array_t<float>>();
                if (!arr) {
                    return false;
                }
                if (arr.size() != 3) {
                    return false;
                }
                value.x = arr.at(0);
                value.y = arr.at(1);
                value.z = arr.at(2);
                return true;
            }

            static handle cast(const physx::PxVec3 &p, return_value_policy /* policy */, handle /* parent */) {
                pybind11::array_t<float> arr(3);
                arr.mutable_at(0) = p.x;
                arr.mutable_at(1) = p.y;
                arr.mutable_at(2) = p.z;
                return arr.release();
            }
        };

        template<>
        struct type_caster<physx::PxTransform> {
            /**
             * @brief Casting of PxTransform and python transformation represented by (pos, quat).
             */
        public:
        PYBIND11_TYPE_CASTER(physx::PxTransform, _("pose"));

            bool load(handle src, bool) {
                if (isinstance<pybind11::tuple>(src)) { // pose from tuple (pos,) or (pos, quat)
                    const auto tuple = pybind11::reinterpret_borrow<pybind11::tuple>(src);
                    if (tuple.size() != 1 && tuple.size() != 2 && tuple.size() != 3) {
                        return false;
                    }
                    value.p = pybind11::cast<physx::PxVec3>(tuple.size() == 3 ? tuple : tuple[0]);
                    value.q = tuple.size() == 2 ? value.q = pybind11::cast<physx::PxQuat>(tuple[1]) : physx::PxQuat(
                            physx::PxIdentity);
                    return true;
                }
                auto arr = src.cast<pybind11::array_t<float>>(); // pose from [x,y,z] or [x,y,z,qw,qx,qy,qz]
                if (!arr) return false;
                if (arr.size() != 3 && arr.size() != 7) return false;
                value.p.x = arr.at(0);
                value.p.y = arr.at(1);
                value.p.z = arr.at(2);
                if (arr.size() == 7) {
                    value.q.w = arr.at(3);
                    value.q.x = arr.at(4);
                    value.q.y = arr.at(5);
                    value.q.z = arr.at(6);
                    value.q.normalize();
                } else {
                    value.q = physx::PxQuat(physx::PxIdentity);
                }
                return true;
            }

            static handle cast(const physx::PxTransform &t, return_value_policy /* policy */, handle /* parent */) {
                return pybind11::make_tuple(pybind11::cast(t.p), pybind11::cast(t.q)).release();
            }
        };
    }
}

/**
 * @brief Cast given transformation into the python transformation.
 *  Used primarly for testing python->c++->python conversion of transformation.
 *  Note, that precision might be decreased if input is double as it is casted to float.
 *  In python, this function always returns tuple of position and np quaternion but it can handle various inputs.
 */
auto cast_transformation(const physx::PxTransform &transform) {
    return transform;
}

template<class T>
auto from_vector_of_physx_ptr(const std::vector<typename T::type_physx *> &ptrs) {
    std::vector<T> vec;
    vec.reserve(ptrs.size());
    for (const auto &p : ptrs) {
        vec.emplace_back(T(p));
    }
    return vec;
}

template<class T, class TP>
auto from_vector_of_physx_ptr(const std::vector<TP *> &ptrs) {
    std::vector<T> vec;
    vec.reserve(ptrs.size());
    for (const auto &p : ptrs) {
        vec.emplace_back(T(p));
    }
    return vec;
}

#endif //SIM_PHYSX_TRANSFORMATION_UTILS_H
