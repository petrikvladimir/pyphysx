/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_TRANSFORMATION_UTILS_H
#define SIM_PHYSX_TRANSFORMATION_UTILS_H

#include <Eigen/Eigen>
#include <foundation/PxTransform.h>

/** @brief Takes 3d vector and transfrom it to PxVec3 */
auto eigen_to_pxvec(const Eigen::Vector3f &vec) {
    return physx::PxVec3(vec[0], vec[1], vec[2]);
}

auto eigen_to_pxvec(const Eigen::Vector4f &vec) {
    return physx::PxVec4(vec[0], vec[1], vec[2], vec[3]);
}

/** @brief Create eigen vector from pxvec. */
auto pxvec_to_eigen(const physx::PxVec3 &vec) {
    return Eigen::Vector3f(vec[0], vec[1], vec[2]);
}

auto pxvec_to_eigen(const physx::PxVec4 &vec) {
    return Eigen::Vector4f(vec[0], vec[1], vec[2], vec[3]);
}

auto quat_to_eigen(const physx::PxQuat &vec) {
    return Eigen::Vector4f(vec.x, vec.y, vec.z, vec.w);
}

auto eigen_to_quat(const Eigen::Vector4f &vec) {
    return physx::PxQuat(vec[0], vec[1], vec[2], vec[3]);
}

auto transform_to_eigen(const physx::PxTransform &transform) {
    return std::make_tuple(pxvec_to_eigen(transform.p), quat_to_eigen(transform.q));
}

auto eigen_to_transform(const Eigen::Vector3f &pos, const Eigen::Vector4f &quat) {
    return physx::PxTransform(eigen_to_pxvec(pos), eigen_to_quat(quat));
}

#endif //SIM_PHYSX_TRANSFORMATION_UTILS_H
