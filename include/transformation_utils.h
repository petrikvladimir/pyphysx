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
auto pxvec_to_eigen(const physx::PxVec3 &vec) {
    return Eigen::Vector3f(vec[0], vec[1], vec[2]);
}

/** @brief Takes 7d vector and transform it into pose by taking 3 positions + 4 quaternion params */
auto eigen_to_transform(const Eigen::VectorXf &row) {
    return physx::PxTransform(
            physx::PxVec3(row(0), row(1), row(2)),
            physx::PxQuat(row(3), row(4), row(5), row(6))
    );
}

#endif //SIM_PHYSX_TRANSFORMATION_UTILS_H
