/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 5/5/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef PYPHYSX_D6JOINT_H
#define PYPHYSX_D6JOINT_H

#include <Physics.h>
#include <RigidActor.h>

class D6Joint : public BasePhysxPointer<physx::PxD6Joint> {

public:
    D6Joint(RigidActor a0, RigidActor a1,
            const Eigen::Vector3f &local_pos0, const Eigen::Vector4f &local_quat0,
            const Eigen::Vector3f &local_pos1, const Eigen::Vector4f &local_quat1) :
            BasePhysxPointer(physx::PxD6JointCreate(*Physics::get_physics(),
                                                    a0.get_physx_ptr(), eigen_to_transform(local_pos0, local_quat0),
                                                    a1.get_physx_ptr(), eigen_to_transform(local_pos1, local_quat1))) {}

};

#endif //PYPHYSX_D6JOINT_H
