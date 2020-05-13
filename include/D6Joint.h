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

    void set_motion(physx::PxD6Axis::Enum axis, physx::PxD6Motion::Enum type) {
        get_physx_ptr()->setMotion(axis, type);
    }

    /** @brief Set hard joint limit. */
    void set_linear_limit(physx::PxD6Axis::Enum axis, float lower_limit, float upper_limit, float contact_dist) {
        get_physx_ptr()->setLinearLimit(axis, physx::PxJointLinearLimitPair(
                Physics::get_physics()->getTolerancesScale(), lower_limit, upper_limit, contact_dist));
    }

    /** @brief Set soft joint limit. */
    void set_linear_limit_soft(physx::PxD6Axis::Enum axis, float lower_limit, float upper_limit, float spring_stiffness,
                               float spring_damping) {
        get_physx_ptr()->setLinearLimit(axis, physx::PxJointLinearLimitPair(
                lower_limit, upper_limit, physx::PxSpring(spring_stiffness, spring_damping)));
    }

    void set_twist_limit(float lower_limit, float upper_limit, float contact_dist) {
        get_physx_ptr()->setTwistLimit(physx::PxJointAngularLimitPair(lower_limit, upper_limit, contact_dist));
    }

    void set_twist_limit_soft(float lower_limit, float upper_limit, float spring_stiffness, float spring_damping) {
        get_physx_ptr()->setSwingLimit(physx::PxJointLimitCone(
                lower_limit, upper_limit, physx::PxSpring(spring_stiffness, spring_damping)));
    }

    void set_swing_limit(float y_limit_angle, float z_limit_angle, float contact_dist) {
        get_physx_ptr()->setSwingLimit(physx::PxJointLimitCone(y_limit_angle, z_limit_angle, contact_dist));
    }

    void set_swing_limit_soft(float y_limit_angle, float z_limit_angle, float spring_stiffness, float spring_damping) {
        get_physx_ptr()->setSwingLimit(physx::PxJointLimitCone(
                y_limit_angle, z_limit_angle, physx::PxSpring(spring_stiffness, spring_damping)));
    }

    void set_break_force(float force, float torque) {
        get_physx_ptr()->setBreakForce(force, torque);
    }

    void set_drive(physx::PxD6Drive::Enum axis, float stiffness, float damping, float force_limit,
                   bool is_acceleration) {
        get_physx_ptr()->setDrive(axis, physx::PxD6JointDrive(
                stiffness, damping, force_limit, is_acceleration));
    }

    void set_drive_position(const Eigen::Vector3f &pos, const Eigen::Vector4f &quat) {
        get_physx_ptr()->setDrivePosition(eigen_to_transform(pos, quat));
    }

    void set_drive_velocity(const Eigen::Vector3f &linear, const Eigen::Vector3f &angular) {
        get_physx_ptr()->setDriveVelocity(eigen_to_pxvec(linear), eigen_to_pxvec(angular));
    }

    auto get_drive_position() {
        return transform_to_eigen(get_physx_ptr()->getDrivePosition());
    }

    auto get_drive_velocity() {
        physx::PxVec3 lin, ang;
        get_physx_ptr()->getDriveVelocity(lin, ang);
        return std::make_tuple(pxvec_to_eigen(lin), pxvec_to_eigen(ang));
    }

    auto get_force_torque() {
        physx::PxVec3 force, torque;
        get_physx_ptr()->getConstraint()->getForce(force, torque);
        return std::make_tuple(pxvec_to_eigen(force), pxvec_to_eigen(torque));
    }

};

#endif //PYPHYSX_D6JOINT_H
