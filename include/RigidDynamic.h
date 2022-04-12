/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 4/30/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef SIM_PHYSX_RIGIDDYNAMIC_H
#define SIM_PHYSX_RIGIDDYNAMIC_H

#include <Physics.h>
#include <RigidActor.h>


class RigidDynamic : public RigidActor {

private:
    /** @brief Cast physx base pointer into physx RigidDynamic pointer. */
    auto get_dyn_ptr() {
        return reinterpret_cast<physx::PxRigidDynamic *>(get_physx_ptr());
    }

public:
    RigidDynamic() : RigidDynamic(Physics::get_physics()->createRigidDynamic(physx::PxTransform(physx::PxIdentity))) {
    }

    explicit RigidDynamic(physx::PxRigidDynamic *physxPtr) :
            RigidActor(reinterpret_cast<RigidActor::type_physx *>(physxPtr)) {
    }

    void set_mass(float mass) {
        physx::PxRigidBodyExt::setMassAndUpdateInertia(*get_dyn_ptr(), mass);
    }

    auto get_mass() {
        return get_dyn_ptr()->getMass();
    }

    void set_angular_damping(float damping) {
        get_dyn_ptr()->setAngularDamping(damping);
    }

    auto get_angular_damping() {
        return get_dyn_ptr()->getAngularDamping();
    }

    void set_angular_velocity(const physx::PxVec3 &vel) {
        get_dyn_ptr()->setAngularVelocity(vel);
    }

    auto get_angular_velocity() {
        return get_dyn_ptr()->getAngularVelocity();
    }

    void set_linear_damping(float damping) {
        get_dyn_ptr()->setLinearDamping(damping);
    }

    auto get_linear_damping() {
        return get_dyn_ptr()->getLinearDamping();
    }

    auto get_linear_velocity() {
        return get_dyn_ptr()->getLinearVelocity();
    }

    void set_linear_velocity(const physx::PxVec3 &vel) {
        get_dyn_ptr()->setLinearVelocity(vel);
    }

    void set_max_linear_velocity(float max_vel) {
        get_dyn_ptr()->setMaxLinearVelocity(max_vel);
    }

    void set_max_angular_velocity(float max_vel) {
        get_dyn_ptr()->setMaxAngularVelocity(max_vel);
    }

    void add_force(const physx::PxVec3 &force, physx::PxForceMode::Enum &force_mode) {
        get_dyn_ptr()->addForce(force, force_mode);
    }

    void add_torque(const physx::PxVec3 &torque, physx::PxForceMode::Enum &force_mode) {
        get_dyn_ptr()->addTorque(torque, force_mode);
    }

    void set_rigid_body_flag(const physx::PxRigidBodyFlag::Enum &flag, bool value) {
        get_dyn_ptr()->setRigidBodyFlag(flag, value);
    }

    void set_kinematic_target(const physx::PxTransform &pose) {
        get_dyn_ptr()->setKinematicTarget(pose);
    }
	
	void set_rigid_dynamic_lockflag(const physx::PxRigidDynamicLockFlag::Enum& flag, const bool value) {
        get_dyn_ptr()->setRigidDynamicLockFlag(flag, value);
    }

};

#endif //SIM_PHYSX_RIGIDDYNAMIC_H
