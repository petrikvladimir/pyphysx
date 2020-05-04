/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 5/1/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#ifndef PYPHYSX_SHAPE_H
#define PYPHYSX_SHAPE_H

#include <Eigen/Eigen>
#include <PxPhysicsAPI.h>
#include <BasePhysxPointer.h>
#include <Material.h>
#include <Physics.h>
#include <transformation_utils.h>

class Shape : public BasePhysxPointer<physx::PxShape> {

public:
    explicit Shape(physx::PxShape *pref) : BasePhysxPointer<physx::PxShape>(pref) {}

//    todo:  functions for enabling/disabling flags
// todo check flag status function

    void set_local_pose(const Eigen::Vector3f &pos, const Eigen::Vector4f &quat) {
        get_physx_ptr()->setLocalPose(eigen_to_transform(pos, quat));
    }

    auto get_local_pose() {
        return transform_to_eigen(get_physx_ptr()->getLocalPose());
    }

    Eigen::MatrixXf get_shape_data() {
        if (get_physx_ptr()->getGeometryType() == physx::PxGeometryType::eBOX) {
            physx::PxBoxGeometry geom;
            get_physx_ptr()->getBoxGeometry(geom);
            const auto x = -geom.halfExtents.x;
            const auto y = -geom.halfExtents.y;
            const auto z = -geom.halfExtents.z;
            const auto xp = geom.halfExtents.x;
            const auto yp = geom.halfExtents.y;
            const auto zp = geom.halfExtents.z;

            Eigen::MatrixXf data(6, 4 * 3);
            data.row(0) << xp, y, z, x, y, z, x, yp, z, xp, yp, z;
            data.row(1) << x, y, zp, xp, y, zp, xp, yp, zp, x, yp, zp;
            data.row(2) << x, y, z, x, y, zp, x, yp, zp, x, yp, z;
            data.row(3) << xp, y, zp, xp, y, z, xp, yp, z, xp, yp, zp;
            data.row(4) << x, y, z, xp, y, z, xp, y, zp, x, y, zp;
            data.row(5) << x, yp, zp, xp, yp, zp, xp, yp, z, x, yp, z;
            return data;
        }
        return Eigen::MatrixXf(0, 0);
    }

    static Shape create_box(const Eigen::Vector3f &sz, Material mat, bool is_exclusive) {
        return Shape::from_geometry(physx::PxBoxGeometry(0.5 * sz[0], 0.5 * sz[1], 0.5 * sz[2]), mat, is_exclusive);
    }

    static Shape create_sphere(float radius, Material mat, bool is_exclusive) {
        return Shape::from_geometry(physx::PxSphereGeometry(radius), mat, is_exclusive);
    }

private:
    static Shape from_geometry(const physx::PxGeometry &geometry, Material mat, bool is_exclusive) {
        return Shape(Physics::get_physics()->createShape(geometry, *mat.get_physx_ptr(), is_exclusive,
                                                         physx::PxShapeFlag::eSIMULATION_SHAPE |
                                                         physx::PxShapeFlag::eVISUALIZATION));
    }

};

#endif //PYPHYSX_SHAPE_H
