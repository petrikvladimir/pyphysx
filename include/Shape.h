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
            const auto x = geom.halfExtents.x;
            const auto y = geom.halfExtents.y;
            const auto z = geom.halfExtents.z;
//            v1 = x, -y, z,
//            v2 = x, y, z,
//            v3 = x, -y, -z,
//            v4 = x, y, -z,
//            v5 = -x, -y, -z,
//            v6 = -x, y, -z,
//            v7 = -x, -y, z,
//            v8 = -x, y, z,
            Eigen::MatrixXf data(6, 4 * 3);
            data.row(0) << x, y, z, x, -y, z, x, -y, -z, x, y, -z;  // 2-1-3-4
            data.row(1) << x, y, -z, x, -y, -z, -x, -y, -z, -x, y, -z;  // 4-3-5-6
            data.row(2) << -x, y, -z, -x, -y, -z, -x, -y, z, -x, y, z;  // 6-5-7-8
            data.row(3) << -x, y, z, -x, -y, z, x, -y, z, x, y, z;  // 8-7-1-2
            data.row(4) << -x, y, -z, -x, y, z, x, y, z, x, y, -z;  // 6-8-2-4
            data.row(5) << -x, -y, z, -x, -y, -z, x, -y, -z, x, -y, z;  // 7-5-3-1
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
