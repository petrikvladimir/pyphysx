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
            return render_box_geometry();
        } else if (get_physx_ptr()->getGeometryType() == physx::PxGeometryType::eSPHERE) {
            return render_sphere_geometry(12, 12);
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


    /** @brief Get data for rendering box.
     *
     * Vertices of the box are:
     * v1 = x, -y, z,
     * v2 = x, y, z,
     * v3 = x, -y, -z,
     * v4 = x, y, -z,
     * v5 = -x, -y, -z,
     * v6 = -x, y, -z,
     * v7 = -x, -y, z,
     * v8 = -x, y, z,
     * */
    Eigen::MatrixXf render_box_geometry() const {
        physx::PxBoxGeometry geom;
        get_physx_ptr()->getBoxGeometry(geom);
        const auto x = geom.halfExtents.x;
        const auto y = geom.halfExtents.y;
        const auto z = geom.halfExtents.z;
        Eigen::MatrixXf data(6, 4 * 3);
        data.row(0) << x, y, z, x, -y, z, x, -y, -z, x, y, -z;  // 2-1-3-4
        data.row(1) << x, y, -z, x, -y, -z, -x, -y, -z, -x, y, -z;  // 4-3-5-6
        data.row(2) << -x, y, -z, -x, -y, -z, -x, -y, z, -x, y, z;  // 6-5-7-8
        data.row(3) << -x, y, z, -x, -y, z, x, -y, z, x, y, z;  // 8-7-1-2
        data.row(4) << -x, y, -z, -x, y, z, x, y, z, x, y, -z;  // 6-8-2-4
        data.row(5) << -x, -y, z, -x, -y, -z, x, -y, -z, x, -y, z;  // 7-5-3-1
        return data;
    }

    auto spherical_to_cartesian(double r, double theta, double rho) const {
        return std::make_tuple(r * sin(theta) * cos(rho), r * sin(theta) * sin(rho), r * cos(theta));
    }

    /** @brief Get data for rendering sphere geometry. */
    Eigen::MatrixXf render_sphere_geometry(size_t n_slices = 4, size_t n_segments = 4) const {
        Eigen::MatrixXf data((n_slices) * (n_segments), 4 * 3);
        physx::PxSphereGeometry geom;
        get_physx_ptr()->getSphereGeometry(geom);
        for (size_t slice = 0; slice < n_slices; ++slice) {
            const auto theta0 = M_PI * (slice) / float(n_slices);
            const auto theta1 = M_PI * (slice + 1) / float(n_slices);
            for (size_t segment = 0; segment < n_segments; ++segment) {
                const auto rho0 = 2 * M_PI * segment / float(n_segments);
                const auto rho1 = 2 * M_PI * (segment + 1) / float(n_segments);
                const auto p0 = spherical_to_cartesian(geom.radius, theta0, rho0);
                const auto p1 = spherical_to_cartesian(geom.radius, theta1, rho0);
                const auto p2 = spherical_to_cartesian(geom.radius, theta1, rho1);
                const auto p3 = spherical_to_cartesian(geom.radius, theta0, rho1);
                data.row(n_segments * slice + segment) << std::get<0>(p0), std::get<1>(p0), std::get<2>(p0),
                        std::get<0>(p1), std::get<1>(p1), std::get<2>(p1),
                        std::get<0>(p2), std::get<1>(p2), std::get<2>(p2),
                        std::get<0>(p3), std::get<1>(p3), std::get<2>(p3);
            }
        }
        return data;
    }

};

#endif //PYPHYSX_SHAPE_H
