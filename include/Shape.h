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
#include <array>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Shape : public BasePhysxPointer<physx::PxShape> {

public:
    explicit Shape(physx::PxShape *pref) : BasePhysxPointer<physx::PxShape>(pref) {}

    void set_flag(physx::PxShapeFlag::Enum flag, bool value) {
        get_physx_ptr()->setFlag(flag, value);
    }

    bool get_flag_value(physx::PxShapeFlag::Enum flag) {
        return get_physx_ptr()->getFlags().isSet(flag);
    }

    void set_local_pose(const physx::PxTransform &pose) {
        get_physx_ptr()->setLocalPose(pose);
    }

    auto get_local_pose() {
        return get_physx_ptr()->getLocalPose();
    }

    auto get_geometry_type() {
        return get_physx_ptr()->getGeometryType();
    }

    auto get_sphere_radius() {
        physx::PxSphereGeometry geom;
        get_physx_ptr()->getSphereGeometry(geom);
        return geom.radius;
    }

    auto get_box_half_extents() {
        physx::PxBoxGeometry geom;
        get_physx_ptr()->getBoxGeometry(geom);
        return geom.halfExtents;
    }

    Eigen::MatrixXf get_shape_data() {
        if (get_physx_ptr()->getGeometryType() == physx::PxGeometryType::eBOX) {
            return render_box_geometry();
        } else if (get_physx_ptr()->getGeometryType() == physx::PxGeometryType::eSPHERE) {
            return render_sphere_geometry(12, 12);
        } else if (get_physx_ptr()->getGeometryType() == physx::PxGeometryType::eCONVEXMESH) {
            return render_convex_geometry();
        }
        return Eigen::MatrixXf(0, 0);
    }

    void set_user_data(const pybind11::object &o) {
        if (get_physx_ptr()->userData != nullptr) { // release the old object first
            pybind11::handle(static_cast<PyObject *>(get_physx_ptr()->userData)).dec_ref();
        }
        o.inc_ref(); // make object safe from garbage collector
        get_physx_ptr()->userData = o.ptr();
    }

    auto get_user_data() {
        if (get_physx_ptr()->userData == nullptr) {
            return pybind11::reinterpret_borrow<pybind11::object>(pybind11::none());
        }
        return pybind11::reinterpret_borrow<pybind11::object>(
                pybind11::handle(
                        static_cast<PyObject *>(get_physx_ptr()->userData)
                )
        );
    }

    /** @brief Get all materials specified for this shape. */
    auto get_materials() {
        const auto n = get_physx_ptr()->getNbMaterials();
        std::vector<physx::PxMaterial *> materials(n);
        get_physx_ptr()->getMaterials(&materials[0], materials.size());
        return from_vector_of_physx_ptr<Material>(materials);
    }

    /** @brief Return boolean value indicating if the two shapes with overlaps if their global pose is known. */
    auto overlaps(const Shape &other_shape, const physx::PxTransform &global_pose,
                  const physx::PxTransform &global_pose_other) const {
        return physx::PxGeometryQuery::overlap(
                this->get_physx_ptr()->getGeometry().any(),
                global_pose * this->get_physx_ptr()->getLocalPose(),
                other_shape.get_physx_ptr()->getGeometry().any(),
                global_pose_other * other_shape.get_physx_ptr()->getLocalPose()
        );
    }

    static Shape create_box(const Eigen::Vector3f &sz, Material mat, bool is_exclusive) {
        return Shape::from_geometry(physx::PxBoxGeometry(0.5 * sz[0], 0.5 * sz[1], 0.5 * sz[2]), mat, is_exclusive);
    }

    static Shape create_sphere(float radius, Material mat, bool is_exclusive) {
        return Shape::from_geometry(physx::PxSphereGeometry(radius), mat, is_exclusive);
    }

    /** @brief Given sequence of points (nx3 matrix), cook convex mesh and create shape from it. */
    static Shape create_convex_mesh_from_points(const Eigen::MatrixXf &points, Material mat, bool is_exclusive,
                                                float scale, size_t quantized_count, size_t vertex_limit) {
        using namespace physx;
        std::vector<PxVec3> vertices(points.rows());
        for (size_t i = 0; i < points.rows(); ++i) {
            vertices[i] = PxVec3(points(i, 0), points(i, 1), points(i, 2));
        }

        PxConvexMeshDesc convexDesc;
        convexDesc.points.count = vertices.size();
        convexDesc.points.stride = sizeof(PxVec3);
        convexDesc.points.data = &vertices[0];
        convexDesc.flags =
                PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eQUANTIZE_INPUT | PxConvexFlag::eGPU_COMPATIBLE;
        convexDesc.quantizedCount = quantized_count;
        convexDesc.vertexLimit = vertex_limit;

        PxDefaultMemoryOutputStream buf;
        PxConvexMeshCookingResult::Enum result;
        if (!Physics::get().cooking->cookConvexMesh(convexDesc, buf, &result)) {
            std::cout << "Cannot cook convex mesh from points. Returning unit sphere instead. " << std::endl;
            return Shape::from_geometry(PxSphereGeometry(1.), mat, is_exclusive);
        }
        PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
        auto geom = PxConvexMeshGeometry(Physics::get_physics()->createConvexMesh(input), PxMeshScale(scale));
        return Shape::from_geometry(geom, mat, is_exclusive);
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

    /** @brief Based on SnippetRender from PhysX. */
    Eigen::MatrixXf render_convex_geometry() const {
        using namespace physx;
        PxConvexMeshGeometry geom;
        get_physx_ptr()->getConvexMeshGeometry(geom);

        PxConvexMesh *mesh = geom.convexMesh;
        const PxU32 nbPolys = mesh->getNbPolygons();
        const PxU8 *polygons = mesh->getIndexBuffer();
        const PxVec3 *verts = mesh->getVertices();
        std::list<std::array<PxVec3, 3>> triangle_vertices;
        for (PxU32 i = 0; i < nbPolys; i++) {
            PxHullPolygon data;
            mesh->getPolygonData(i, data);

            const PxU32 nbTris = PxU32(data.mNbVerts - 2);
            const PxU8 vref0 = polygons[data.mIndexBase + 0];
            for (PxU32 j = 0; j < nbTris; j++) {
                const PxU32 vref1 = polygons[data.mIndexBase + 0 + j + 1];
                const PxU32 vref2 = polygons[data.mIndexBase + 0 + j + 2];
                triangle_vertices.push_back({verts[vref0], verts[vref1], verts[vref2]});
            }
        }

        const PxVec3 &scale = geom.scale.scale;
        Eigen::MatrixXf data(triangle_vertices.size(), 3 * 3);
        size_t i = 0;
        for (const auto &triangle : triangle_vertices) {
            data.row(i++) << scale.x * triangle[0].x, scale.y * triangle[0].y, scale.z * triangle[0].z,
                    scale.x * triangle[1].x, scale.y * triangle[1].y, scale.z * triangle[1].z,
                    scale.x * triangle[2].x, scale.y * triangle[2].y, scale.z * triangle[2].z;
        }
        return data;
    }

};

#endif //PYPHYSX_SHAPE_H
