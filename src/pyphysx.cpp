/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 3/16/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Physics.h>
#include <Scene.h>
#include <Material.h>
#include <RigidDynamic.h>
#include <Shape.h>
#include <RigidStatic.h>
#include <D6Joint.h>

namespace py = pybind11;
using py::arg;

PYBIND11_MODULE(pyphysx, m) {
    py::class_<Physics>(m, "Physics")
            .def_static("set_num_cpu", &Physics::set_num_cpu, pybind11::arg("num_cpu") = 0);

    py::class_<Scene>(m, "Scene")
            .def(py::init<>())
            .def("simulate", &Scene::simulate, arg("dt") = 1. / 60., arg("num_substeps") = 1)
            .def("add_actor", &Scene::add_actor, arg("actor"))
            .def("get_static_rigid_actors", &Scene::get_static_rigid_actors)
            .def("get_dynamic_rigid_actors", &Scene::get_dynamic_rigid_actors)
            .def_readwrite("simulation_time", &Scene::simulation_time);

    py::class_<Material>(m, "Material")
            .def(py::init<float, float, float>(),
                 pybind11::arg("static_friction") = 0.,
                 pybind11::arg("dynamic_friction") = 0.,
                 pybind11::arg("restitution") = 0.)
            .def("__repr__",
                 [](const Material &m) {
                     return "<pyphysx.Material ("
                            + std::to_string(m.get_static_friction()) + ", "
                            + std::to_string(m.get_dynamic_friction()) + ", "
                            + std::to_string(m.get_restitution()) + ") >";
                 })
            .def("get_static_friction", &Material::get_static_friction)
            .def("get_dynamic_friction", &Material::get_dynamic_friction)
            .def("get_restitution", &Material::get_restitution)
            .def("set_static_friction", &Material::set_static_friction, pybind11::arg("static_friction") = 0.)
            .def("set_dynamic_friction", &Material::set_dynamic_friction, pybind11::arg("dynamic_friction") = 0.)
            .def("set_restitution", &Material::set_restitution, pybind11::arg("restitution") = 0.);

    py::class_<Shape>(m, "Shape")
            .def_static("create_box", &Shape::create_box, pybind11::arg("size"), pybind11::arg("material"),
                        pybind11::arg("is_exclusive") = true)
            .def_static("create_sphere", &Shape::create_sphere, pybind11::arg("radius"), pybind11::arg("material"),
                        pybind11::arg("is_exclusive") = true)
            .def_static("create_convex_mesh_from_points", &Shape::create_convex_mesh_from_points, arg("points"),
                        arg("material"), arg("is_exclusive") = true, arg("scale") = 1., arg("quantized_count") = 255,
                        arg("vertex_limit") = 255)
            .def("get_shape_data", &Shape::get_shape_data)
            .def("set_local_pose", &Shape::set_local_pose, arg("pos"),
                 arg("quat") = Eigen::Vector4f(0., 0., 0., 1.))
            .def("get_local_pose", &Shape::get_local_pose);

    py::class_<RigidActor>(m, "RigidActor")
            .def("set_global_pose", &RigidDynamic::set_global_pose, arg("pos"),
                 arg("quat") = Eigen::Vector4f(0., 0., 0., 1.))
            .def("get_global_pose", &RigidDynamic::get_global_pose)
            .def("attach_shape", &RigidDynamic::attach_shape, arg("shape"))
            .def("get_atached_shapes", &RigidDynamic::get_atached_shapes)
            .def("disable_gravity", &RigidDynamic::disable_gravity)
            .def("enable_gravity", &RigidDynamic::enable_gravity);

    py::class_<RigidDynamic, RigidActor>(m, "RigidDynamic")
            .def(py::init<>())
            .def("get_mass", &RigidDynamic::get_mass)
            .def("set_mass", &RigidDynamic::set_mass, arg("mass") = 1.)
            .def("get_angular_damping", &RigidDynamic::get_angular_damping)
            .def("set_angular_damping", &RigidDynamic::set_angular_damping, arg("damping") = 0.)
            .def("get_linear_damping", &RigidDynamic::get_linear_damping)
            .def("set_linear_damping", &RigidDynamic::set_linear_damping, arg("damping") = 0.)
            .def("get_angular_velocity", &RigidDynamic::get_angular_velocity)
            .def("set_angular_velocity", &RigidDynamic::set_angular_velocity, arg("vel"))
            .def("get_linear_velocity", &RigidDynamic::get_linear_velocity)
            .def("set_linear_velocity", &RigidDynamic::set_linear_velocity, arg("vel"))
            .def("set_max_linear_velocity", &RigidDynamic::set_max_linear_velocity, arg("max_vel"))
            .def("set_max_angular_velocity", &RigidDynamic::set_max_angular_velocity, arg("max_vel"));

    py::class_<RigidStatic, RigidActor>(m, "RigidStatic")
            .def(py::init<>())
            .def_static("create_plane", &RigidStatic::create_plane, arg("mat"), arg("nx") = 0., arg("ny") = 0.,
                        arg("nz") = 1., arg("distance") = 0.);

    py::class_<D6Joint>(m, "D6Joint")
            .def(py::init<RigidActor, RigidActor, Eigen::Vector3f, Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector4f>(),
                 arg("actor0"), arg("actor1"),
                 arg("local_pos0") = Eigen::Vector3f(0., 0., 0.), arg("local_quat0") = Eigen::Vector4f(0., 0., 0., 1.),
                 arg("local_pos1") = Eigen::Vector3f(0., 0., 0.), arg("local_quat1") = Eigen::Vector4f(0., 0., 0., 1.)
            );
}
