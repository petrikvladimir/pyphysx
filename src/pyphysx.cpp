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

namespace py = pybind11;
using py::arg;

PYBIND11_MODULE(pyphysx, m) {
    py::class_<Physics>(m, "Physics")
            .def_static("set_num_cpu", &Physics::set_num_cpu, pybind11::arg("num_cpu") = 0);

    py::class_<Scene>(m, "Scene")
            .def(py::init<>())
            .def("simulate", &Scene::simulate, arg("dt") = 1. / 60., arg("num_substeps") = 1)
            .def("add_actor", &Scene::add_actor, arg("actor"))
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
            .def("get_shape_data", &Shape::get_shape_data);

    py::class_<RigidDynamic>(m, "RigidDynamic")
            .def(py::init<>())
            .def("attach_shape", &RigidDynamic::attach_shape, arg("shape"))
            .def("set_global_pose", &RigidDynamic::set_global_pose, arg("pos"),
                 arg("quat") = Eigen::Vector4f(0., 0., 0., 1.))
            .def("set_mass_and_update_inertia", &RigidDynamic::set_mass_and_update_inertia, arg("mass") = 1.)
            .def("get_global_pose", &RigidDynamic::get_global_pose)
            .def("get_mass", &RigidDynamic::get_mass)
            .def("get_atached_shapes", &RigidDynamic::get_atached_shapes);


}
