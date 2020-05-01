/**
 * Copyright (c) CTU  - All Rights Reserved
 * Created on: 3/16/20
 *     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Physics.h>
#include <Material.h>
#include <RigidDynamic.h>

namespace py = pybind11;

#ifndef NORENDER

PYBIND11_MODULE(pyphysx, m) {
#else
    PYBIND11_MODULE(pyphysx_norender, m) {
#endif


    py::class_<Physics>(m, "Physics")
            .def(py::init<int>(), pybind11::arg("num_cpu") = 0)
            .def("create_scene", &Physics::create_scene);

    py::class_<Scene>(m, "Scene");

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


    py::class_<RigidDynamic>(m, "RigidDynamic")
            .def(py::init<>());


}
