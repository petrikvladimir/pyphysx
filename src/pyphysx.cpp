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
#include <Aggregate.h>

namespace py = pybind11;
using py::arg;

PYBIND11_MODULE(pyphysx, m) {

    /***
     * Define enumerations.
     */

    py::enum_<physx::PxRigidBodyFlag::Enum>(m, "RigidBodyFlag")
            .value("KINEMATIC", physx::PxRigidBodyFlag::eKINEMATIC)
            .value("USE_KINEMATIC_TARGET_FOR_SCENE_QUERIES",
                   physx::PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES)
            .value("RETAIN_ACCELERATIONS", physx::PxRigidBodyFlag::eRETAIN_ACCELERATIONS)
            .value("ENABLE_CCD", physx::PxRigidBodyFlag::eENABLE_CCD)
            .value("ENABLE_CCD_FRICTION", physx::PxRigidBodyFlag::eENABLE_CCD_FRICTION)
            .value("ENABLE_CCD_MAX_CONTACT_IMPULSE", physx::PxRigidBodyFlag::eENABLE_CCD_MAX_CONTACT_IMPULSE)
            .value("ENABLE_SPECULATIVE_CCD", physx::PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
            .value("ENABLE_POSE_INTEGRATION_PREVIEW", physx::PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)
            .export_values();

    py::enum_<physx::PxSceneFlag::Enum>(m, "SceneFlag")
            .value("ENABLE_CCD", physx::PxSceneFlag::eENABLE_CCD)
            .value("DISABLE_CCD_RESWEEP", physx::PxSceneFlag::eDISABLE_CCD_RESWEEP)
            .value("ADAPTIVE_FORCE", physx::PxSceneFlag::eADAPTIVE_FORCE)
            .value("ENABLE_PCM", physx::PxSceneFlag::eENABLE_PCM)
            .value("DISABLE_CONTACT_REPORT_BUFFER_RESIZE", physx::PxSceneFlag::eDISABLE_CONTACT_REPORT_BUFFER_RESIZE)
            .value("DISABLE_CONTACT_CACHE", physx::PxSceneFlag::eDISABLE_CONTACT_CACHE)
            .value("REQUIRE_RW_LOCK", physx::PxSceneFlag::eREQUIRE_RW_LOCK)
            .value("ENABLE_STABILIZATION", physx::PxSceneFlag::eENABLE_STABILIZATION)
            .value("ENABLE_AVERAGE_POINT", physx::PxSceneFlag::eENABLE_AVERAGE_POINT)
            .value("EXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS", physx::PxSceneFlag::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS)
            .value("ENABLE_GPU_DYNAMICS", physx::PxSceneFlag::eENABLE_GPU_DYNAMICS)
            .value("ENABLE_ENHANCED_DETERMINISM", physx::PxSceneFlag::eENABLE_ENHANCED_DETERMINISM)
            .value("ENABLE_FRICTION_EVERY_ITERATION", physx::PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION)
            .value("MUTABLE_FLAGS", physx::PxSceneFlag::eMUTABLE_FLAGS)
            .export_values();

    py::enum_<physx::PxBroadPhaseType::Enum>(m, "BroadPhaseType")
            .value("SAP", physx::PxBroadPhaseType::eSAP)
            .value("MBP", physx::PxBroadPhaseType::eMBP)
            .value("ABP", physx::PxBroadPhaseType::eABP)
            .value("GPU", physx::PxBroadPhaseType::eGPU)
            .export_values();

    py::enum_<physx::PxFrictionType::Enum>(m, "FrictionType")
            .value("PATCH", physx::PxFrictionType::ePATCH)
            .value("ONE_DIRECTIONAL", physx::PxFrictionType::eONE_DIRECTIONAL)
            .value("TWO_DIRECTIONAL", physx::PxFrictionType::eTWO_DIRECTIONAL)
            .export_values();

    py::enum_<physx::PxShapeFlag::Enum>(m, "ShapeFlag")
            .value("SIMULATION_SHAPE", physx::PxShapeFlag::eSIMULATION_SHAPE)
            .value("SCENE_QUERY_SHAPE", physx::PxShapeFlag::eSCENE_QUERY_SHAPE)
            .value("TRIGGER_SHAPE", physx::PxShapeFlag::eTRIGGER_SHAPE)
            .value("VISUALIZATION", physx::PxShapeFlag::eVISUALIZATION)
            .export_values();

    py::enum_<physx::PxD6Axis::Enum>(m, "D6Axis")
            .value("X", physx::PxD6Axis::eX)
            .value("Y", physx::PxD6Axis::eY)
            .value("Z", physx::PxD6Axis::eZ)
            .value("SWING1", physx::PxD6Axis::eSWING1)
            .value("SWING2", physx::PxD6Axis::eSWING2)
            .value("TWIST", physx::PxD6Axis::eTWIST)
            .export_values();

    py::enum_<physx::PxD6Motion::Enum>(m, "D6Motion")
            .value("FREE", physx::PxD6Motion::eFREE)
            .value("LIMITED", physx::PxD6Motion::eLIMITED)
            .value("LOCKED", physx::PxD6Motion::eLOCKED)
            .export_values();

    py::enum_<physx::PxD6Drive::Enum>(m, "D6Drive")
            .value("X", physx::PxD6Drive::eX)
            .value("Y", physx::PxD6Drive::eY)
            .value("Z", physx::PxD6Drive::eZ)
            .value("SWING", physx::PxD6Drive::eSWING)
            .value("TWIST", physx::PxD6Drive::eTWIST)
            .value("SLERP", physx::PxD6Drive::eSLERP)
            .export_values();

    py::enum_<physx::PxForceMode::Enum>(m, "ForceMode")
            .value("FORCE", physx::PxForceMode::eFORCE)
            .value("ACCELERATION", physx::PxForceMode::eACCELERATION)
            .value("IMPULSE", physx::PxForceMode::eIMPULSE)
            .value("VELOCITY_CHANGE", physx::PxForceMode::eVELOCITY_CHANGE)
            .export_values();

    /***
     * Define classes interface.
     */

    py::class_<Physics>(m, "Physics")
            .def_static("set_num_cpu", &Physics::set_num_cpu,
                        arg("num_cpu") = 0
            )
            .def_static("init_gpu", &Physics::init_gpu);

    py::class_<Scene>(m, "Scene")
            .def(py::init<physx::PxFrictionType::Enum, physx::PxBroadPhaseType::Enum, std::vector<physx::PxSceneFlag::Enum>, size_t, float>(),
                 arg("friction_type") = physx::PxFrictionType::ePATCH,
                 arg("broad_phase_type") = physx::PxBroadPhaseType::eABP,
                 arg("scene_flags") = std::vector<physx::PxSceneFlag::Enum>(),
                 arg("gpu_max_num_partitions") = 8,
                 arg("gpu_dynamic_allocation_scale") = 1.
            )
            .def("simulate", &Scene::simulate,
                 arg("dt") = 1. / 60.
            )
            .def("add_actor", &Scene::add_actor,
                 arg("actor")
            )
            .def("get_static_rigid_actors", &Scene::get_static_rigid_actors)
            .def("get_dynamic_rigid_actors", &Scene::get_dynamic_rigid_actors)
            .def("add_aggregate", &Scene::add_aggregate,
                 arg("agg")
            )
            .def("get_aggregates", &Scene::get_aggregates)
            .def_readwrite("simulation_time", &Scene::simulation_time);

    py::class_<Aggregate>(m, "Aggregate")
            .def(py::init<size_t, bool>(),
                 arg("max_size") = 256,
                 arg("enable_self_collision") = false
            )
            .def("add_actor", &Aggregate::add_actor, arg("actor"))
            .def("remove_actor", &Aggregate::remove_actor, arg("actor"))
            .def("get_actors", &Aggregate::get_actors);

    py::class_<Material>(m, "Material")
            .def(py::init<float, float, float>(),
                 arg("static_friction") = 0.,
                 arg("dynamic_friction") = 0.,
                 arg("restitution") = 0.
            )
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
            .def("set_static_friction", &Material::set_static_friction,
                 arg("static_friction") = 0.
            )
            .def("set_dynamic_friction", &Material::set_dynamic_friction,
                 arg("dynamic_friction") = 0.
            )
            .def("set_restitution", &Material::set_restitution,
                 arg("restitution") = 0.
            );

    py::class_<Shape>(m, "Shape")
            .def_static("create_box", &Shape::create_box,
                        arg("size"),
                        arg("material"),
                        arg("is_exclusive") = true
            )
            .def_static("create_sphere", &Shape::create_sphere,
                        arg("radius"),
                        arg("material"),
                        arg("is_exclusive") = true
            )
            .def_static("create_convex_mesh_from_points", &Shape::create_convex_mesh_from_points,
                        arg("points"),
                        arg("material"),
                        arg("is_exclusive") = true,
                        arg("scale") = 1.,
                        arg("quantized_count") = 255,
                        arg("vertex_limit") = 255
            )
            .def("get_shape_data", &Shape::get_shape_data)
            .def("set_local_pose", &Shape::set_local_pose,
                 arg("pose") = physx::PxTransform(physx::PxIdentity)
            )
            .def("get_local_pose", &Shape::get_local_pose)
            .def("set_user_data", &Shape::set_user_data,
                 arg("o")
            )
            .def("get_user_data", &Shape::get_user_data)
            .def("set_flag", &Shape::set_flag,
                 arg("flag"),
                 arg("value") = true
            )
            .def("get_flag_value", &Shape::get_flag_value,
                 arg("flag")
            );

    py::class_<RigidActor>(m, "RigidActor")
            .def("set_global_pose", &RigidActor::set_global_pose,
                 arg("pose") = physx::PxTransform(physx::PxIdentity)
            )
            .def("get_global_pose", &RigidActor::get_global_pose)
            .def("attach_shape", &RigidActor::attach_shape,
                 arg("shape")
            )
            .def("detach_shape", &RigidActor::detach_shape,
                 arg("shape")
            )
            .def("get_atached_shapes", &RigidActor::get_atached_shapes)
            .def("disable_gravity", &RigidActor::disable_gravity)
            .def("enable_gravity", &RigidActor::enable_gravity)
            .def("set_user_data", &RigidActor::set_user_data,
                 arg("o")
            )
            .def("get_user_data", &RigidActor::get_user_data)
            .def("overlaps", &RigidActor::overlaps,
                 arg("other_actor"),
                 "Check if current actor overlaps with a given actor."
            );


    py::class_<RigidDynamic, RigidActor>(m, "RigidDynamic")
            .def(py::init<>())
            .def("get_mass", &RigidDynamic::get_mass)
            .def("set_mass", &RigidDynamic::set_mass,
                 arg("mass") = 1.
            )
            .def("get_angular_damping", &RigidDynamic::get_angular_damping)
            .def("set_angular_damping", &RigidDynamic::set_angular_damping,
                 arg("damping") = 0.
            )
            .def("get_linear_damping", &RigidDynamic::get_linear_damping)
            .def("set_linear_damping", &RigidDynamic::set_linear_damping,
                 arg("damping") = 0.
            )
            .def("get_angular_velocity", &RigidDynamic::get_angular_velocity)
            .def("set_angular_velocity", &RigidDynamic::set_angular_velocity,
                 arg("vel")
            )
            .def("get_linear_velocity", &RigidDynamic::get_linear_velocity)
            .def("set_linear_velocity", &RigidDynamic::set_linear_velocity,
                 arg("vel")
            )
            .def("set_max_linear_velocity", &RigidDynamic::set_max_linear_velocity,
                 arg("max_vel")
            )
            .def("set_max_angular_velocity", &RigidDynamic::set_max_angular_velocity,
                 arg("max_vel")
            )
            .def("add_force", &RigidDynamic::add_force,
                 arg("force"),
                 arg("force_mode") = physx::PxForceMode::eFORCE
            )
            .def("add_torque", &RigidDynamic::add_torque,
                 arg("torque"),
                 arg("torque_mode") = physx::PxForceMode::eFORCE
            )
            .def("set_rigid_body_flag", &RigidDynamic::set_rigid_body_flag,
                 arg("flag"),
                 arg("value")
            )
            .def("set_kinematic_target", &RigidDynamic::set_kinematic_target,
                 arg("pose") = physx::PxTransform(physx::PxIdentity)
            );

    py::class_<RigidStatic, RigidActor>(m, "RigidStatic")
            .def(py::init<>())
            .def_static("create_plane", &RigidStatic::create_plane,
                        arg("mat"),
                        arg("nx") = 0.,
                        arg("ny") = 0.,
                        arg("nz") = 1.,
                        arg("distance") = 0.
            );

    py::class_<D6Joint>(m, "D6Joint")
            .def(py::init<RigidActor, RigidActor, physx::PxTransform, physx::PxTransform>(),
                 arg("actor0"), arg("actor1"),
                 arg("local_pose0") = physx::PxTransform(physx::PxIdentity),
                 arg("local_pose1") = physx::PxTransform(physx::PxIdentity)
            )
            .def("set_motion", &D6Joint::set_motion,
                 arg("axis"),
                 arg("motion")
            )
            .def("get_local_pose", &D6Joint::get_local_pose,
                 arg("actor_id") = 0
            )
            .def("get_relative_transform", &D6Joint::get_relative_transform)
            .def("set_linear_limit", &D6Joint::set_linear_limit,
                 arg("axis"),
                 arg("lower_limit"),
                 arg("upper_limit"),
                 arg("contact_dist") = -1
            )
            .def("set_linear_limit_soft", &D6Joint::set_linear_limit_soft,
                 arg("axis"),
                 arg("lower_limit"),
                 arg("upper_limit"),
                 arg("spring_stiffness"),
                 arg("spring_damping")
            )
            .def("set_twist_limit", &D6Joint::set_twist_limit,
                 arg("lower_limit"),
                 arg("upper_limit"),
                 arg("contact_dist") = -1
            )
            .def("set_twist_limit_soft", &D6Joint::set_twist_limit_soft,
                 arg("lower_limit"),
                 arg("upper_limit"),
                 arg("spring_stiffness"),
                 arg("spring_damping")
            )
            .def("set_swing_limit", &D6Joint::set_swing_limit,
                 arg("y_limit_angle"),
                 arg("z_limit_angle"),
                 arg("contact_dist") = -1
            )
            .def("set_swing_limit_soft", &D6Joint::set_swing_limit_soft,
                 arg("y_limit_angle"),
                 arg("z_limit_angle"),
                 arg("spring_stiffness"),
                 arg("spring_damping")
            )
            .def("set_break_force", &D6Joint::set_break_force,
                 arg("force"),
                 arg("torque")
            )
            .def("set_drive", &D6Joint::set_drive,
                 arg("axis"),
                 arg("stiffness"),
                 arg("damping"),
                 arg("force_limit"),
                 arg("is_acceleration") = false
            )
            .def("set_drive_position", &D6Joint::set_drive_position,
                 arg("pose") = physx::PxTransform(physx::PxIdentity)
            )
            .def("set_drive_velocity", &D6Joint::set_drive_velocity,
                 arg("linear") = Eigen::Vector3f(0., 0., 0.),
                 arg("angular") = Eigen::Vector3f(0., 0., 0.)
            )
            .def("get_drive_position", &D6Joint::get_drive_position)
            .def("get_drive_velocity", &D6Joint::get_drive_velocity)
            .def("get_force_torque", &D6Joint::get_force_torque)
            .def("is_broken", &D6Joint::is_broken)
            .def("release", &D6Joint::release);


    /***
     * Arbitrary support functions.
     */

    m.def("cast_transformation", &cast_transformation,
          arg("pose") = physx::PxTransform(physx::PxIdentity),
          "A function that takes all allowed pose representation and returns tuple pose representation.");

}
