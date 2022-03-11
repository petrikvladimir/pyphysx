#!/usr/bin/env python

# Copyright (c) CTU  - All Rights Reserved
# Created on: 1/19/21
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

import numpy as np

import meshcat
import meshcat.geometry as g
import trimesh.exchange.obj

from pyphysx import ShapeFlag, GeometryType
from pyphysx_render.render_base import ViewerBase
from pyphysx_utils.transformations import multiply_transformations, pose_to_transformation_matrix


class MeshcatViewer(ViewerBase):

    def __init__(self, open_meshcat=False, print_url=False, wait_for_open=False, zmq_url=None,
                 show_frames=False, frame_scale=1., object_prefix="objects",
                 render_to_animation=False, animation_fps=30,
                 **kwargs) -> None:
        super().__init__()
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        if open_meshcat:
            self.vis.open()
        if print_url:
            self.vis.url()
        if wait_for_open:
            self.vis.wait()

        self.vis["/Background"].set_property("top_color", [1] * 3)
        self.vis["/Background"].set_property("bottom_color", [1] * 3)

        self.actors_and_offsets = []
        self.show_frames = show_frames
        self.frame_scale = frame_scale
        self.object_prefix = object_prefix
        self.animation = meshcat.animation.Animation(default_framerate=animation_fps) if render_to_animation else None
        self.itr = 0
        self._vis_group = None

    @property
    def vis_group(self):
        return self.vis[self.object_prefix] if self._vis_group is None else self._vis_group[self.object_prefix]

    def vis_frame(self, actor_id):
        return self.vis_group['frame' + str(actor_id)]

    def vis_actor(self, actor_id):
        return self.vis_group[str(actor_id)]

    def vis_shape(self, actor_id, shape_id):
        return self.vis_actor(actor_id=actor_id)[str(shape_id)]

    def get_start_index_for_next_scene(self):
        if len(self.actors_and_offsets) == 0:
            return 0
        return self.actors_and_offsets[-1][2] + len(self.actors_and_offsets[-1][0])

    def add_physx_scene(self, scene, render_shapes_with_one_of_flags=(ShapeFlag.VISUALIZATION,), offset=None):
        actors = scene.get_dynamic_rigid_actors() + scene.get_static_rigid_actors()
        start_index = self.get_start_index_for_next_scene()
        for i, actor in enumerate(actors, start=start_index):
            for j, shape in enumerate(actor.get_atached_shapes()):
                if not self.has_shape_any_of_flags(shape, render_shapes_with_one_of_flags):
                    continue
                if shape.get_geometry_type() == GeometryType.PLANE:  # plane is ignored as there is a grid in meshcat
                    continue
                self.vis_shape(i, j).set_object(self._get_shape_geometry(shape), self._get_shape_material(shape))
                self.vis_shape(i, j).set_transform(pose_to_transformation_matrix(shape.get_local_pose()))
            if self.show_frames:
                self.vis_frame(i).set_object(g.triad(self.frame_scale))
        self.actors_and_offsets.append((actors, offset, start_index))

    def _update_actors(self):
        for actors, offset, start_index in self.actors_and_offsets:
            for i, actor in enumerate(actors, start=start_index):
                pose = actor.get_global_pose()
                if offset is not None:
                    pose = multiply_transformations(offset, pose)
                self.vis_actor(i).set_transform(pose_to_transformation_matrix(pose))
                if self.show_frames:
                    self.vis_frame(i).set_transform(pose_to_transformation_matrix(pose))

    def update(self, blocking=False):
        if self.animation is not None:
            with self.animation.at_frame(self.vis, self.itr) as self._vis_group:
                self._update_actors()
        else:
            self._update_actors()
        self.itr += 1

    def clear_physx_scenes(self):
        for actors, _, start_index in self.actors_and_offsets:
            for i, actor in enumerate(actors, start=start_index):
                for j, shape in enumerate(actor.get_atached_shapes()):
                    self.vis_shape(i, j).delete()
                self.vis_actor(i).delete()
                self.vis_frame(i).delete()
        self.vis_group.delete()
        self.actors_and_offsets.clear()

    def _get_shape_material(self, shape):
        texture = shape.get_user_data().get('visual_mesh_texture', None) if shape.get_user_data() is not None else None
        if texture is not None:
            return g.MeshLambertMaterial(map=texture, opacity=1.)
        clr = [int(v) for v in self.get_shape_color(shape=shape)]
        color = int(clr[0]) * 256 ** 2 + int(clr[1]) * 256 + int(clr[2])
        return g.MeshLambertMaterial(color=color, opacity=clr[3] / 255.)

    def _get_shape_geometry(self, shape):
        visual_mesh = shape.get_user_data().get('visual_mesh', None) if shape.get_user_data() is not None else None
        if visual_mesh is not None:
            try:
                exp_obj = trimesh.exchange.obj.export_obj(visual_mesh)
            except ValueError:
                exp_obj = trimesh.exchange.obj.export_obj(visual_mesh, include_texture=False)
            return g.ObjMeshGeometry.from_stream(trimesh.util.wrap_as_stream(exp_obj))
        elif shape.get_geometry_type() == GeometryType.CONVEXMESH:
            data = shape.get_shape_data()  # N x 9 - i.e. 3 triangles
            faces = np.arange(0, data.shape[0] * 3, 1, dtype=np.int).reshape(-1, 3)
            return g.TriangularMeshGeometry(vertices=data.reshape(-1, 3), faces=faces)
        elif shape.get_geometry_type() == GeometryType.SPHERE:
            return g.Sphere(radius=shape.get_sphere_radius())
        elif shape.get_geometry_type() == GeometryType.BOX:
            return g.Box((2 * shape.get_box_half_extents()).tolist())
        else:
            raise NotImplementedError("Not supported geometry type.")

    def publish_animation(self, play=True, repetitions=1):
        """ If animation was recorded, then publish to meshcat server. """
        if self.animation is not None:
            self.vis.set_animation(self.animation, play=play, repetitions=repetitions)
