# SPDX-FileCopyrightText: 2025 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Operator to convert mesh objects to SDF grids via Geometry Nodes."""

import bpy
from bpy.types import Operator


def _get_or_create_node_group():
    """Return the 'Mesh to SDF Grid' node group, creating it if needed."""
    name = "Mesh to SDF Grid"
    existing = bpy.data.node_groups.get(name)
    if existing is not None:
        return existing

    tree = bpy.data.node_groups.new(name, 'GeometryNodeTree')

    # --- Interface sockets ---
    tree.interface.new_socket("Geometry", in_out='INPUT', socket_type='NodeSocketGeometry')
    vs_socket = tree.interface.new_socket(
        "Voxel Size", in_out='INPUT', socket_type='NodeSocketFloat',
    )
    vs_socket.default_value = 0.3
    vs_socket.min_value = 0.001
    vs_socket.subtype = 'DISTANCE'

    bw_socket = tree.interface.new_socket(
        "Band Width", in_out='INPUT', socket_type='NodeSocketInt',
    )
    bw_socket.default_value = 3
    bw_socket.min_value = 1

    blend_socket = tree.interface.new_socket(
        "Blend", in_out='INPUT', socket_type='NodeSocketFloat',
    )
    blend_socket.default_value = 0.0
    blend_socket.min_value = 0.0

    tree.interface.new_socket("Geometry", in_out='OUTPUT', socket_type='NodeSocketGeometry')

    # --- Nodes ---
    group_in = tree.nodes.new('NodeGroupInput')
    group_in.location = (-400, 0)

    mesh_to_sdf = tree.nodes.new('GeometryNodeMeshToSDFGrid')
    mesh_to_sdf.location = (-100, 0)

    store_grid = tree.nodes.new('GeometryNodeStoreNamedGrid')
    store_grid.location = (200, 0)
    # Set data type to FLOAT (SDF is scalar).
    store_grid.data_type = 'FLOAT'

    group_out = tree.nodes.new('NodeGroupOutput')
    group_out.location = (500, 0)

    # --- Links ---
    links = tree.links
    # Geometry -> Mesh to SDF Grid (Mesh input)
    links.new(group_in.outputs["Geometry"], mesh_to_sdf.inputs["Mesh"])
    # Voxel Size -> Mesh to SDF Grid
    links.new(group_in.outputs["Voxel Size"], mesh_to_sdf.inputs["Voxel Size"])
    # Band Width -> Mesh to SDF Grid
    links.new(group_in.outputs["Band Width"], mesh_to_sdf.inputs["Band Width"])
    # Mesh to SDF Grid (SDF Grid) -> Store Named Grid (Grid)
    links.new(mesh_to_sdf.outputs["SDF Grid"], store_grid.inputs["Grid"])
    # Store Named Grid (Volume) -> Group Output
    links.new(store_grid.outputs["Volume"], group_out.inputs["Geometry"])

    # Set grid name to "sdf".
    for inp in store_grid.inputs:
        if inp.name == "Name":
            inp.default_value = "sdf"
            break

    return tree


class OBJECT_OT_mesh_to_sdf_grid(Operator):
    """Add a Geometry Nodes modifier that converts a mesh to an SDF grid"""
    bl_idname = "object.mesh_to_sdf_grid"
    bl_label = "Mesh to SDF Grid"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (context.active_object is not None and
                context.active_object.type == 'MESH')

    def execute(self, context):
        node_group = _get_or_create_node_group()

        for ob in context.selected_objects:
            if ob.type != 'MESH':
                continue

            # Check if modifier already exists on this object.
            existing = None
            for mod in ob.modifiers:
                if (mod.type == 'NODES' and
                        mod.node_group == node_group):
                    existing = mod
                    break

            if existing is not None:
                continue

            mod = ob.modifiers.new(name="Mesh to SDF Grid", type='NODES')
            mod.node_group = node_group
            # Hide the node group selector widget.
            mod.show_group_selector = False

            # Hide the volume from other draw engines (Workbench).
            # BOUNDS shows only a minimal bounding box, not the full volume.
            # The SDF engine reads the volume data regardless of display type.
            ob.display_type = 'BOUNDS'

            ob.update_tag()

        context.view_layer.update()
        return {'FINISHED'}


classes = (
    OBJECT_OT_mesh_to_sdf_grid,
)
