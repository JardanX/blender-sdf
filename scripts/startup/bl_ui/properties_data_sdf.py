# SPDX-FileCopyrightText: 2025 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from bpy.types import Panel, Menu, Operator, GizmoGroup
from bpy.props import EnumProperty, IntProperty
from mathutils import Matrix, Vector

_CORNER_HANDLE_GAP = 0.15
_context_point_index = -1


# Polygon Point Gizmos

def _local_to_world_offset(ob, local_x, local_y):
    """Transform a local XY point to a world-space offset from the object origin."""
    world = ob.matrix_world @ Vector((local_x, local_y, 0.0))
    origin = ob.matrix_world.translation
    return (world.x - origin.x, world.y - origin.y, world.z - origin.z)


def _world_offset_to_local(ob, wx, wy, wz):
    """Transform a world-space offset (from object origin) back to local XY."""
    origin = ob.matrix_world.translation
    world = Vector((wx + origin.x, wy + origin.y, wz + origin.z))
    local = ob.matrix_world.inverted() @ world
    return (local.x, local.y)


def _make_sdf_point_getter(point_index):
    def getter():
        ob = bpy.context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return (0.0, 0.0, 0.0)
        sdf = ob.data
        if point_index >= len(sdf.polygon_points):
            return (0.0, 0.0, 0.0)
        pt = sdf.polygon_points[point_index]
        return _local_to_world_offset(ob, pt.co[0], pt.co[1])
    return getter

def _make_sdf_point_setter(point_index):
    def setter(value):
        ob = bpy.context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return
        sdf = ob.data
        if point_index >= len(sdf.polygon_points):
            return
        pt = sdf.polygon_points[point_index]
        lx, ly = _world_offset_to_local(ob, value[0], value[1], value[2])
        pt.co[0] = lx
        pt.co[1] = ly
        ob.data.update_tag()
    return setter


def _bisector_for_point(sdf, index):
    pts = sdf.polygon_points
    pc = len(pts)
    if pc < 3:
        return Vector((0.0, 1.0))
    ip = (index - 1) % pc
    in_ = (index + 1) % pc
    vi = Vector((pts[index].co[0], pts[index].co[1]))
    vp = Vector((pts[ip].co[0], pts[ip].co[1]))
    vn = Vector((pts[in_].co[0], pts[in_].co[1]))
    to_prev = (vp - vi).normalized()
    to_next = (vn - vi).normalized()
    bisector = (to_prev + to_next)
    if bisector.length < 1e-6:
        bisector = Vector((-to_prev.y, to_prev.x))
    else:
        bisector.normalize()
    cross = to_prev.x * to_next.y - to_prev.y * to_next.x
    if cross > 0:
        bisector = -bisector
    return bisector


def _make_corner_getter(point_index):
    def getter():
        ob = bpy.context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return (0.0, 0.0, 0.0)
        sdf = ob.data
        if point_index >= len(sdf.polygon_points):
            return (0.0, 0.0, 0.0)
        pt = sdf.polygon_points[point_index]
        b = _bisector_for_point(sdf, point_index)
        lx = pt.co[0] + b.x * (pt.corner + _CORNER_HANDLE_GAP)
        ly = pt.co[1] + b.y * (pt.corner + _CORNER_HANDLE_GAP)
        return _local_to_world_offset(ob, lx, ly)
    return getter


def _make_corner_setter(point_index):
    def setter(value):
        ob = bpy.context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return
        sdf = ob.data
        if point_index >= len(sdf.polygon_points):
            return
        pt = sdf.polygon_points[point_index]
        lx, ly = _world_offset_to_local(ob, value[0], value[1], value[2])
        vi = Vector((pt.co[0], pt.co[1]))
        new_pos = Vector((lx, ly))
        b = _bisector_for_point(sdf, point_index)
        dist = (new_pos - vi).dot(b)
        pt.corner = max(dist - _CORNER_HANDLE_GAP, 0.0)
        ob.data.update_tag()
    return setter


class SDF_GT_corner_line(bpy.types.Gizmo):
    bl_idname = "SDF_GT_corner_line"

    def setup(self):
        self._point_index = 0

    def draw(self, context):
        ob = context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return
        sdf = ob.data
        i = self._point_index
        if i >= len(sdf.polygon_points):
            return
        pt = sdf.polygon_points[i]
        b = _bisector_for_point(sdf, i)
        offset = pt.corner + _CORNER_HANDLE_GAP

        s0 = _local_to_world_offset(ob, pt.co[0], pt.co[1])
        s1 = _local_to_world_offset(ob, pt.co[0] + b.x * offset, pt.co[1] + b.y * offset)

        gpu.matrix.push()
        gpu.matrix.multiply_matrix(self.matrix_basis)
        shader = gpu.shader.from_builtin('UNIFORM_COLOR')
        batch = batch_for_shader(shader, 'LINES', {"pos": [s0, s1]})
        shader.bind()
        c = self._color if hasattr(self, '_color') else (0.25, 0.45, 0.7, 0.5)
        shader.uniform_float("color", c)
        gpu.state.blend_set('ALPHA')
        batch.draw(shader)
        gpu.state.blend_set('NONE')
        gpu.matrix.pop()

    def draw_select(self, context, select_id):
        pass

    def test_select(self, context, location):
        return -1


class VIEW3D_GGT_sdf_polygon(GizmoGroup):
    bl_idname = "VIEW3D_GGT_sdf_polygon"
    bl_label = "SDF Polygon Points"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'WINDOW'
    bl_options = {'3D', 'PERSISTENT', 'SHOW_MODAL_ALL'}

    @classmethod
    def poll(cls, context):
        ob = context.object
        if not (ob and ob.type == 'SDF' and ob.data and ob.data.sdf_type == 'POLYGON'):
            return False
        if not ob.select_get():
            return False
        view = context.space_data
        if view and hasattr(view, 'overlay'):
            if not view.overlay.show_overlays or not view.overlay.show_sdf_ngon:
                return False
        return True

    def setup(self, context):
        self.gizmos_list = []
        self.corner_gizmos = []
        self.line_gizmos = []
        self._rebuild(context)

    def _rebuild(self, context):
        for gz in self.gizmos_list:
            self.gizmos.remove(gz)
        self.gizmos_list.clear()
        for gz in self.corner_gizmos:
            self.gizmos.remove(gz)
        self.corner_gizmos.clear()
        for gz in self.line_gizmos:
            self.gizmos.remove(gz)
        self.line_gizmos.clear()

        ob = context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return
        sdf = ob.data
        if sdf.sdf_type != 'POLYGON':
            return

        mat = Matrix.Translation(ob.matrix_world.translation)

        for i, _pt in enumerate(sdf.polygon_points):
            gz = self.gizmos.new("GIZMO_GT_move_3d")
            gz.draw_style = 'SQUARE_2D'
            gz.draw_options = {'FILL', 'ALIGN_VIEW'}
            gz.use_draw_modal = True
            gz.use_draw_value = True
            gz.scale_basis = 0.08
            gz.color = 0.4, 0.7, 1.0
            gz.alpha = 0.9
            gz.color_highlight = 1.0, 1.0, 1.0
            gz.alpha_highlight = 1.0
            gz.matrix_basis = mat
            gz.target_set_handler("offset",
                                  get=_make_sdf_point_getter(i),
                                  set=_make_sdf_point_setter(i))
            self.gizmos_list.append(gz)

            cgz = self.gizmos.new("GIZMO_GT_move_3d")
            cgz.draw_style = 'RING_2D'
            cgz.draw_options = {'ALIGN_VIEW'}
            cgz.use_draw_modal = True
            cgz.use_draw_value = True
            cgz.scale_basis = 0.06
            cgz.color = 0.25, 0.45, 0.7
            cgz.alpha = 0.5
            cgz.color_highlight = 0.6, 0.9, 1.0
            cgz.alpha_highlight = 1.0
            cgz.matrix_basis = mat
            cgz.target_set_handler("offset",
                                   get=_make_corner_getter(i),
                                   set=_make_corner_setter(i))
            self.corner_gizmos.append(cgz)

            lgz = self.gizmos.new("SDF_GT_corner_line")
            lgz._point_index = i
            lgz.matrix_basis = mat
            self.line_gizmos.append(lgz)

        self._last_count = len(sdf.polygon_points)

    def draw_prepare(self, context):
        ob = context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return
        mat = Matrix.Translation(ob.matrix_world.translation)
        for gz in self.gizmos_list:
            gz.matrix_basis = mat
        for cgz in self.corner_gizmos:
            cgz.matrix_basis = mat
        for lgz in self.line_gizmos:
            lgz.matrix_basis = mat

    def refresh(self, context):
        ob = context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return
        sdf = ob.data
        if sdf.sdf_type != 'POLYGON':
            return

        count = len(sdf.polygon_points)
        if count != getattr(self, '_last_count', -1):
            self._rebuild(context)
            return

        any_modal = any(gz.is_modal for gz in self.gizmos_list) or \
                    any(gz.is_modal for gz in self.corner_gizmos)
        was_modal = getattr(self, '_was_modal', False)
        if was_modal and not any_modal:
            bpy.app.timers.register(
                lambda: (bpy.ops.ed.undo_push(message="Edit SDF Polygon"), None)[1],
                first_interval=0.0)
        self._was_modal = any_modal

        mat = Matrix.Translation(ob.matrix_world.translation)
        for i in range(count):
            gz = self.gizmos_list[i]
            cgz = self.corner_gizmos[i]
            lgz = self.line_gizmos[i]
            gz.matrix_basis = mat
            cgz.matrix_basis = mat
            lgz.matrix_basis = mat

            active = gz.is_highlight or gz.is_modal or cgz.is_highlight or cgz.is_modal
            if active:
                gz.color = 1.0, 1.0, 1.0
                gz.alpha = 1.0
                cgz.color = 0.4, 0.7, 1.0
                cgz.alpha = 0.9
                lgz._color = (0.4, 0.7, 1.0, 0.9)
            else:
                gz.color = 0.4, 0.7, 1.0
                gz.alpha = 0.9
                cgz.color = 0.25, 0.45, 0.7
                cgz.alpha = 0.5
                lgz._color = (0.25, 0.45, 0.7, 0.4)


# Pie Menus

class SDF_MT_shape_pie(Menu):
    bl_idname = "SDF_MT_shape_pie"
    bl_label = "Shape"

    def draw(self, context):
        pie = self.layout.menu_pie()
        sdf = context.object.data
        pie.prop_enum(sdf, "sdf_type", value='BOX')         # W
        pie.prop_enum(sdf, "sdf_type", value='SPHERE')      # E
        pie.prop_enum(sdf, "sdf_type", value='CONE')        # S
        pie.prop_enum(sdf, "sdf_type", value='CYLINDER')    # N
        pie.prop_enum(sdf, "sdf_type", value='TORUS')       # NW
        pie.prop_enum(sdf, "sdf_type", value='CAPSULE')     # NE
        pie.prop_enum(sdf, "sdf_type", value='NGON')        # SW
        pie.prop_enum(sdf, "sdf_type", value='POLYGON')     # SE


class SDF_MT_csg_pie(Menu):
    bl_idname = "SDF_MT_csg_pie"
    bl_label = "CSG Operation"

    def draw(self, context):
        pie = self.layout.menu_pie()
        sdf = context.object.data
        pie.prop_enum(sdf, "csg_operation", value='UNION')       # W
        pie.prop_enum(sdf, "csg_operation", value='SUBTRACT')    # E
        pie.separator()                                           # S (skip)
        pie.separator()                                           # N (skip)
        pie.prop_enum(sdf, "csg_operation", value='INTERSECT')   # NW
        pie.prop_enum(sdf, "csg_operation", value='SHELL')       # NE
        pie.prop_enum(sdf, "csg_operation", value='PUSH')        # SW
        pie.prop_enum(sdf, "csg_operation", value='AVOID')       # SE


class SDF_MT_main_pie(Menu):
    bl_idname = "SDF_MT_main_pie"
    bl_label = "SDF"

    def draw(self, context):
        pie = self.layout.menu_pie()
        sdf = context.object.data
        pie.prop_enum(sdf, "csg_operation", value='UNION')       # W
        pie.prop_enum(sdf, "csg_operation", value='SUBTRACT')    # E
        op = pie.operator("wm.call_menu_pie", text="CSG Operation", icon='MOD_BOOLEAN')  # S
        op.name = "SDF_MT_csg_pie"
        op = pie.operator("wm.call_menu_pie", text="Shape", icon='MESH_UVSPHERE')        # N
        op.name = "SDF_MT_shape_pie"
        pie.prop_enum(sdf, "csg_operation", value='INTERSECT')   # NW
        pie.prop_enum(sdf, "csg_operation", value='SHELL')       # NE
        pie.prop_enum(sdf, "csg_operation", value='PUSH')        # SW
        pie.prop_enum(sdf, "csg_operation", value='AVOID')       # SE


class SDF_MT_blend_pie(Menu):
    bl_idname = "SDF_MT_blend_pie"
    bl_label = "Blend Type"

    def draw(self, context):
        pie = self.layout.menu_pie()
        sdf = context.object.data
        pie.prop_enum(sdf, "blend_type", value='LINEAR')
        pie.prop_enum(sdf, "blend_type", value='SMOOTH')
        pie.prop_enum(sdf, "blend_type", value='CHAMFER')
        pie.prop_enum(sdf, "blend_type", value='ROUND')


class SDF_OT_csg_pie_call(Operator):
    """Open the SDF pie menu (Tab) — Shape on top, CSG on bottom"""
    bl_idname = "sdf.csg_pie_call"
    bl_label = "SDF Pie"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob is not None and ob.type == 'SDF'

    def execute(self, context):
        bpy.ops.wm.call_menu_pie(name="SDF_MT_main_pie")
        return {'FINISHED'}


class SDF_OT_blend_pie_call(Operator):
    """Open the blend type pie menu (Shift+Tab)"""
    bl_idname = "sdf.blend_pie_call"
    bl_label = "SDF Blend Pie"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob is not None and ob.type == 'SDF'

    def execute(self, context):
        bpy.ops.wm.call_menu_pie(name="SDF_MT_blend_pie")
        return {'FINISHED'}


# Properties Panels

class SDFButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "data"

    @classmethod
    def poll(cls, context):
        return context.sdf


class DATA_PT_context_sdf(SDFButtonsPanel, Panel):
    bl_label = ""
    bl_options = {'HIDE_HEADER'}

    def draw(self, context):
        layout = self.layout

        ob = context.object
        sdf = context.sdf
        space = context.space_data

        if ob:
            layout.template_ID(ob, "data")
        elif sdf:
            layout.template_ID(space, "pin_id")


class DATA_PT_sdf_shape(SDFButtonsPanel, Panel):
    bl_label = "Shape"

    @classmethod
    def poll(cls, context):
        sdf = context.sdf
        return sdf is not None and sdf.sdf_type != 'GROUP'

    def draw(self, context):
        layout = self.layout
        sdf = context.sdf

        grid = layout.grid_flow(row_major=True, columns=4, even_columns=True, even_rows=True, align=True)
        grid.scale_x = 1.0
        grid.scale_y = 1.6
        for item in sdf.bl_rna.properties["sdf_type"].enum_items:
            if item.identifier == 'GROUP':
                continue
            grid.prop_enum(sdf, "sdf_type", item.identifier, text="")

        layout.separator()

        layout.use_property_split = True
        layout.use_property_decorate = False
        layout.prop(sdf, "color")


class DATA_PT_sdf_group(SDFButtonsPanel, Panel):
    bl_label = "Group"

    @classmethod
    def poll(cls, context):
        sdf = context.sdf
        return sdf is not None and sdf.sdf_type == 'GROUP'

    def draw(self, context):
        layout = self.layout
        sdf = context.sdf
        layout.use_property_split = True
        layout.use_property_decorate = False
        layout.prop(sdf, "color", text="Tint")


# Shape Property Panel

class DATA_PT_sdf_property(SDFButtonsPanel, Panel):
    bl_label = "Property"

    @classmethod
    def poll(cls, context):
        sdf = context.sdf
        return sdf and sdf.sdf_type in ('BOX', 'NGON', 'TORUS', 'POLYGON')

    def draw(self, context):
        layout = self.layout
        sdf = context.sdf

        if sdf.sdf_type == 'NGON':
            self.draw_ngon(layout, sdf)
        elif sdf.sdf_type == 'TORUS':
            self.draw_torus(layout, sdf)
        elif sdf.sdf_type == 'POLYGON':
            self.draw_polygon(layout, sdf)
        else:
            self.draw_box(layout, sdf)

    @staticmethod
    def draw_box(layout, sdf):
        # Corner Bevels
        layout.label(text="Corner Bevels")
        col = layout.column(align=True)
        row = col.row(align=True)
        row.prop(sdf, "box_corners", index=0, text="1")
        row.prop(sdf, "box_corners", index=1, text="2")
        row = col.row(align=True)
        row.prop(sdf, "box_corners", index=2, text="3")
        row.prop(sdf, "box_corners", index=3, text="4")

        layout.separator()

        # Edge Chamfer
        layout.label(text="Edge Chamfer")
        col = layout.column(align=True)
        row = col.row(align=True)
        row.prop(sdf, "box_edge_top", text="Top")
        row.prop(sdf, "box_edge_bottom", text="Bottom")

        layout.separator()

        # Taper
        layout.prop(sdf, "box_taper")

        # Mode dropdowns — only show when any shape property is active
        corners = sdf.box_corners
        has_shape = (corners[0] + corners[1] + corners[2] + corners[3]
                     + sdf.box_edge_top + sdf.box_edge_bottom
                     + abs(sdf.box_taper)) > 0.001
        if has_shape:
            layout.separator()
            row = layout.row(align=True)
            row.prop(sdf, "box_corner_mode", text="Corners")
            row.prop(sdf, "box_edge_mode", text="Edges")

    @staticmethod
    def draw_torus(layout, sdf):
        layout.prop(sdf, "torus_angle")

    @staticmethod
    def draw_ngon(layout, sdf):
        # Sides
        layout.prop(sdf, "ngon_sides")

        layout.separator()

        # Star
        layout.prop(sdf, "ngon_star")

        layout.separator()

        # Corner Bevel
        layout.prop(sdf, "ngon_corner")

        layout.separator()

        # Edge Chamfer
        layout.label(text="Edge Chamfer")
        col = layout.column(align=True)
        row = col.row(align=True)
        row.prop(sdf, "ngon_edge_top", text="Top")
        row.prop(sdf, "ngon_edge_bottom", text="Bottom")

        layout.separator()

        # Taper
        layout.prop(sdf, "ngon_taper")

        # Edge mode — only show when any shape property is active
        has_shape = (sdf.ngon_corner + sdf.ngon_edge_top
                     + sdf.ngon_edge_bottom + abs(sdf.ngon_taper)
                     + sdf.ngon_star) > 0.001
        if has_shape:
            layout.separator()
            layout.prop(sdf, "ngon_edge_mode", text="Edges")

    @staticmethod
    def draw_polygon(layout, sdf):
        layout.prop(sdf, "polygon_is_line")

        if sdf.polygon_is_line:
            layout.prop(sdf, "polygon_line_thickness")
            layout.separator()

        layout.label(text="Points")
        for i, pt in enumerate(sdf.polygon_points):
            row = layout.row(align=True)
            row.prop(pt, "co", index=0, text=f"P{i+1} X")
            row.prop(pt, "co", index=1, text="Y")
            row.prop(pt, "corner", text="R")

        row = layout.row(align=True)
        row.operator("sdf.polygon_point_add", text="Add Point", icon='ADD')
        row.operator("sdf.polygon_point_remove", text="Remove", icon='REMOVE')

        layout.separator()

        layout.label(text="Edge Chamfer")
        col = layout.column(align=True)
        row = col.row(align=True)
        row.prop(sdf, "polygon_edge_top", text="Top")
        row.prop(sdf, "polygon_edge_bottom", text="Bottom")

        layout.separator()

        layout.prop(sdf, "polygon_taper")

        has_shape = (sdf.polygon_edge_top + sdf.polygon_edge_bottom
                     + abs(sdf.polygon_taper)) > 0.001
        if has_shape:
            layout.separator()
            layout.prop(sdf, "polygon_edge_mode", text="Edges")


# Polygon Point Operators

class SDF_OT_polygon_point_add(Operator):
    """Add a point to the polygon"""
    bl_idname = "sdf.polygon_point_add"
    bl_label = "Add Polygon Point"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        sdf = getattr(context, 'sdf', None)
        return sdf is not None and sdf.sdf_type == 'POLYGON'

    def execute(self, context):
        sdf = context.sdf
        sdf.polygon_points.new(x=0.0, y=0.0)
        return {'FINISHED'}


def _mouse_to_local_xy(context, event):
    from bpy_extras import view3d_utils
    region = context.region
    rv3d = context.region_data
    coord = (event.mouse_region_x, event.mouse_region_y)

    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
    view_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)

    ob = context.object
    mat_inv = ob.matrix_world.inverted()
    local_origin = mat_inv @ ray_origin
    local_dir = (mat_inv @ (ray_origin + view_vector) - local_origin).normalized()

    if abs(local_dir.z) < 1e-6:
        return None
    t = -local_origin.z / local_dir.z
    local_pos = local_origin + local_dir * t
    return Vector((local_pos.x, local_pos.y))


def _nearest_edge_index(sdf, local_xy):
    pts = sdf.polygon_points
    n = len(pts)
    if n < 2:
        return 0
    best_dist = float('inf')
    best_i = 0
    for i in range(n):
        a = Vector((pts[i].co[0], pts[i].co[1]))
        b = Vector((pts[(i + 1) % n].co[0], pts[(i + 1) % n].co[1]))
        ab = b - a
        ab_sq = ab.dot(ab)
        if ab_sq < 1e-12:
            d = (local_xy - a).length
        else:
            t = max(0.0, min(1.0, (local_xy - a).dot(ab) / ab_sq))
            closest = a + ab * t
            d = (local_xy - closest).length
        if d < best_dist:
            best_dist = d
            best_i = i
    return best_i


class SDF_OT_polygon_point_add_click(Operator):
    """Subdivide the nearest polygon edge at its midpoint"""
    bl_idname = "sdf.polygon_point_add_click"
    bl_label = "Subdivide Polygon Edge"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob and ob.type == 'SDF' and ob.data and ob.data.sdf_type == 'POLYGON'

    def invoke(self, context, event):
        ob = context.object
        sdf = ob.data
        local_xy = _mouse_to_local_xy(context, event)
        if local_xy is None:
            return {'CANCELLED'}

        pts = sdf.polygon_points
        n = len(pts)
        edge_i = _nearest_edge_index(sdf, local_xy)

        a = Vector((pts[edge_i].co[0], pts[edge_i].co[1]))
        b = Vector((pts[(edge_i + 1) % n].co[0], pts[(edge_i + 1) % n].co[1]))
        mid = (a + b) * 0.5

        sdf.polygon_points.new(x=mid.x, y=mid.y)
        last = len(pts) - 1
        target = edge_i + 1
        sdf.polygon_points.move(last, target)

        ob.data.update_tag()
        return {'FINISHED'}


class SDF_OT_polygon_point_remove(Operator):
    """Remove the last point from the polygon"""
    bl_idname = "sdf.polygon_point_remove"
    bl_label = "Remove Polygon Point"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        sdf = getattr(context, 'sdf', None)
        return sdf is not None and sdf.sdf_type == 'POLYGON' and len(sdf.polygon_points) > 3

    def execute(self, context):
        sdf = context.sdf
        pts = sdf.polygon_points
        if len(pts) > 3:
            pts.remove(pts[-1])
        return {'FINISHED'}


class SDF_OT_polygon_point_remove_index(Operator):
    """Remove a specific polygon point by index"""
    bl_idname = "sdf.polygon_point_remove_index"
    bl_label = "Delete Point"
    bl_options = {'REGISTER', 'UNDO'}

    index: IntProperty(default=-1)

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob and ob.type == 'SDF' and ob.data and ob.data.sdf_type == 'POLYGON' and len(ob.data.polygon_points) > 3

    def execute(self, context):
        sdf = context.object.data
        pts = sdf.polygon_points
        if len(pts) > 3 and 0 <= self.index < len(pts):
            pts.remove(pts[self.index])
            context.object.data.update_tag()
        return {'FINISHED'}


class SDF_MT_polygon_point_context(Menu):
    bl_idname = "SDF_MT_polygon_point_context"
    bl_label = "Polygon Point"

    def draw(self, context):
        layout = self.layout
        idx = _context_point_index
        if idx >= 0:
            op = layout.operator("sdf.polygon_point_remove_index", icon='X')
            op.index = idx


class SDF_OT_polygon_point_context_menu(Operator):
    """Right-click context menu for the nearest polygon point"""
    bl_idname = "sdf.polygon_point_context_menu"
    bl_label = "Polygon Point Context Menu"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob and ob.type == 'SDF' and ob.data and ob.data.sdf_type == 'POLYGON'

    def invoke(self, context, event):
        from bpy_extras import view3d_utils
        region = context.region
        rv3d = context.region_data
        coord = (event.mouse_region_x, event.mouse_region_y)
        ob = context.object
        sdf = ob.data
        pts = sdf.polygon_points
        mat = ob.matrix_world

        best_dist = 30.0  # pixel threshold
        best_i = -1
        for i, pt in enumerate(pts):
            world = mat @ Vector((pt.co[0], pt.co[1], 0.0))
            screen = view3d_utils.location_3d_to_region_2d(region, rv3d, world)
            if screen is None:
                continue
            d = (Vector(coord) - screen).length
            if d < best_dist:
                best_dist = d
                best_i = i

        if best_i < 0:
            bpy.ops.wm.call_menu(name="VIEW3D_MT_object_context_menu")
            return {'FINISHED'}

        global _context_point_index
        _context_point_index = best_i
        bpy.ops.wm.call_menu(name="SDF_MT_polygon_point_context")
        return {'FINISHED'}


# Operation Panel

class DATA_PT_sdf_operation(SDFButtonsPanel, Panel):
    bl_label = "Operation"

    @classmethod
    def poll(cls, context):
        return context.sdf is not None

    def draw(self, context):
        layout = self.layout
        sdf = context.sdf

        ob = context.object
        is_first_in_scene = (sdf.sdf_index == 0)
        is_group = (sdf.sdf_type == 'GROUP')
        is_child = (ob and ob.parent and ob.parent.type == 'SDF')
        is_first_child = is_child and sdf.sdf_index == 0
        is_forced_union = (is_first_in_scene and not is_group and not is_child) or is_first_child

        layout.label(text="CSG Operation")
        grid = layout.grid_flow(row_major=True, columns=4, even_columns=True, even_rows=True, align=True)
        grid.scale_y = 1.4
        grid.enabled = not is_forced_union
        for item in sdf.bl_rna.properties["csg_operation"].enum_items:
            grid.prop_enum(sdf, "csg_operation", item.identifier, text="")
        if is_forced_union:
            layout.label(text="First in stack — forced to Union", icon='INFO')

        if not is_forced_union:
            layout.separator()

            layout.label(text="Blend Type")
            grid = layout.grid_flow(row_major=True, columns=4, even_columns=True, even_rows=True, align=True)
            grid.scale_y = 1.4
            for item in sdf.bl_rna.properties["blend_type"].enum_items:
                grid.prop_enum(sdf, "blend_type", item.identifier, text="")

            is_shell = (sdf.csg_operation == 'SHELL')
            bt = sdf.blend_type

            if is_shell:
                row = layout.row(align=True)
                row.prop(sdf, "shell_distance", text="Distance")
                sub = row.row(align=True)
                sub.scale_x = 0.9
                sub.enabled = (sdf.shell_mode != 'AVOID')
                sub.prop_enum(sdf, "shell_op", 'UNION', text="", icon='ADD')
                sub.prop_enum(sdf, "shell_op", 'SUBTRACTION', text="", icon='REMOVE')
                row = layout.row(align=True)
                row.prop(sdf, "shell_mode", expand=True)

            if bt != 'LINEAR':
                col = layout.column(align=True)
                col.separator()

                if is_shell:
                    inward = (sdf.shell_op == 'SUBTRACTION')
                    start_flip_on = (not inward) or (bt == 'ROUND')
                    end_flip_on = inward or (bt == 'ROUND')

                    row = col.row(align=True)
                    row.prop(sdf, "shell_blend_top", text="Start")
                    sub = row.row()
                    sub.enabled = start_flip_on
                    sub.prop(sdf, "flip_blend", text="", icon='UV_SYNC_SELECT', toggle=True)

                    row = col.row(align=True)
                    row.prop(sdf, "shell_blend_bottom", text="End")
                    sub = row.row()
                    sub.enabled = end_flip_on
                    sub.prop(sdf, "flip_blend_end", text="", icon='UV_SYNC_SELECT', toggle=True)

                    if sdf.shell_mode == 'AVOID':
                        col.prop(sdf, "blend", text="Avoid Blend")
                else:
                    col.prop(sdf, "blend", text="Blend Strength")

                if bt in ('CHAMFER', 'ROUND'):
                    col.separator()
                    if is_shell:
                        col.label(text="Smooth Start")
                        row = col.row(align=True)
                        row.prop(sdf, "chamfer_k2", text="K2")
                        row.prop(sdf, "chamfer_k3", text="K3")
                        col.label(text="Smooth End")
                        row = col.row(align=True)
                        row.prop(sdf, "chamfer_k4", text="K4")
                        row.prop(sdf, "chamfer_k5", text="K5")
                    else:
                        lbl = "Smooth Chamfer" if bt == 'CHAMFER' else "Smooth Round"
                        col.label(text=lbl)
                        row = col.row(align=True)
                        row.prop(sdf, "chamfer_k2", text="K2")
                        row.prop(sdf, "chamfer_k3", text="K3")

        layout.separator()


# MATHOPS: Removed — SDF modifiers moved to native Blender modifier system




classes = (
    SDF_GT_corner_line,
    SDF_MT_shape_pie,
    SDF_MT_csg_pie,
    SDF_MT_main_pie,
    SDF_MT_blend_pie,
    SDF_OT_csg_pie_call,
    SDF_OT_blend_pie_call,
    SDF_OT_polygon_point_add,
    SDF_OT_polygon_point_add_click,
    SDF_OT_polygon_point_remove,
    SDF_OT_polygon_point_remove_index,
    SDF_MT_polygon_point_context,
    SDF_OT_polygon_point_context_menu,
    VIEW3D_GGT_sdf_polygon,
    DATA_PT_context_sdf,
    DATA_PT_sdf_shape,
    DATA_PT_sdf_group,
    DATA_PT_sdf_operation,
    DATA_PT_sdf_property,
)


_sdf_group_cleanup_pending = False

def _sdf_group_cleanup_deferred():
    global _sdf_group_cleanup_pending
    _sdf_group_cleanup_pending = False
    try:
        to_delete = []
        for ob in list(bpy.data.objects):
            if ob.type != 'SDF' or ob.data is None:
                continue
            if ob.data.sdf_type != 'GROUP':
                continue
            has_children = False
            for c in ob.children:
                if c.type == 'SDF':
                    has_children = True
                    break
            if not has_children:
                to_delete.append(ob)
        for ob in to_delete:
            bpy.data.objects.remove(ob, do_unlink=True)
    except Exception:
        pass
    return None

def _sdf_group_cleanup(scene, depsgraph):
    global _sdf_group_cleanup_pending
    if not _sdf_group_cleanup_pending:
        _sdf_group_cleanup_pending = True
        bpy.app.timers.register(_sdf_group_cleanup_deferred, first_interval=0.2)


def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
    bpy.app.handlers.depsgraph_update_post.append(_sdf_group_cleanup)


def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)
    if _sdf_group_cleanup in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(_sdf_group_cleanup)


if __name__ == "__main__":
    register()
