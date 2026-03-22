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

def _no_scale_matrix(ob):
    loc, rot, _sca = ob.matrix_world.decompose()
    return Matrix.Translation(loc) @ rot.to_matrix().to_4x4()


def _poly_scale(ob):
    _loc, _rot, sca = ob.matrix_world.decompose()
    return min(sca.x, sca.y)


# Polygon Point Gizmos

def _make_sdf_point_getter(point_index):
    def getter():
        ob = bpy.context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return (0.0, 0.0, 0.0)
        sdf = ob.data
        if point_index >= len(sdf.polygon_points):
            return (0.0, 0.0, 0.0)
        pt = sdf.polygon_points[point_index]
        s = _poly_scale(ob)
        return (pt.co[0] * s, pt.co[1] * s, 0.0)
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
        s = _poly_scale(ob)
        if s > 1e-6:
            pt.co[0] = value[0] / s
            pt.co[1] = value[1] / s
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
        s = _poly_scale(ob)
        b = _bisector_for_point(sdf, point_index)
        pos = Vector((pt.co[0] * s, pt.co[1] * s)) + b * (pt.corner * s + _CORNER_HANDLE_GAP)
        return (pos.x, pos.y, 0.0)
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
        s = _poly_scale(ob)
        vi_s = Vector((pt.co[0] * s, pt.co[1] * s))
        new_pos = Vector((value[0], value[1]))
        b = _bisector_for_point(sdf, point_index)
        dist = (new_pos - vi_s).dot(b)
        if s > 1e-6:
            pt.corner = max((dist - _CORNER_HANDLE_GAP) / s, 0.0)
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
        s = _poly_scale(ob)
        b = _bisector_for_point(sdf, i)
        start = (pt.co[0] * s, pt.co[1] * s, 0.0)
        offset = pt.corner * s + _CORNER_HANDLE_GAP
        end = (pt.co[0] * s + b.x * offset, pt.co[1] * s + b.y * offset, 0.0)

        gpu.matrix.push()
        gpu.matrix.multiply_matrix(self.matrix_basis)
        shader = gpu.shader.from_builtin('UNIFORM_COLOR')
        batch = batch_for_shader(shader, 'LINES', {"pos": [start, end]})
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
        if ob and ob.type == 'SDF' and ob.data:
            return ob.data.sdf_type == 'POLYGON'
        return False

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

        mat = _no_scale_matrix(ob)
        s = _poly_scale(ob)

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

        mat = _no_scale_matrix(ob)
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
        # Layout: 3 left (NW=Intersect, W=Union, SW=Push)
        #         3 right (NE=Shell, E=Subtract, SE=Avoid)


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
    """Open the CSG operation pie menu (Tab)"""
    bl_idname = "sdf.csg_pie_call"
    bl_label = "SDF CSG Pie"

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob is not None and ob.type == 'SDF'

    def execute(self, context):
        bpy.ops.wm.call_menu_pie(name="SDF_MT_csg_pie")
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

    def draw(self, context):
        layout = self.layout
        sdf = context.sdf

        row = layout.row(align=True)
        row.scale_y = 1.6
        row.prop(sdf, "sdf_type", expand=True, icon_only=True)

        layout.separator()

        layout.use_property_split = True
        layout.use_property_decorate = False
        layout.prop(sdf, "color")


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

        if not (sdf.sdf_group and sdf.group_order == 0):
            layout.label(text="CSG Operation")
            row = layout.row(align=True)
            row.scale_y = 1.4
            row.prop(sdf, "csg_operation", expand=True, icon_only=True)

            layout.separator()

            if sdf.csg_operation == 'SHELL':
                layout.label(text="Shell Mode")
                layout.prop(sdf, "shell_mode", text="")

            layout.label(text="Blend Type")
            row = layout.row(align=True)
            row.scale_y = 1.4
            row.prop(sdf, "blend_type", expand=True, icon_only=True)

            if sdf.blend_type != 'LINEAR':
                if sdf.csg_operation == 'SHELL':
                    layout.label(text="Blend Top")
                    layout.prop(sdf, "shell_blend_top", text="")
                    layout.label(text="Blend Bottom")
                    layout.prop(sdf, "shell_blend_bottom", text="")
                else:
                    layout.label(text="Blend")
                    layout.prop(sdf, "blend", text="")

                if sdf.blend_type in ('CHAMFER', 'ROUND'):
                    layout.label(text="Edge Smoothness")
                    row = layout.row(align=True)
                    row.prop(sdf, "chamfer_k2", text="K2")
                    row.prop(sdf, "chamfer_k3", text="K3")

            if sdf.csg_operation == 'SHELL':
                layout.label(text="Distance")
                layout.prop(sdf, "shell_distance", text="")

        layout.separator()


# Modifier Operators

class SDF_OT_modifier_add(Operator):
    """Add an SDF modifier"""
    bl_idname = "sdf.modifier_add"
    bl_label = "Add SDF Modifier"
    bl_options = {'REGISTER', 'UNDO'}

    type: EnumProperty(
        name="Type",
        items=[
            ('MIRROR', "Mirror", "Mirror across axes", 'MOD_MIRROR', 0),
            ('TWIST', "Twist", "Twist around Z axis", 'MOD_SIMPLEDEFORM', 1),
            ('BEND', "Bend", "Bend around an axis", 'MOD_SIMPLEDEFORM', 2),
            ('ELONGATE', "Elongate", "Stretch along axes", 'MOD_LENGTH', 3),
            ('HOLLOW', "Hollow", "Make hollow with wall thickness", 'MOD_SOLIDIFY', 4),
            ('ROUND', "Round", "Additional rounding", 'MOD_SMOOTH', 5),
            ('ONION', "Onion", "Concentric shells", 'MOD_SOLIDIFY', 6),
            ('BEVEL', "Bevel", "Bevel/round edges", 'MOD_BEVEL', 7),
            ('ARRAY', "Array", "Duplicate geometry", 'MOD_ARRAY', 8),
        ],
    )

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf', None) is not None

    def execute(self, context):
        sdf = getattr(context, 'sdf', None)
        if sdf is None:
            return {'CANCELLED'}
        mod = sdf.modifiers.new(type=self.type)

        if self.type == 'MIRROR' and context.object:
            ob = context.object
            empty = bpy.data.objects.new(f"{ob.name}_Mirror", None)
            empty.empty_display_type = 'PLAIN_AXES'
            empty.empty_display_size = 0.5
            empty.show_in_front = True
            empty["sdf_mirror_internal"] = True
            context.collection.objects.link(empty)
            empty.location = ob.location
            empty.parent = ob
            mod.mirror_object = empty

        return {'FINISHED'}


class SDF_OT_modifier_remove(Operator):
    """Remove an SDF modifier"""
    bl_idname = "sdf.modifier_remove"
    bl_label = "Remove SDF Modifier"
    bl_options = {'REGISTER', 'UNDO'}

    index: IntProperty()

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf', None) is not None

    def execute(self, context):
        sdf = getattr(context, 'sdf', None)
        if sdf is None:
            return {'CANCELLED'}
        if self.index < len(sdf.modifiers):
            mod = sdf.modifiers[self.index]
            if mod.type == 'MIRROR' and mod.mirror_object:
                mirror_ob = mod.mirror_object
                mod.mirror_object = None
                try:
                    bpy.data.objects.remove(mirror_ob)
                except RuntimeError:
                    pass
            sdf.modifiers.remove(mod)
        return {'FINISHED'}


class SDF_OT_modifier_move(Operator):
    """Move an SDF modifier up or down"""
    bl_idname = "sdf.modifier_move"
    bl_label = "Move SDF Modifier"
    bl_options = {'REGISTER', 'UNDO'}

    index: IntProperty()
    direction: IntProperty()  # -1 = up, 1 = down

    @classmethod
    def poll(cls, context):
        return getattr(context, 'sdf', None) is not None

    def execute(self, context):
        sdf = getattr(context, 'sdf', None)
        if sdf is None:
            return {'CANCELLED'}
        new_index = self.index + self.direction
        if 0 <= new_index < len(sdf.modifiers):
            sdf.modifiers.move(self.index, new_index)
        return {'FINISHED'}


# Modifier Panel

class DATA_PT_sdf_modifiers(SDFButtonsPanel, Panel):
    bl_label = "Modifiers"

    def draw(self, context):
        layout = self.layout
        sdf = context.sdf

        row = layout.row()
        row.menu("SDF_MT_modifier_add", text="Add Modifier", icon='ADD')

        if not sdf.modifiers:
            return

        for idx, mod in enumerate(sdf.modifiers):
            box = layout.box()
            row = box.row(align=True)
            row.prop(mod, "show_viewport", text="", icon='RESTRICT_VIEW_OFF' if mod.show_viewport else 'RESTRICT_VIEW_ON')
            row.prop(mod, "name", text="")

            sub = row.row(align=True)
            sub.scale_x = 0.8
            op = sub.operator("sdf.modifier_move", text="", icon='TRIA_UP')
            op.index = idx
            op.direction = -1
            op = sub.operator("sdf.modifier_move", text="", icon='TRIA_DOWN')
            op.index = idx
            op.direction = 1
            op = row.operator("sdf.modifier_remove", text="", icon='X')
            op.index = idx

            if not mod.show_viewport:
                continue

            col = box.column(align=True)
            col.use_property_split = True
            col.use_property_decorate = False

            if mod.type == 'MIRROR':
                row = col.row(align=True)
                row.label(text="Axes:")
                row.prop(mod, "use_mirror_x", toggle=True)
                row.prop(mod, "use_mirror_y", toggle=True)
                row.prop(mod, "use_mirror_z", toggle=True)
                col.prop(mod, "offset_distance")
                col.prop(mod, "mirror_object")

                box_csg = box.box()
                box_csg.label(text="Mirror Blending:")

                row = box_csg.row(align=True)
                row.scale_y = 1.2
                row.prop(mod, "blend_type", expand=True, icon_only=True)

                sub = box_csg.row()
                sub.enabled = (mod.blend_type != 'LINEAR')
                sub.prop(mod, "mirror_blend", text="Radius")

            elif mod.type == 'TWIST':
                col.prop(mod, "strength", text="Strength")
            elif mod.type == 'BEND':
                col.prop(mod, "strength", text="Strength")
                col.prop(mod, "bend_axis", text="Axis")
            elif mod.type == 'ELONGATE':
                col.prop(mod, "elongation")
            elif mod.type == 'HOLLOW':
                col.prop(mod, "thickness")
            elif mod.type == 'ROUND':
                col.prop(mod, "radius")
            elif mod.type == 'ONION':
                col.prop(mod, "thickness")
            elif mod.type == 'BEVEL':
                col.prop(mod, "radius")
            elif mod.type == 'ARRAY':
                col.prop(mod, "array_type")
                col.prop(mod, "count")
                if mod.array_type == 'LINEAR':
                    col.prop(mod, "offset")
                elif mod.array_type == 'RADIAL':
                    col.prop(mod, "array_radius")
                
                box_csg = box.box()
                box_csg.label(text="Array Blending:")
                
                row = box_csg.row(align=True)
                row.scale_y = 1.2
                row.prop(mod, "blend_type", expand=True, icon_only=True)
                
                sub = box_csg.row()
                sub.enabled = (mod.blend_type != 'LINEAR')
                sub.prop(mod, "array_blend", text="Radius")


class SDF_MT_modifier_add(Menu):
    bl_idname = "SDF_MT_modifier_add"
    bl_label = "Add SDF Modifier"

    def draw(self, _context):
        layout = self.layout
        layout.operator("sdf.modifier_add", text="Mirror", icon='MOD_MIRROR').type = 'MIRROR'
        layout.operator("sdf.modifier_add", text="Twist", icon='MOD_SIMPLEDEFORM').type = 'TWIST'
        layout.operator("sdf.modifier_add", text="Bend", icon='MOD_SIMPLEDEFORM').type = 'BEND'
        layout.operator("sdf.modifier_add", text="Elongate", icon='MOD_LENGTH').type = 'ELONGATE'
        layout.separator()
        layout.operator("sdf.modifier_add", text="Hollow", icon='MOD_SOLIDIFY').type = 'HOLLOW'
        layout.operator("sdf.modifier_add", text="Round", icon='MOD_SMOOTH').type = 'ROUND'
        layout.operator("sdf.modifier_add", text="Onion", icon='MOD_SOLIDIFY').type = 'ONION'
        layout.separator()
        layout.operator("sdf.modifier_add", text="Bevel", icon='MOD_BEVEL').type = 'BEVEL'
        layout.operator("sdf.modifier_add", text="Array", icon='MOD_ARRAY').type = 'ARRAY'


classes = (
    SDF_GT_corner_line,
    SDF_MT_csg_pie,
    SDF_MT_blend_pie,
    SDF_OT_csg_pie_call,
    SDF_OT_blend_pie_call,
    SDF_OT_modifier_add,
    SDF_OT_modifier_remove,
    SDF_OT_modifier_move,
    SDF_OT_polygon_point_add,
    SDF_OT_polygon_point_add_click,
    SDF_OT_polygon_point_remove,
    SDF_OT_polygon_point_remove_index,
    SDF_MT_polygon_point_context,
    SDF_OT_polygon_point_context_menu,
    VIEW3D_GGT_sdf_polygon,
    SDF_MT_modifier_add,
    DATA_PT_context_sdf,
    DATA_PT_sdf_shape,
    DATA_PT_sdf_operation,
    DATA_PT_sdf_property,
    DATA_PT_sdf_modifiers,
)


def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)


def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)


if __name__ == "__main__":
    register()
