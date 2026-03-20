# SPDX-FileCopyrightText: 2025 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Panel, Menu, Operator, GizmoGroup
from bpy.props import EnumProperty, IntProperty
from mathutils import Matrix, Vector


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
        return (pt.co[0], pt.co[1], 0.0)
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
        pt.co[0] = value[0]
        pt.co[1] = value[1]
    return setter


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
        self._rebuild(context)

    def _rebuild(self, context):
        for gz in self.gizmos_list:
            self.gizmos.remove(gz)
        self.gizmos_list.clear()

        ob = context.object
        if not ob or ob.type != 'SDF' or not ob.data:
            return
        sdf = ob.data
        if sdf.sdf_type != 'POLYGON':
            return

        for i, _pt in enumerate(sdf.polygon_points):
            gz = self.gizmos.new("GIZMO_GT_move_3d")
            gz.draw_style = 'SQUARE_2D'
            gz.draw_options = {'FILL', 'ALIGN_VIEW'}
            gz.use_draw_modal = True
            gz.use_draw_value = True
            gz.scale_basis = 0.08
            gz.color = 0.9, 0.9, 0.9
            gz.alpha = 0.9
            gz.color_highlight = 1.0, 1.0, 1.0
            gz.alpha_highlight = 1.0
            gz.target_set_handler("offset",
                                  get=_make_sdf_point_getter(i),
                                  set=_make_sdf_point_setter(i))
            self.gizmos_list.append(gz)

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

        mat = ob.matrix_world
        for i, gz in enumerate(self.gizmos_list):
            if i < count:
                gz.matrix_basis = mat


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
    SDF_MT_csg_pie,
    SDF_MT_blend_pie,
    SDF_OT_csg_pie_call,
    SDF_OT_blend_pie_call,
    SDF_OT_modifier_add,
    SDF_OT_modifier_remove,
    SDF_OT_modifier_move,
    SDF_OT_polygon_point_add,
    SDF_OT_polygon_point_remove,
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
