# SPDX-FileCopyrightText: 2009-2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.types import Menu, Panel, UIList
from bpy.app.translations import contexts as i18n_contexts
from rna_prop_ui import PropertyPanel
from bl_ui.space_properties import PropertiesAnimationMixin


class MATERIAL_MT_context_menu(Menu):
    bl_label = "Material Specials"

    def draw(self, _context):
        layout = self.layout

        layout.operator("material.copy", icon='COPYDOWN')
        layout.operator("object.material_slot_copy")
        layout.operator("material.paste", icon='PASTEDOWN')
        layout.operator("object.material_slot_remove_unused")
        layout.operator("object.material_slot_remove_all")


class MATERIAL_UL_matslots(UIList):

    def draw_item(self, _context, layout, _data, item, icon, _active_data, _active_propname, _index):
        # assert(isinstance(item, bpy.types.MaterialSlot)
        # ob = data
        slot = item
        ma = slot.material

        layout.context_pointer_set("id", ma)
        layout.context_pointer_set("material_slot", slot)

        if ma:
            layout.prop(ma, "name", text="", emboss=False, icon_value=icon)
        else:
            layout.label(text="", icon_value=icon)


class MaterialButtonsPanel:
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "material"
    # COMPAT_ENGINES must be defined in each subclass, external engines can add themselves here

    @classmethod
    def poll(cls, context):
        mat = context.material
        return mat and (context.engine in cls.COMPAT_ENGINES) and not mat.grease_pencil
class MATERIAL_PT_custom_props(MaterialButtonsPanel, PropertyPanel, Panel):
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }
    _context_path = "material"
    _property_type = bpy.types.Material


class EEVEE_MATERIAL_PT_context_material(MaterialButtonsPanel, Panel):
    bl_label = ""
    bl_context = "material"
    bl_options = {'HIDE_HEADER'}
    COMPAT_ENGINES = {
        'BLENDER_PROXIMITY',
    }

    @classmethod
    def poll(cls, context):
        ob = context.object
        mat = context.material

        if mat and mat.grease_pencil:
            return False
        if ob and ob.type == 'GREASEPENCIL':
            return False

        return (
            (ob or mat) and
            (context.engine in cls.COMPAT_ENGINES)
        )

    def draw(self, context):
        layout = self.layout

        mat = context.material
        ob = context.object
        slot = context.material_slot
        space = context.space_data

        if ob:
            is_sortable = len(ob.material_slots) > 1
            rows = 3
            if is_sortable:
                rows = 5

            row = layout.row()

            row.template_list("MATERIAL_UL_matslots", "", ob, "material_slots", ob, "active_material_index", rows=rows)

            col = row.column(align=True)
            col.operator("object.material_slot_add", icon='ADD', text="")
            col.operator("object.material_slot_remove", icon='REMOVE', text="")

            col.separator()

            col.menu("MATERIAL_MT_context_menu", icon='DOWNARROW_HLT', text="")

            if is_sortable:
                col.separator()

                col.operator("object.material_slot_move", icon='TRIA_UP', text="").direction = 'UP'
                col.operator("object.material_slot_move", icon='TRIA_DOWN', text="").direction = 'DOWN'

        row = layout.row()

        if ob:
            row.template_ID(ob, "active_material", new="material.new")

            if slot:
                row.prop(slot, "link", icon_only=True)

            if ob.mode == 'EDIT':
                row = layout.row(align=True)
                row.operator("object.material_slot_assign", text="Assign")
                row.operator("object.material_slot_select", text="Select")
                row.operator("object.material_slot_deselect", text="Deselect")

        elif mat:
            row.template_ID(space, "pin_id")


class MATERIAL_PT_lineart(MaterialButtonsPanel, Panel):
    bl_label = "Line Art"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 10

    @classmethod
    def poll(cls, context):
        mat = context.material
        return mat and not mat.grease_pencil

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        mat = context.material
        lineart = mat.lineart

        layout.prop(lineart, "use_material_mask", text="Material Mask")

        col = layout.column(align=True)
        col.active = lineart.use_material_mask
        row = col.row(align=True, heading="Masks")
        for i in range(8):
            row.prop(lineart, "use_material_mask_bits", text=" ", index=i, toggle=True)
            if i == 3:
                row = col.row(align=True)

        row = layout.row(align=True, heading="Custom Occlusion")
        row.prop(lineart, "mat_occlusion", text="Levels")

        row = layout.row(heading="Intersection Priority")
        row.prop(lineart, "use_intersection_priority_override", text="")
        subrow = row.row()
        subrow.active = lineart.use_intersection_priority_override
        subrow.prop(lineart, "intersection_priority", text="")


class MATERIAL_PT_animation(MaterialButtonsPanel, Panel, PropertiesAnimationMixin):
    COMPAT_ENGINES = {
        'BLENDER_RENDER',
        'BLENDER_PROXIMITY',
    }

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False

        # MaterialButtonsPanel.poll ensures this is not None.
        material = context.material

        col = layout.column(align=True)
        col.label(text="Material")
        self.draw_action_and_slot_selector(context, col, material)

        if node_tree := material.node_tree:
            col = layout.column(align=True)
            col.label(text="Shader Node Tree")
            self.draw_action_and_slot_selector(context, col, node_tree)


class MATERIAL_PT_viewport(MaterialButtonsPanel, Panel):
    bl_label = "Viewport Display"
    bl_context = "material"
    bl_options = {'DEFAULT_CLOSED'}
    bl_order = 10

    @classmethod
    def poll(cls, context):
        mat = context.material
        return mat and not mat.grease_pencil

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True

        mat = context.material

        col = layout.column()
        col.prop(mat, "diffuse_color", text="Color")
        col.prop(mat, "metallic")
        col.prop(mat, "roughness")


classes = (
    MATERIAL_MT_context_menu,
    MATERIAL_UL_matslots,
    EEVEE_MATERIAL_PT_context_material,
    MATERIAL_PT_lineart,
    MATERIAL_PT_viewport,
    MATERIAL_PT_animation,
    MATERIAL_PT_custom_props,
)


if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
