/* SPDX-FileCopyrightText: 2004 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spoutliner
 */

#include <cstring>

#include "MEM_guardedalloc.h"

#include "DNA_collection_types.h"
#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"
#include "DNA_space_types.h"

#include "BLI_listbase.h"

#include "BLT_translation.hh"

#include "BKE_collection.hh"
#include "BKE_context.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_library.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "ED_object.hh"
#include "ED_outliner.hh"
#include "ED_screen.hh"

#include "UI_interface.hh"
#include "UI_view2d.hh"

#include "RNA_access.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "outliner_intern.hh"

namespace blender::ed::outliner {

static Collection *collection_parent_from_ID(ID *id);

/* -------------------------------------------------------------------- */
/** \name Drop Target Find
 * \{ */

static TreeElement *outliner_dropzone_element(TreeElement *te,
                                              const float fmval[2],
                                              const bool children)
{
  if ((fmval[1] > te->ys) && (fmval[1] < (te->ys + UI_UNIT_Y))) {
    /* name and first icon */
    if ((fmval[0] > te->xs + UI_UNIT_X) && (fmval[0] < te->xend)) {
      return te;
    }
  }
  /* Not it.  Let's look at its children. */
  if (children && (TREESTORE(te)->flag & TSE_CLOSED) == 0 && (te->subtree.first)) {
    for (TreeElement &te_sub : te->subtree) {
      TreeElement *te_valid = outliner_dropzone_element(&te_sub, fmval, children);
      if (te_valid) {
        return te_valid;
      }
    }
  }
  return nullptr;
}

/* Find tree element to drop into. */
static TreeElement *outliner_dropzone_find(const SpaceOutliner *space_outliner,
                                           const float fmval[2],
                                           const bool children)
{
  for (TreeElement &te : space_outliner->tree) {
    TreeElement *te_valid = outliner_dropzone_element(&te, fmval, children);
    if (te_valid) {
      return te_valid;
    }
  }
  return nullptr;
}

static TreeElement *outliner_drop_find(bContext *C, const wmEvent *event)
{
  ARegion *region = CTX_wm_region(C);
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);
  float fmval[2];
  ui::view2d_region_to_view(&region->v2d, event->mval[0], event->mval[1], &fmval[0], &fmval[1]);

  return outliner_dropzone_find(space_outliner, fmval, true);
}

static ID *outliner_ID_drop_find(bContext *C, const wmEvent *event, short idcode)
{
  TreeElement *te = outliner_drop_find(C, event);
  TreeStoreElem *tselem = (te) ? TREESTORE(te) : nullptr;

  if (te && (te->idcode == idcode) && (tselem->type == TSE_SOME_ID)) {
    return tselem->id;
  }
  return nullptr;
}

/* Find tree element to drop into, with additional before and after reorder support. */
static TreeElement *outliner_drop_insert_find(bContext *C,
                                              const int xy[2],
                                              TreeElementInsertType *r_insert_type)
{
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);
  ARegion *region = CTX_wm_region(C);
  TreeElement *te_hovered;
  float view_mval[2];

  /* Empty tree, e.g. while filtered. */
  if (BLI_listbase_is_empty(&space_outliner->tree)) {
    return nullptr;
  }

  int mval[2];
  mval[0] = xy[0] - region->winrct.xmin;
  mval[1] = xy[1] - region->winrct.ymin;

  ui::view2d_region_to_view(&region->v2d, mval[0], mval[1], &view_mval[0], &view_mval[1]);
  te_hovered = outliner_find_item_at_y(space_outliner, &space_outliner->tree, view_mval[1]);

  if (te_hovered) {
    /* Mouse hovers an element (ignoring x-axis),
     * now find out how to insert the dragged item exactly. */
    const float margin = UI_UNIT_Y * (1.0f / 4);

    if (view_mval[1] < (te_hovered->ys + margin)) {
      if (TSELEM_OPEN(TREESTORE(te_hovered), space_outliner) &&
          !BLI_listbase_is_empty(&te_hovered->subtree))
      {
        /* inserting after a open item means we insert into it, but as first child */
        if (BLI_listbase_is_empty(&te_hovered->subtree)) {
          *r_insert_type = TE_INSERT_INTO;
          return te_hovered;
        }
        *r_insert_type = TE_INSERT_BEFORE;
        return static_cast<TreeElement *>(te_hovered->subtree.first);
      }
      *r_insert_type = TE_INSERT_AFTER;
      return te_hovered;
    }
    if (view_mval[1] > (te_hovered->ys + (3 * margin))) {
      *r_insert_type = TE_INSERT_BEFORE;
      return te_hovered;
    }
    *r_insert_type = TE_INSERT_INTO;
    return te_hovered;
  }

  /* Mouse doesn't hover any item (ignoring x-axis),
   * so it's either above list bounds or below. */
  TreeElement *first = static_cast<TreeElement *>(space_outliner->tree.first);
  TreeElement *last = static_cast<TreeElement *>(space_outliner->tree.last);

  if (view_mval[1] < last->ys) {
    *r_insert_type = TE_INSERT_AFTER;
    return last;
  }
  if (view_mval[1] > (first->ys + UI_UNIT_Y)) {
    *r_insert_type = TE_INSERT_BEFORE;
    return first;
  }

  BLI_assert_unreachable();
  return nullptr;
}

using CheckTypeFn = bool (*)(TreeElement *te);

static TreeElement *outliner_data_from_tree_element_and_parents(CheckTypeFn check_type,
                                                                TreeElement *te)
{
  while (te != nullptr) {
    if (check_type(te)) {
      return te;
    }
    te = te->parent;
  }
  return nullptr;
}

static bool is_collection_element(TreeElement *te)
{
  return outliner_is_collection_tree_element(te);
}

static bool is_object_element(TreeElement *te)
{
  TreeStoreElem *tselem = TREESTORE(te);
  return (tselem->type == TSE_SOME_ID) && te->idcode == ID_OB;
}

static bool is_pchan_element(TreeElement *te)
{
  TreeStoreElem *tselem = TREESTORE(te);
  return tselem->type == TSE_POSE_CHANNEL;
}

static TreeElement *outliner_drop_insert_collection_find(bContext *C,
                                                         const int xy[2],
                                                         TreeElementInsertType *r_insert_type)
{
  TreeElement *te = outliner_drop_insert_find(C, xy, r_insert_type);
  if (!te) {
    return nullptr;
  }

  TreeElement *collection_te = outliner_data_from_tree_element_and_parents(is_collection_element,
                                                                           te);
  if (!collection_te) {
    return nullptr;
  }

  /* We can't insert before/after/into a collection that itself is selected/dragged. */
  TreeStoreElem *collection_tselem = TREESTORE(collection_te);
  if ((collection_tselem->flag & TSE_SELECTED) != 0) {
    return nullptr;
  }

  Collection *collection = outliner_collection_from_tree_element(collection_te);

  if (collection_te != te) {
    *r_insert_type = TE_INSERT_INTO;
  }

  /* We can't insert before/after master collection. */
  if (collection->flag & COLLECTION_IS_MASTER) {
    *r_insert_type = TE_INSERT_INTO;
  }

  return collection_te;
}

template<typename T>
static int outliner_get_insert_index(TreeElement *drag_te,
                                     TreeElement *drop_te,
                                     TreeElementInsertType insert_type,
                                     ListBaseT<T> *listbase)
{
  /* Find the element to insert after. Null is the start of the list. */
  if (drag_te->index < drop_te->index) {
    if (insert_type == TE_INSERT_BEFORE) {
      drop_te = drop_te->prev;
    }
  }
  else {
    if (insert_type == TE_INSERT_AFTER) {
      drop_te = drop_te->next;
    }
  }

  if (drop_te == nullptr) {
    return 0;
  }

  return BLI_findindex(listbase, drop_te->directdata);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Parent Drop Operator
 * \{ */

static bool parent_drop_allowed(TreeElement *te, Object *potential_child)
{
  TreeStoreElem *tselem = TREESTORE(te);
  if ((te->idcode != ID_OB) || (tselem->type != TSE_SOME_ID)) {
    return false;
  }

  Object *potential_parent = id_cast<Object *>(tselem->id);

  if (potential_parent == potential_child) {
    return false;
  }
  if (BKE_object_is_child_recursive(potential_child, potential_parent)) {
    return false;
  }
  if (potential_parent == potential_child->parent) {
    return false;
  }

  /* check that parent/child are both in the same scene */
  Scene *scene = id_cast<Scene *>(outliner_search_back(te, ID_SCE));

  /* currently outliner organized in a way that if there's no parent scene
   * element for object it means that all displayed objects belong to
   * active scene and parenting them is allowed (sergey) */
  if (scene) {
    for (ViewLayer &view_layer : scene->view_layers) {
      BKE_view_layer_synced_ensure(scene, &view_layer);
      if (BKE_view_layer_base_find(&view_layer, potential_child)) {
        return true;
      }
    }
    return false;
  }
  return true;
}

static bool allow_parenting_without_modifier_key(SpaceOutliner *space_outliner)
{
  switch (space_outliner->outlinevis) {
    case SO_VIEW_LAYER:
      return space_outliner->filter & SO_FILTER_NO_COLLECTION;
    case SO_SCENES:
      return true;
    default:
      return false;
  }
}

static bool parent_drop_poll(bContext *C, wmDrag *drag, const wmEvent *event)
{
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);

  bool changed = outliner_flag_set(*space_outliner, TSE_DRAG_ANY, false);
  if (changed) {
    ED_region_tag_redraw_no_rebuild(CTX_wm_region(C));
  }

  Object *potential_child = id_cast<Object *>(WM_drag_get_local_ID(drag, ID_OB));
  if (!potential_child) {
    return false;
  }

  if (!allow_parenting_without_modifier_key(space_outliner)) {
    if ((event->modifier & KM_SHIFT) == 0) {
      return false;
    }
  }

  TreeElement *te = outliner_drop_find(C, event);
  if (!te) {
    return false;
  }

  if (parent_drop_allowed(te, potential_child)) {
    TREESTORE(te)->flag |= TSE_DRAG_INTO;
    ED_region_tag_redraw_no_rebuild(CTX_wm_region(C));
    return true;
  }

  return false;
}

static void parent_drop_set_parents(bContext *C,
                                    ReportList *reports,
                                    wmDragID *drag,
                                    Object *parent,
                                    short parent_type,
                                    const bool keep_transform)
{
  Main *bmain = CTX_data_main(C);
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);

  TreeElement *te = outliner_find_id(
      space_outliner, &space_outliner->tree, &parent->id, TreeElementFlag(0));
  Scene *scene = id_cast<Scene *>(outliner_search_back(te, ID_SCE));

  if (scene == nullptr) {
    /* currently outliner organized in a way, that if there's no parent scene
     * element for object it means that all displayed objects belong to
     * active scene and parenting them is allowed (sergey)
     */

    scene = CTX_data_scene(C);
  }

  bool parent_set = false;
  bool linked_objects = false;

  for (wmDragID *drag_id = drag; drag_id; drag_id = drag_id->next) {
    if (GS(drag_id->id->name) == ID_OB) {
      Object *object = id_cast<Object *>(drag_id->id);

      /* Do nothing to linked data */
      if (!BKE_id_is_editable(bmain, &object->id)) {
        linked_objects = true;
        continue;
      }

      if (object::parent_set(
              reports, C, scene, object, parent, parent_type, false, keep_transform, nullptr))
      {
        parent_set = true;
      }
    }
  }

  if (linked_objects) {
    BKE_report(reports, RPT_INFO, "Cannot edit library linked or non-editable override object(s)");
  }

  if (parent_set) {
    DEG_relations_tag_update(bmain);
    WM_event_add_notifier(C, NC_OBJECT | ND_TRANSFORM, nullptr);
    WM_event_add_notifier(C, NC_OBJECT | ND_PARENT, nullptr);
  }
}

static wmOperatorStatus parent_drop_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  TreeElement *te = outliner_drop_find(C, event);
  TreeStoreElem *tselem = te ? TREESTORE(te) : nullptr;

  if (!(te && (te->idcode == ID_OB) && (tselem->type == TSE_SOME_ID))) {
    return OPERATOR_CANCELLED;
  }

  Object *par = id_cast<Object *>(tselem->id);
  Object *ob = id_cast<Object *>(WM_drag_get_local_ID_from_event(event, ID_OB));

  if (ELEM(nullptr, ob, par)) {
    return OPERATOR_CANCELLED;
  }
  if (ob == par) {
    return OPERATOR_CANCELLED;
  }

  if (event->custom != EVT_DATA_DRAGDROP) {
    return OPERATOR_CANCELLED;
  }

  ListBaseT<wmDrag> *lb = static_cast<ListBaseT<wmDrag> *>(event->customdata);
  wmDrag *drag = static_cast<wmDrag *>(lb->first);

  parent_drop_set_parents(C,
                          op->reports,
                          static_cast<wmDragID *>(drag->ids.first),
                          par,
                          object::PAR_OBJECT,
                          !(event->modifier & KM_ALT));

  return OPERATOR_FINISHED;
}

void OUTLINER_OT_parent_drop(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Drop to Set Parent (hold Alt to not keep transforms)";
  ot->description = "Drag to parent in Outliner";
  ot->idname = "OUTLINER_OT_parent_drop";

  /* API callbacks. */
  ot->invoke = parent_drop_invoke;

  ot->poll = ED_operator_region_outliner_active;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Parent Clear Operator
 * \{ */

static bool parent_clear_poll(bContext *C, wmDrag *drag, const wmEvent *event)
{
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);

  if (!allow_parenting_without_modifier_key(space_outliner)) {
    if ((event->modifier & KM_SHIFT) == 0) {
      return false;
    }
  }

  Object *ob = id_cast<Object *>(WM_drag_get_local_ID(drag, ID_OB));
  if (!ob) {
    return false;
  }
  if (!ob->parent) {
    return false;
  }

  TreeElement *te = outliner_drop_find(C, event);
  if (te) {
    TreeStoreElem *tselem = TREESTORE(te);
    ID *id = tselem->id;
    if (!id) {
      return true;
    }

    switch (GS(id->name)) {
      case ID_OB:
        return ELEM(tselem->type, TSE_MODIFIER_BASE, TSE_CONSTRAINT_BASE);
      case ID_GR:
        return (event->modifier & KM_SHIFT) || ELEM(tselem->type, TSE_LIBRARY_OVERRIDE_BASE);
      default:
        return true;
    }
  }
  else {
    return true;
  }
}

static wmOperatorStatus parent_clear_invoke(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  Main *bmain = CTX_data_main(C);

  if (event->custom != EVT_DATA_DRAGDROP) {
    return OPERATOR_CANCELLED;
  }

  ListBaseT<wmDrag> *lb = static_cast<ListBaseT<wmDrag> *>(event->customdata);
  wmDrag *drag = static_cast<wmDrag *>(lb->first);

  for (wmDragID &drag_id : drag->ids) {
    if (GS(drag_id.id->name) == ID_OB) {
      Object *object = id_cast<Object *>(drag_id.id);

      object::parent_clear(object,
                           (event->modifier & KM_ALT) ? object::CLEAR_PARENT_ALL :
                                                        object::CLEAR_PARENT_KEEP_TRANSFORM);
    }
  }

  DEG_relations_tag_update(bmain);
  WM_event_add_notifier(C, NC_OBJECT | ND_TRANSFORM, nullptr);
  WM_event_add_notifier(C, NC_OBJECT | ND_PARENT, nullptr);
  return OPERATOR_FINISHED;
}

void OUTLINER_OT_parent_clear(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Drop to Clear Parent (hold Alt to not keep transforms)";
  ot->description = "Drag to clear parent in Outliner";
  ot->idname = "OUTLINER_OT_parent_clear";

  /* API callbacks. */
  ot->invoke = parent_clear_invoke;

  ot->poll = ED_operator_outliner_active;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Scene Drop Operator
 * \{ */

static bool scene_drop_poll(bContext *C, wmDrag *drag, const wmEvent *event)
{
  /* Ensure item under cursor is valid drop target */
  Object *ob = id_cast<Object *>(WM_drag_get_local_ID(drag, ID_OB));
  return (ob && (outliner_ID_drop_find(C, event, ID_SCE) != nullptr));
}

static wmOperatorStatus scene_drop_invoke(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = id_cast<Scene *>(outliner_ID_drop_find(C, event, ID_SCE));
  Object *ob = id_cast<Object *>(WM_drag_get_local_ID_from_event(event, ID_OB));

  if (ELEM(nullptr, ob, scene) || !BKE_id_is_editable(bmain, &scene->id)) {
    return OPERATOR_CANCELLED;
  }

  if (BKE_scene_has_object(scene, ob)) {
    return OPERATOR_CANCELLED;
  }

  Collection *collection;
  if (scene != CTX_data_scene(C)) {
    /* when linking to an inactive scene link to the master collection */
    collection = scene->master_collection;
  }
  else {
    collection = CTX_data_collection(C);
  }

  BKE_collection_object_add(bmain, collection, ob);

  for (ViewLayer &view_layer : scene->view_layers) {
    BKE_view_layer_synced_ensure(scene, &view_layer);
    Base *base = BKE_view_layer_base_find(&view_layer, ob);
    if (base) {
      object::base_select(base, object::BA_SELECT);
    }
  }

  ED_region_tag_redraw(CTX_wm_region(C));
  DEG_relations_tag_update(bmain);

  DEG_id_tag_update(&scene->id, ID_RECALC_SELECT);
  WM_main_add_notifier(NC_SCENE | ND_OB_SELECT, scene);

  return OPERATOR_FINISHED;
}

void OUTLINER_OT_scene_drop(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Drop Object to Scene";
  ot->description = "Drag object to scene in Outliner";
  ot->idname = "OUTLINER_OT_scene_drop";

  /* API callbacks. */
  ot->invoke = scene_drop_invoke;

  ot->poll = ED_operator_region_outliner_active;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Material Drop Operator
 * \{ */

static bool material_drop_poll(bContext *C, wmDrag *drag, const wmEvent *event)
{
  /* Ensure item under cursor is valid drop target */
  Material *ma = id_cast<Material *>(WM_drag_get_local_ID(drag, ID_MA));
  Object *ob = reinterpret_cast<Object *>(outliner_ID_drop_find(C, event, ID_OB));

  return (!ELEM(nullptr, ob, ma) && ID_IS_EDITABLE(&ob->id) && !ID_IS_OVERRIDE_LIBRARY(&ob->id));
}

static wmOperatorStatus material_drop_invoke(bContext *C,
                                             wmOperator * /*op*/,
                                             const wmEvent *event)
{
  Main *bmain = CTX_data_main(C);
  Object *ob = id_cast<Object *>(outliner_ID_drop_find(C, event, ID_OB));
  Material *ma = id_cast<Material *>(WM_drag_get_local_ID_from_event(event, ID_MA));

  if (ELEM(nullptr, ob, ma) || !BKE_id_is_editable(bmain, &ob->id)) {
    return OPERATOR_CANCELLED;
  }

  /* only drop grease pencil material on grease pencil objects */
  if ((ma->gp_style != nullptr) && (ob->type != OB_GREASE_PENCIL)) {
    return OPERATOR_CANCELLED;
  }

  BKE_object_material_assign(bmain, ob, ma, ob->totcol + 1, BKE_MAT_ASSIGN_USERPREF);

  WM_event_add_notifier(C, NC_OBJECT | ND_OB_SHADING, ob);
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_VIEW3D, nullptr);
  WM_event_add_notifier(C, NC_MATERIAL | ND_SHADING_LINKS, ma);

  return OPERATOR_FINISHED;
}

void OUTLINER_OT_material_drop(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Drop Material on Object";
  ot->description = "Drag material to object in Outliner";
  ot->idname = "OUTLINER_OT_material_drop";

  /* API callbacks. */
  ot->invoke = material_drop_invoke;

  ot->poll = ED_operator_region_outliner_active;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Data Stack Drop Operator
 *
 * A generic operator to allow drag and drop for modifiers, constraints,
 * and shader effects which all share the same UI stack layout.
 *
 * The following operations are allowed:
 * - Reordering within an object.
 * - Copying a single modifier/constraint/effect to another object.
 * - Copying (linking) an object's modifiers/constraints/effects to another.
 * \{ */

enum eDataStackDropAction {
  DATA_STACK_DROP_REORDER,
  DATA_STACK_DROP_COPY,
  DATA_STACK_DROP_LINK,
};

struct StackDropData {
  Object *ob_parent;
  bPoseChannel *pchan_parent;
  TreeStoreElem *drag_tselem;
  void *drag_directdata;
  int drag_index;

  eDataStackDropAction drop_action;
  TreeElement *drop_te;
  TreeElementInsertType insert_type;
};

static void datastack_drop_data_init(wmDrag *drag,
                                     Object *ob,
                                     bPoseChannel *pchan,
                                     TreeElement *te,
                                     TreeStoreElem *tselem,
                                     void *directdata)
{
  StackDropData *drop_data = MEM_new_zeroed<StackDropData>("datastack drop data");

  drop_data->ob_parent = ob;
  drop_data->pchan_parent = pchan;
  drop_data->drag_tselem = tselem;
  drop_data->drag_directdata = directdata;
  drop_data->drag_index = te->index;

  drag->poin = drop_data;
  drag->flags |= WM_DRAG_FREE_DATA;
}

static bool datastack_drop_init(bContext *C, const wmEvent *event, StackDropData *drop_data)
{
  if (!ELEM(drop_data->drag_tselem->type,
            TSE_MODIFIER,
            TSE_MODIFIER_BASE,
            TSE_CONSTRAINT,
            TSE_CONSTRAINT_BASE,
            TSE_GPENCIL_EFFECT,
            TSE_GPENCIL_EFFECT_BASE))
  {
    return false;
  }

  TreeElement *te_target = outliner_drop_insert_find(C, event->xy, &drop_data->insert_type);
  if (!te_target) {
    return false;
  }
  TreeStoreElem *tselem_target = TREESTORE(te_target);

  if (drop_data->drag_tselem == tselem_target) {
    return false;
  }

  Object *ob = nullptr;
  TreeElement *object_te = outliner_data_from_tree_element_and_parents(is_object_element,
                                                                       te_target);
  if (object_te) {
    ob = id_cast<Object *>(TREESTORE(object_te)->id);
  }

  bPoseChannel *pchan = nullptr;
  TreeElement *pchan_te = outliner_data_from_tree_element_and_parents(is_pchan_element, te_target);
  if (pchan_te) {
    pchan = static_cast<bPoseChannel *>(pchan_te->directdata);
  }
  if (pchan) {
    ob = nullptr;
  }

  if (ob && !BKE_id_is_editable(CTX_data_main(C), &ob->id)) {
    return false;
  }

  /* Drag a base for linking. */
  if (ELEM(drop_data->drag_tselem->type,
           TSE_MODIFIER_BASE,
           TSE_CONSTRAINT_BASE,
           TSE_GPENCIL_EFFECT_BASE))
  {
    drop_data->insert_type = TE_INSERT_INTO;
    drop_data->drop_action = DATA_STACK_DROP_LINK;

    if (pchan && pchan != drop_data->pchan_parent) {
      drop_data->drop_te = pchan_te;
      tselem_target = TREESTORE(pchan_te);
    }
    else if (ob && ob != drop_data->ob_parent) {
      drop_data->drop_te = object_te;
      tselem_target = TREESTORE(object_te);
    }
    else {
      return false;
    }
  }
  else if (ob || pchan) {
    /* Drag a single item. */
    if (pchan && pchan != drop_data->pchan_parent) {
      drop_data->insert_type = TE_INSERT_INTO;
      drop_data->drop_action = DATA_STACK_DROP_COPY;
      drop_data->drop_te = pchan_te;
      tselem_target = TREESTORE(pchan_te);
    }
    else if (ob && ob != drop_data->ob_parent) {
      drop_data->insert_type = TE_INSERT_INTO;
      drop_data->drop_action = DATA_STACK_DROP_COPY;
      drop_data->drop_te = object_te;
      tselem_target = TREESTORE(object_te);
    }
    else if (tselem_target->type == drop_data->drag_tselem->type) {
      if (drop_data->insert_type == TE_INSERT_INTO) {
        return false;
      }
      drop_data->drop_action = DATA_STACK_DROP_REORDER;
      drop_data->drop_te = te_target;
    }
    else {
      return false;
    }
  }
  else {
    return false;
  }

  return true;
}

/* Ensure that grease pencil and object data remain separate. */
static bool datastack_drop_are_types_valid(StackDropData *drop_data)
{
  TreeStoreElem *tselem = TREESTORE(drop_data->drop_te);
  Object *ob_parent = drop_data->ob_parent;
  Object *ob_dst = id_cast<Object *>(tselem->id);

  /* Don't allow data to be moved between objects and bones. */
  if (tselem->type == TSE_CONSTRAINT) {
  }
  else if ((drop_data->pchan_parent && tselem->type != TSE_POSE_CHANNEL) ||
           (!drop_data->pchan_parent && tselem->type == TSE_POSE_CHANNEL))
  {
    return false;
  }

  switch (drop_data->drag_tselem->type) {
    case TSE_MODIFIER_BASE:
    case TSE_MODIFIER:
      return (ob_parent->type == OB_GREASE_PENCIL) == (ob_dst->type == OB_GREASE_PENCIL);
      break;
    case TSE_CONSTRAINT_BASE:
    case TSE_CONSTRAINT:

      break;
    case TSE_GPENCIL_EFFECT_BASE:
    case TSE_GPENCIL_EFFECT:
      return ob_parent->type == OB_GREASE_PENCIL && ob_dst->type == OB_GREASE_PENCIL;
      break;
  }

  return true;
}

static bool datastack_drop_poll(bContext *C, wmDrag *drag, const wmEvent *event)
{
  if (drag->type != WM_DRAG_DATASTACK) {
    return false;
  }

  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);
  ARegion *region = CTX_wm_region(C);
  bool changed = outliner_flag_set(*space_outliner, TSE_HIGHLIGHTED_ANY | TSE_DRAG_ANY, false);

  StackDropData *drop_data = static_cast<StackDropData *>(drag->poin);
  if (!drop_data) {
    return false;
  }

  if (!datastack_drop_init(C, event, drop_data)) {
    return false;
  }

  if (!datastack_drop_are_types_valid(drop_data)) {
    return false;
  }

  TreeStoreElem *tselem_target = TREESTORE(drop_data->drop_te);
  switch (drop_data->insert_type) {
    case TE_INSERT_BEFORE:
      tselem_target->flag |= TSE_DRAG_BEFORE;
      break;
    case TE_INSERT_AFTER:
      tselem_target->flag |= TSE_DRAG_AFTER;
      break;
    case TE_INSERT_INTO:
      tselem_target->flag |= TSE_DRAG_INTO;
      break;
  }

  if (changed) {
    ED_region_tag_redraw_no_rebuild(region);
  }

  return true;
}

static std::string datastack_drop_tooltip(bContext * /*C*/,
                                          wmDrag *drag,
                                          const int /*xy*/[2],
                                          wmDropBox * /*drop*/)
{
  StackDropData *drop_data = static_cast<StackDropData *>(drag->poin);
  switch (drop_data->drop_action) {
    case DATA_STACK_DROP_REORDER:
      return TIP_("Reorder");
    case DATA_STACK_DROP_COPY:
      if (drop_data->pchan_parent) {
        return TIP_("Copy to bone");
      }
      return TIP_("Copy to object");

    case DATA_STACK_DROP_LINK:
      if (drop_data->pchan_parent) {
        return TIP_("Link all to bone");
      }
      return TIP_("Link all to object");
  }
  return {};
}

static void datastack_drop_link(bContext *C, StackDropData *drop_data)
{
  Main *bmain = CTX_data_main(C);
  TreeStoreElem *tselem = TREESTORE(drop_data->drop_te);
  Object *ob_dst = id_cast<Object *>(tselem->id);

  switch (drop_data->drag_tselem->type) {
    case TSE_MODIFIER_BASE:
      object::modifier_link(C, ob_dst, drop_data->ob_parent);
      break;
    case TSE_CONSTRAINT_BASE: {
      ListBaseT<bConstraint> *src;

      if (drop_data->pchan_parent) {
        src = &drop_data->pchan_parent->constraints;
      }
      else {
        src = &drop_data->ob_parent->constraints;
      }

      ListBaseT<bConstraint> *dst;
      if (tselem->type == TSE_POSE_CHANNEL) {
        bPoseChannel *pchan = static_cast<bPoseChannel *>(drop_data->drop_te->directdata);
        dst = &pchan->constraints;
      }
      else {
        dst = &ob_dst->constraints;
      }

      object::constraint_link(bmain, ob_dst, dst, src);
      break;
    }
    case TSE_GPENCIL_EFFECT_BASE:
      if (ob_dst->type != OB_GREASE_PENCIL) {
        return;
      }

      object::shaderfx_link(ob_dst, drop_data->ob_parent);
      break;
  }
}

static void datastack_drop_copy(bContext *C, StackDropData *drop_data)
{
  Main *bmain = CTX_data_main(C);

  TreeStoreElem *tselem = TREESTORE(drop_data->drop_te);
  Object *ob_dst = id_cast<Object *>(tselem->id);

  switch (drop_data->drag_tselem->type) {
    case TSE_MODIFIER:
      object::modifier_copy_to_object(
          bmain,
          CTX_data_scene(C),
          drop_data->ob_parent,
          static_cast<const ModifierData *>(drop_data->drag_directdata),
          ob_dst,
          CTX_wm_reports(C));
      break;
    case TSE_CONSTRAINT:
      if (tselem->type == TSE_POSE_CHANNEL) {
        object::constraint_copy_for_pose(
            bmain,
            ob_dst,
            static_cast<bPoseChannel *>(drop_data->drop_te->directdata),
            static_cast<bConstraint *>(drop_data->drag_directdata));
      }
      else {
        object::constraint_copy_for_object(
            bmain, ob_dst, static_cast<bConstraint *>(drop_data->drag_directdata));
      }
      break;
    case TSE_GPENCIL_EFFECT: {
      if (ob_dst->type != OB_GREASE_PENCIL) {
        return;
      }

      object::shaderfx_copy(ob_dst, static_cast<ShaderFxData *>(drop_data->drag_directdata));
      break;
    }
  }
}

static void datastack_drop_reorder(bContext *C, ReportList *reports, StackDropData *drop_data)
{
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);

  TreeElement *drag_te = outliner_find_tree_element(&space_outliner->tree, drop_data->drag_tselem);
  if (!drag_te) {
    return;
  }

  TreeElement *drop_te = drop_data->drop_te;
  TreeElementInsertType insert_type = drop_data->insert_type;

  Object *ob = drop_data->ob_parent;

  int index = 0;
  switch (drop_data->drag_tselem->type) {
    case TSE_MODIFIER:
      index = outliner_get_insert_index(drag_te, drop_te, insert_type, &ob->modifiers);
      object::modifier_move_to_index(reports,
                                     RPT_WARNING,
                                     ob,
                                     static_cast<ModifierData *>(drop_data->drag_directdata),
                                     index,
                                     true);
      break;
    case TSE_CONSTRAINT:
      if (drop_data->pchan_parent) {
        index = outliner_get_insert_index(
            drag_te, drop_te, insert_type, &drop_data->pchan_parent->constraints);
      }
      else {
        index = outliner_get_insert_index(drag_te, drop_te, insert_type, &ob->constraints);
      }
      object::constraint_move_to_index(
          ob, static_cast<bConstraint *>(drop_data->drag_directdata), index);

      break;
    case TSE_GPENCIL_EFFECT:
      index = outliner_get_insert_index(drag_te, drop_te, insert_type, &ob->shader_fx);
      object::shaderfx_move_to_index(
          reports, ob, static_cast<ShaderFxData *>(drop_data->drag_directdata), index);
  }
}

static wmOperatorStatus datastack_drop_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  if (event->custom != EVT_DATA_DRAGDROP) {
    return OPERATOR_CANCELLED;
  }

  ListBaseT<wmDrag> *lb = static_cast<ListBaseT<wmDrag> *>(event->customdata);
  wmDrag *drag = static_cast<wmDrag *>(lb->first);
  StackDropData *drop_data = static_cast<StackDropData *>(drag->poin);

  switch (drop_data->drop_action) {
    case DATA_STACK_DROP_LINK:
      datastack_drop_link(C, drop_data);
      break;
    case DATA_STACK_DROP_COPY:
      datastack_drop_copy(C, drop_data);
      break;
    case DATA_STACK_DROP_REORDER:
      datastack_drop_reorder(C, op->reports, drop_data);
      break;
  }

  return OPERATOR_FINISHED;
}

void OUTLINER_OT_datastack_drop(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Data Stack Drop";
  ot->description = "Copy or reorder modifiers, constraints, and effects";
  ot->idname = "OUTLINER_OT_datastack_drop";

  /* API callbacks. */
  ot->invoke = datastack_drop_invoke;

  ot->poll = ED_operator_outliner_active;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Collection Drop Operator
 * \{ */

struct CollectionDrop {
  Collection *from;
  Collection *to;

  TreeElement *te;
  TreeElementInsertType insert_type;
};

static Collection *collection_parent_from_ID(ID *id)
{
  /* Can't change linked or override parent collections. */
  if (!id || !ID_IS_EDITABLE(id) || ID_IS_OVERRIDE_LIBRARY(id)) {
    return nullptr;
  }

  /* Also support dropping into/from scene collection. */
  if (GS(id->name) == ID_SCE) {
    return (id_cast<Scene *>(id))->master_collection;
  }
  if (GS(id->name) == ID_GR) {
    return id_cast<Collection *>(id);
  }

  return nullptr;
}

static bool collection_drop_init(bContext *C, wmDrag *drag, const int xy[2], CollectionDrop *data)
{
  /* Get collection to drop into. */
  TreeElementInsertType insert_type;
  TreeElement *te = outliner_drop_insert_collection_find(C, xy, &insert_type);
  if (!te) {
    return false;
  }

  Collection *to_collection = outliner_collection_from_tree_element(te);
  if (!ID_IS_EDITABLE(to_collection) || ID_IS_OVERRIDE_LIBRARY(to_collection)) {
    if (insert_type == TE_INSERT_INTO) {
      return false;
    }
  }

  /* Get drag datablocks. */
  if (drag->type != WM_DRAG_ID) {
    return false;
  }

  wmDragID *drag_id = static_cast<wmDragID *>(drag->ids.first);
  if (drag_id == nullptr) {
    return false;
  }

  ID *id = drag_id->id;
  if (!(id && ELEM(GS(id->name), ID_GR, ID_OB))) {
    return false;
  }

  /* SDF objects belong in SDF Groups, not collections. */
  if (GS(id->name) == ID_OB) {
    Object *ob = (Object *)id;
    if (ob->type == OB_SDF) {
      return false;
    }
  }

  /* Get collection to drag out of. */
  ID *parent = drag_id->from_parent;
  Collection *from_collection = collection_parent_from_ID(parent);

  /* Currently this should not be allowed, cannot edit items in an override of a Collection. */
  if (from_collection != nullptr && ID_IS_OVERRIDE_LIBRARY(from_collection)) {
    return false;
  }

  /* Get collections. */
  if (GS(id->name) == ID_GR) {
    if (id == &to_collection->id) {
      return false;
    }
  }
  else {
    insert_type = TE_INSERT_INTO;
  }

  /* Currently this should not be allowed, cannot edit items in an override of a Collection. */
  if (ID_IS_OVERRIDE_LIBRARY(to_collection) &&
      !ELEM(insert_type, TE_INSERT_AFTER, TE_INSERT_BEFORE))
  {
    return false;
  }

  data->from = from_collection;
  data->to = to_collection;
  data->te = te;
  data->insert_type = insert_type;

  return true;
}

static bool collection_drop_poll(bContext *C, wmDrag *drag, const wmEvent *event)
{
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);
  ARegion *region = CTX_wm_region(C);
  bool changed = outliner_flag_set(*space_outliner, TSE_HIGHLIGHTED_ANY | TSE_DRAG_ANY, false);

  CollectionDrop data;
  if (((event->modifier & KM_SHIFT) == 0) && collection_drop_init(C, drag, event->xy, &data)) {
    TreeElement *te = data.te;
    TreeStoreElem *tselem = TREESTORE(te);
    switch (data.insert_type) {
      case TE_INSERT_BEFORE:
        tselem->flag |= TSE_DRAG_BEFORE;
        changed = true;
        break;
      case TE_INSERT_AFTER:
        tselem->flag |= TSE_DRAG_AFTER;
        changed = true;
        break;
      case TE_INSERT_INTO: {
        tselem->flag |= TSE_DRAG_INTO;
        changed = true;
        break;
      }
    }
    if (changed) {
      ED_region_tag_redraw_no_rebuild(region);
    }
    return true;
  }
  if (changed) {
    ED_region_tag_redraw_no_rebuild(region);
  }
  return false;
}

static std::string collection_drop_tooltip(bContext *C,
                                           wmDrag *drag,
                                           const int xy[2],
                                           wmDropBox * /*drop*/)
{
  wmWindow *win = CTX_wm_window(C);
  const wmEvent *event = win ? win->runtime->eventstate : nullptr;

  CollectionDrop data;
  if (event && ((event->modifier & KM_SHIFT) == 0) && collection_drop_init(C, drag, xy, &data)) {
    const bool is_link = !data.from || (event->modifier & KM_CTRL);

    /* Test if we are moving within same parent collection. */
    bool same_level = false;
    for (CollectionParent &parent : data.to->runtime->parents) {
      if (data.from == parent.collection) {
        same_level = true;
      }
    }

    /* Tooltips when not moving directly into another collection i.e. mouse on border of
     * collections. Later we will decide which tooltip to return. */
    const bool tooltip_link = (is_link && !same_level);
    const char *tooltip_before = tooltip_link ? TIP_("Link before collection") :
                                                TIP_("Move before collection");
    const char *tooltip_between = tooltip_link ? TIP_("Link between collections") :
                                                 TIP_("Move between collections");
    const char *tooltip_after = tooltip_link ? TIP_("Link after collection") :
                                               TIP_("Move after collection");

    TreeElement *te = data.te;
    switch (data.insert_type) {
      case TE_INSERT_BEFORE:
        if (te->prev && outliner_is_collection_tree_element(te->prev)) {
          return tooltip_between;
        }
        return tooltip_before;
      case TE_INSERT_AFTER:
        if (te->next && outliner_is_collection_tree_element(te->next)) {
          return tooltip_between;
        }
        return tooltip_after;
      case TE_INSERT_INTO: {
        if (is_link) {
          return TIP_("Link inside collection");
        }

        /* Check the type of the drag IDs to avoid the incorrect "Shift to parent"
         * for collections. Checking the type of the first ID works fine here since
         * all drag IDs are the same type. */
        wmDragID *drag_id = static_cast<wmDragID *>(drag->ids.first);
        const bool is_object = (GS(drag_id->id->name) == ID_OB);
        if (is_object) {
          return TIP_("Move inside collection (Ctrl to link, Shift to parent)");
        }
        return TIP_("Move inside collection (Ctrl to link)");
      }
    }
  }
  return {};
}

static wmOperatorStatus collection_drop_invoke(bContext *C,
                                               wmOperator * /*op*/,
                                               const wmEvent *event)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);

  if (event->custom != EVT_DATA_DRAGDROP) {
    return OPERATOR_CANCELLED;
  }

  ListBaseT<wmDrag> *lb = static_cast<ListBaseT<wmDrag> *>(event->customdata);
  wmDrag *drag = static_cast<wmDrag *>(lb->first);

  CollectionDrop data;
  if (!collection_drop_init(C, drag, event->xy, &data)) {
    return OPERATOR_CANCELLED;
  }

  /* Before/after insert handling. */
  Collection *relative = nullptr;
  bool relative_after = false;

  if (ELEM(data.insert_type, TE_INSERT_BEFORE, TE_INSERT_AFTER)) {
    SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);

    relative = data.to;
    relative_after = (data.insert_type == TE_INSERT_AFTER);

    TreeElement *parent_te = outliner_find_parent_element(&space_outliner->tree, nullptr, data.te);
    data.to = (parent_te) ? outliner_collection_from_tree_element(parent_te) : nullptr;
  }

  if (!data.to) {
    return OPERATOR_CANCELLED;
  }

  if (BKE_collection_is_empty(data.to)) {
    TREESTORE(data.te)->flag &= ~TSE_CLOSED;
  }

  if (relative_after) {
    BLI_listbase_reverse(&drag->ids);
  }

  for (wmDragID &drag_id : drag->ids) {
    /* Ctrl enables linking, so we don't need a from collection then. */
    Collection *from = (event->modifier & KM_CTRL) ?
                           nullptr :
                           collection_parent_from_ID(drag_id.from_parent);

    if (GS(drag_id.id->name) == ID_OB) {
      /* Move/link object into collection. */
      Object *object = id_cast<Object *>(drag_id.id);

      if (from) {
        BKE_collection_object_move(bmain, scene, data.to, from, object);
      }
      else {
        BKE_collection_object_add(bmain, data.to, object);
      }
    }
    else if (GS(drag_id.id->name) == ID_GR) {
      /* Move/link collection into collection. */
      Collection *collection = id_cast<Collection *>(drag_id.id);

      if (collection != from) {
        BKE_collection_move(bmain, data.to, from, relative, relative_after, collection);
      }
    }

    if (from) {
      DEG_id_tag_update(&from->id,
                        ID_RECALC_SYNC_TO_EVAL | ID_RECALC_GEOMETRY | ID_RECALC_HIERARCHY);
    }
  }

  /* Update dependency graph. */
  DEG_id_tag_update(&data.to->id, ID_RECALC_SYNC_TO_EVAL | ID_RECALC_HIERARCHY);
  DEG_relations_tag_update(bmain);
  /* NOTE: It is possible to drag-and-drop between different windows, which means that the source
   * window/Outliner may also need to be updated. So do not pass the current window in this
   * notifier (unless there is a way to get the drag source window as well?). */
  WM_event_add_notifier_ex(CTX_wm_manager(C), nullptr, NC_SCENE | ND_LAYER, nullptr);

  return OPERATOR_FINISHED;
}

void OUTLINER_OT_collection_drop(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Move to Collection";
  ot->description = "Drag to move to collection in Outliner";
  ot->idname = "OUTLINER_OT_collection_drop";

  /* API callbacks. */
  ot->invoke = collection_drop_invoke;
  ot->poll = ED_operator_outliner_active;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Outliner Drag Operator
 * \{ */

#define OUTLINER_DRAG_SCOLL_OUTSIDE_PAD 7 /* In UI units */

static TreeElement *outliner_item_drag_element_find(SpaceOutliner *space_outliner,
                                                    ARegion *region,
                                                    const wmEvent *event)
{
  /* NOTE: using click-drag events to trigger dragging is fine,
   * it sends coordinates from where dragging was started */
  int mval[2];
  WM_event_drag_start_mval(event, region, mval);

  const float my = ui::view2d_region_to_view_y(&region->v2d, mval[1]);
  return outliner_find_item_at_y(space_outliner, &space_outliner->tree, my);
}

static wmOperatorStatus outliner_item_drag_drop_invoke(bContext *C,
                                                       wmOperator * /*op*/,
                                                       const wmEvent *event)
{
  ARegion *region = CTX_wm_region(C);
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);
  TreeElement *te = outliner_item_drag_element_find(space_outliner, region, event);

  int mval[2];
  WM_event_drag_start_mval(event, region, mval);

  if (!te) {
    return (OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH);
  }

  TreeStoreElem *tselem = TREESTORE(te);
  TreeElementIcon data = tree_element_get_icon(tselem, te);
  if (!data.drag_id) {
    return (OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH);
  }

  float view_mval[2];
  ui::view2d_region_to_view(&region->v2d, mval[0], mval[1], &view_mval[0], &view_mval[1]);
  if (outliner_item_is_co_within_close_toggle(te, view_mval[0])) {
    return (OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH);
  }
  if (outliner_is_co_within_mode_column(space_outliner, view_mval)) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  /* Scroll the view when dragging near edges, but not
   * when the drag goes too far outside the region. */
  {
    wmOperatorType *ot = WM_operatortype_find("VIEW2D_OT_edge_pan", true);
    PointerRNA op_ptr = WM_operator_properties_create_ptr(ot);
    RNA_float_set(&op_ptr, "outside_padding", OUTLINER_DRAG_SCOLL_OUTSIDE_PAD);
    WM_operator_name_call_ptr(C, ot, wm::OpCallContext::InvokeDefault, &op_ptr, event);
    WM_operator_properties_free(&op_ptr);
  }

  const bool use_datastack_drag = ELEM(tselem->type,
                                       TSE_MODIFIER,
                                       TSE_MODIFIER_BASE,
                                       TSE_CONSTRAINT,
                                       TSE_CONSTRAINT_BASE,
                                       TSE_GPENCIL_EFFECT,
                                       TSE_GPENCIL_EFFECT_BASE);

  const eWM_DragDataType wm_drag_type = use_datastack_drag ? WM_DRAG_DATASTACK : WM_DRAG_ID;
  wmDrag *drag = WM_drag_data_create(C, data.icon, wm_drag_type, nullptr, WM_DRAG_NOP);

  if (use_datastack_drag) {
    TreeElement *te_bone = nullptr;
    bPoseChannel *pchan = outliner_find_parent_bone(te, &te_bone);
    datastack_drop_data_init(
        drag, id_cast<Object *>(tselem->id), pchan, te, tselem, te->directdata);
  }
  else if (ELEM(GS(data.drag_id->name), ID_OB, ID_GR)) {
    /* For collections and objects we cheat and drag all selected. */

    /* Only drag element under mouse if it was not selected before. */
    if ((tselem->flag & TSE_SELECTED) == 0) {
      outliner_flag_set(*space_outliner, TSE_SELECTED, 0);
      tselem->flag |= TSE_SELECTED;
    }

    /* Gather all selected elements. */
    IDsSelectedData selected{};

    if (GS(data.drag_id->name) == ID_OB) {
      outliner_tree_traverse(space_outliner,
                             &space_outliner->tree,
                             0,
                             TSE_SELECTED,
                             outliner_collect_selected_objects,
                             &selected);
    }
    else {
      outliner_tree_traverse(space_outliner,
                             &space_outliner->tree,
                             0,
                             TSE_SELECTED,
                             outliner_collect_selected_collections,
                             &selected);
    }

    for (LinkData &link : selected.selected_array) {
      TreeElement *te_selected = static_cast<TreeElement *>(link.data);
      ID *id;

      if (GS(data.drag_id->name) == ID_OB) {
        id = TREESTORE(te_selected)->id;
      }
      else {
        /* Keep collection hierarchies intact when dragging. */
        bool parent_selected = false;
        for (TreeElement *te_parent = te_selected->parent; te_parent;
             te_parent = te_parent->parent)
        {
          if (outliner_is_collection_tree_element(te_parent)) {
            if (TREESTORE(te_parent)->flag & TSE_SELECTED) {
              parent_selected = true;
              break;
            }
          }
        }

        if (parent_selected) {
          continue;
        }

        id = &outliner_collection_from_tree_element(te_selected)->id;
      }

      /* Find parent collection. SDF objects may live under SDF Groups
       * which are not collection tree elements, so fall back to the
       * scene master collection when no collection parent is found. */
      Collection *parent = nullptr;

      if (te_selected->parent) {
        for (TreeElement *te_parent = te_selected->parent; te_parent;
             te_parent = te_parent->parent)
        {
          if (outliner_is_collection_tree_element(te_parent)) {
            parent = outliner_collection_from_tree_element(te_parent);
            break;
          }
        }
      }

      if (!parent) {
        Scene *scene = CTX_data_scene(C);
        parent = scene->master_collection;
      }

      WM_drag_add_local_ID(drag, id, &parent->id);
    }

    BLI_freelistN(&selected.selected_array);
  }
  else {
    /* Add single ID. */
    WM_drag_add_local_ID(drag, data.drag_id, data.drag_parent);
  }

  WM_event_start_prepared_drag(C, drag);

  ED_outliner_select_sync_from_outliner(C, space_outliner);

  return (OPERATOR_FINISHED | OPERATOR_PASS_THROUGH);
}

/* Outliner drag and drop. This operator mostly exists to support dragging
 * from outliner text instead of only from the icon, and also to show a
 * hint in the status-bar key-map. */

void OUTLINER_OT_item_drag_drop(wmOperatorType *ot)
{
  ot->name = "Drag and Drop";
  ot->idname = "OUTLINER_OT_item_drag_drop";
  ot->description = "Drag and drop element to another place";

  ot->invoke = outliner_item_drag_drop_invoke;
  ot->poll = ED_operator_outliner_active;
}

#undef OUTLINER_DRAG_SCOLL_OUTSIDE_PAD

/** \} */

/* -------------------------------------------------------------------- */
/** \name SDF Group Drop Operator
 * \{ */

struct SDFDropTarget {
  Object *group_empty;
  Object *target_ob;
  TreeElementInsertType insert_type;
  bool create_group;
};

static bool is_sdf_group_empty(Object *ob)
{
  if (!ob || ob->type != OB_SDF || !ob->data) {
    return false;
  }
  const SDF *sdf = reinterpret_cast<const SDF *>(ob->data);
  return sdf->sdf_type == SDF_TYPE_GROUP;
}

static SDFDropTarget sdf_group_drop_find(bContext *C, const int xy[2])
{
  SDFDropTarget result = {nullptr, nullptr, TE_INSERT_INTO, false};
  TreeElementInsertType insert_type = TE_INSERT_INTO;

  TreeElement *te = outliner_drop_insert_find(C, xy, &insert_type);
  if (!te) {
    return result;
  }

  TreeStoreElem *tselem = TREESTORE(te);

  /* Drop on non-SDF element (e.g. Scene Collection) — ungroup target */
  if (!tselem->id || GS(tselem->id->name) != ID_OB ||
      ((Object *)tselem->id)->type != OB_SDF)
  {
    result.insert_type = TE_INSERT_AFTER;
    return result;
  }

  Object *ob = (Object *)tselem->id;

  /* Drop on a group empty */
  if (is_sdf_group_empty(ob)) {
    if (insert_type == TE_INSERT_INTO) {
      result.group_empty = ob;
      result.insert_type = insert_type;
    }
    else {
      /* BEFORE/AFTER on a group empty = reorder groups */
      result.target_ob = ob;
      result.insert_type = insert_type;
    }
    return result;
  }

  /* Drop on an SDF that is a child of a group empty */
  if (ob->parent && is_sdf_group_empty(ob->parent)) {
    result.group_empty = ob->parent;
    result.target_ob = ob;
    result.insert_type = insert_type;
    return result;
  }

  /* Drop on an ungrouped SDF — will create a new group */
  if (insert_type == TE_INSERT_INTO) {
    result.target_ob = ob;
    result.create_group = true;
    result.insert_type = TE_INSERT_INTO;
    return result;
  }

  /* BEFORE/AFTER on ungrouped SDF — reorder only (no grouping) */
  result.target_ob = ob;
  result.insert_type = insert_type;
  return result;
}

static bool sdf_group_drop_poll(bContext *C, wmDrag *drag, const wmEvent *event)
{
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);
  ARegion *region = CTX_wm_region(C);
  bool changed = outliner_flag_set(*space_outliner, TSE_HIGHLIGHTED_ANY | TSE_DRAG_ANY, false);

  if (drag->type != WM_DRAG_ID) {
    if (changed) {
      ED_region_tag_redraw_no_rebuild(region);
    }
    return false;
  }

  wmDragID *drag_id = static_cast<wmDragID *>(drag->ids.first);
  if (!drag_id || !drag_id->id || GS(drag_id->id->name) != ID_OB) {
    if (changed) {
      ED_region_tag_redraw_no_rebuild(region);
    }
    return false;
  }
  Object *ob = (Object *)drag_id->id;
  if (ob->type != OB_SDF) {
    if (changed) {
      ED_region_tag_redraw_no_rebuild(region);
    }
    return false;
  }

  SDFDropTarget target = sdf_group_drop_find(C, event->xy);

  /* Allow drop on empty space if dragging a group child (ungroup) */
  bool is_ungroup = (!target.group_empty && !target.target_ob &&
                     ob->parent && is_sdf_group_empty(ob->parent));
  if (!target.group_empty && !target.target_ob && !is_ungroup) {
    if (changed) {
      ED_region_tag_redraw_no_rebuild(region);
    }
    return false;
  }

  /* Can't drop on self */
  if (target.target_ob == ob) {
    if (changed) {
      ED_region_tag_redraw_no_rebuild(region);
    }
    return false;
  }

  /* Group empties can only reorder, not create or join groups */
  if (is_sdf_group_empty(ob)) {
    if (target.create_group || target.group_empty) {
      if (changed) {
        ED_region_tag_redraw_no_rebuild(region);
      }
      return false;
    }
  }

  /* Same-group siblings: INTO means reorder (place after target) */
  if (target.group_empty && target.target_ob &&
      target.insert_type == TE_INSERT_INTO &&
      ob->parent == target.group_empty)
  {
    target.insert_type = TE_INSERT_AFTER;
  }

  /* Highlight */
  TreeElementInsertType highlight_type = TE_INSERT_INTO;
  TreeElement *te = outliner_drop_insert_find(C, event->xy, &highlight_type);
  if (te) {
    TreeStoreElem *tselem = TREESTORE(te);
    if (highlight_type == TE_INSERT_BEFORE) {
      tselem->flag |= TSE_DRAG_BEFORE;
    }
    else if (highlight_type == TE_INSERT_AFTER) {
      tselem->flag |= TSE_DRAG_AFTER;
    }
    else {
      tselem->flag |= TSE_DRAG_INTO;
    }
    changed = true;
  }

  if (changed) {
    ED_region_tag_redraw_no_rebuild(region);
  }
  return true;
}

static std::string sdf_group_drop_tooltip(bContext *C,
                                           wmDrag * /*drag*/,
                                           const int xy[2],
                                           wmDropBox * /*drop*/)
{
  SDFDropTarget target = sdf_group_drop_find(C, xy);
  if (target.create_group) {
    return TIP_("Create SDF Group");
  }
  if (target.group_empty && target.target_ob) {
    return TIP_("Reorder in SDF Group");
  }
  if (target.group_empty) {
    return TIP_("Add to SDF Group");
  }
  if (target.target_ob) {
    return TIP_("Reorder SDF");
  }
  return {};
}

static void sdf_reindex_siblings(Scene *scene,
                                 ViewLayer *view_layer,
                                 Object *drag_ob,
                                 Object *target_ob,
                                 bool before,
                                 Object *parent_filter)
{
  BKE_view_layer_synced_ensure(scene, view_layer);
  Vector<Object *> siblings;
  for (Base &base : *BKE_view_layer_object_bases_get(view_layer)) {
    Object *ob = base.object;
    if (ob->type != OB_SDF || !ob->data) {
      continue;
    }
    const SDF *sdf = reinterpret_cast<const SDF *>(ob->data);
    if (parent_filter) {
      /* Inside a group: children of this group (not sub-groups) */
      if (ob->parent != parent_filter) continue;
      if (sdf->sdf_type == SDF_TYPE_GROUP) continue;
    }
    else {
      /* Top level: groups + ungrouped SDFs (not children of groups) */
      bool is_grouped_child = ob->parent && is_sdf_group_empty(ob->parent);
      if (is_grouped_child) continue;
    }
    siblings.append(ob);
  }

  std::sort(siblings.begin(), siblings.end(), [](Object *a, Object *b) {
    const SDF *sa = reinterpret_cast<const SDF *>(a->data);
    const SDF *sb = reinterpret_cast<const SDF *>(b->data);
    return sa->sdf_index < sb->sdf_index;
  });

  /* Remove drag from list */
  int drag_pos = -1;
  for (int i = 0; i < int(siblings.size()); i++) {
    if (siblings[i] == drag_ob) { drag_pos = i; break; }
  }
  if (drag_pos >= 0) {
    siblings.remove(drag_pos);
  }

  /* Find target position */
  int insert_pos = int(siblings.size());
  for (int i = 0; i < int(siblings.size()); i++) {
    if (siblings[i] == target_ob) {
      insert_pos = before ? i : i + 1;
      break;
    }
  }

  siblings.insert(insert_pos, drag_ob);

  /* Re-index sequentially with globally unique indices.
   * Find max sdf_index in scene to avoid collisions. */
  int max_idx = 0;
  for (Base &base : *BKE_view_layer_object_bases_get(view_layer)) {
    if (base.object->type == OB_SDF && base.object->data) {
      const SDF *s = reinterpret_cast<const SDF *>(base.object->data);
      /* Only count objects NOT in this sibling set */
      bool in_set = false;
      for (Object *sib : siblings) {
        if (sib == base.object) { in_set = true; break; }
      }
      if (!in_set) {
        max_idx = std::max(max_idx, s->sdf_index + 1);
      }
    }
  }

  for (int i = 0; i < int(siblings.size()); i++) {
    SDF *sdf = reinterpret_cast<SDF *>(siblings[i]->data);
    sdf->sdf_index = max_idx + i;
    DEG_id_tag_update(&sdf->id, ID_RECALC_GEOMETRY);
    DEG_id_tag_update(&siblings[i]->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY);
  }
}

static wmOperatorStatus sdf_group_drop_invoke(bContext *C,
                                               wmOperator * /*op*/,
                                               const wmEvent *event)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);

  if (event->custom != EVT_DATA_DRAGDROP) {
    return OPERATOR_CANCELLED;
  }

  ListBaseT<wmDrag> *lb = static_cast<ListBaseT<wmDrag> *>(event->customdata);
  wmDrag *drag = static_cast<wmDrag *>(lb->first);

  SDFDropTarget target = sdf_group_drop_find(C, event->xy);
  /* Allow ungroup (both null = drop on non-SDF area) */
  {
    wmDragID *chk = static_cast<wmDragID *>(drag->ids.first);
    bool any_grouped = false;
    if (chk && chk->id && GS(chk->id->name) == ID_OB) {
      Object *chk_ob = (Object *)chk->id;
      if (chk_ob->parent && is_sdf_group_empty(chk_ob->parent)) {
        any_grouped = true;
      }
    }
    if (!target.group_empty && !target.target_ob && !any_grouped) {
      return OPERATOR_CANCELLED;
    }
  }

  /* Pre-scan: get the dragged object to fix same-group INTO */
  wmDragID *first_drag = static_cast<wmDragID *>(drag->ids.first);
  if (first_drag && first_drag->id && GS(first_drag->id->name) == ID_OB) {
    Object *drag_ob = (Object *)first_drag->id;
    if (target.group_empty && target.target_ob &&
        target.insert_type == TE_INSERT_INTO &&
        drag_ob->parent == target.group_empty)
    {
      target.insert_type = TE_INSERT_AFTER;
    }
  }

  for (wmDragID &drag_id : drag->ids) {
    if (!drag_id.id || GS(drag_id.id->name) != ID_OB) {
      continue;
    }

    Object *ob = (Object *)drag_id.id;
    if (ob->type != OB_SDF || !ob->data) {
      continue;
    }

    /* Group empties can only reorder, skip everything else */
    if (is_sdf_group_empty(ob)) {
      if (target.target_ob && !target.create_group &&
          (target.insert_type == TE_INSERT_BEFORE || target.insert_type == TE_INSERT_AFTER))
      {
        printf("  -> calling sdf_reindex_siblings\n");
        sdf_reindex_siblings(scene, view_layer, ob, target.target_ob,
                             target.insert_type == TE_INSERT_BEFORE, nullptr);
      }
      else {
        printf("  -> SKIPPED (conditions not met)\n");
      }
      continue;
    }

    /* Create new group from two ungrouped SDFs */
    if (target.create_group && target.target_ob) {
      SDF *grp_sdf = reinterpret_cast<SDF *>(BKE_id_new(bmain, ID_SF, "SDF Group"));
      grp_sdf->sdf_type = SDF_TYPE_GROUP;
      grp_sdf->csg_operation = SDF_CSG_UNION;
      grp_sdf->blend_type = 0;
      grp_sdf->blend = 0.0f;

      /* Group takes the target's position; children get 0, 1 */
      SDF *target_sdf = reinterpret_cast<SDF *>(target.target_ob->data);
      SDF *drag_sdf = reinterpret_cast<SDF *>(ob->data);
      grp_sdf->sdf_index = target_sdf->sdf_index;
      target_sdf->sdf_index = 0;
      drag_sdf->sdf_index = 1;

      Object *grp_ob = BKE_object_add_only_object(bmain, OB_SDF, "SDF Group");
      grp_ob->data = &grp_sdf->id;
      id_us_plus(&grp_sdf->id);

      /* Group origin = bbox center of both SDFs */
      const float *loc_a = target.target_ob->object_to_world().location();
      const float *loc_b = ob->object_to_world().location();
      grp_ob->loc[0] = (loc_a[0] + loc_b[0]) * 0.5f;
      grp_ob->loc[1] = (loc_a[1] + loc_b[1]) * 0.5f;
      grp_ob->loc[2] = (loc_a[2] + loc_b[2]) * 0.5f;

      BKE_view_layer_synced_ensure(scene, view_layer);
      Collection *active_collection = BKE_view_layer_active_collection_get(view_layer)->collection;
      BKE_collection_object_add(bmain, active_collection, grp_ob);

      BKE_view_layer_synced_ensure(scene, view_layer);
      Base *grp_base = BKE_view_layer_base_find(view_layer, grp_ob);
      if (grp_base) {
        BKE_view_layer_base_select_and_set_active(view_layer, grp_base);
      }

      /* Parent both SDFs — adjust local positions to keep world positions */
      target.target_ob->parent = grp_ob;
      target.target_ob->loc[0] = loc_a[0] - grp_ob->loc[0];
      target.target_ob->loc[1] = loc_a[1] - grp_ob->loc[1];
      target.target_ob->loc[2] = loc_a[2] - grp_ob->loc[2];
      DEG_id_tag_update(&target.target_ob->id, ID_RECALC_TRANSFORM);

      ob->parent = grp_ob;
      ob->loc[0] = loc_b[0] - grp_ob->loc[0];
      ob->loc[1] = loc_b[1] - grp_ob->loc[1];
      ob->loc[2] = loc_b[2] - grp_ob->loc[2];
      DEG_id_tag_update(&ob->id, ID_RECALC_TRANSFORM);

      DEG_id_tag_update(&grp_ob->id, ID_RECALC_TRANSFORM);

      target.create_group = false;
      target.group_empty = grp_ob;
      continue;
    }

    /* Reorder */
    if (target.target_ob && !target.create_group &&
        (target.insert_type == TE_INSERT_BEFORE || target.insert_type == TE_INSERT_AFTER))
    {
      if (target.group_empty && ob->parent != target.group_empty) {
        /* Dragged into a different group — re-parent */
        const float *world_loc = ob->object_to_world().location();
        const float *grp_loc = target.group_empty->object_to_world().location();
        ob->parent = target.group_empty;
        ob->loc[0] = world_loc[0] - grp_loc[0];
        ob->loc[1] = world_loc[1] - grp_loc[1];
        ob->loc[2] = world_loc[2] - grp_loc[2];
        DEG_id_tag_update(&ob->id, ID_RECALC_TRANSFORM);
      }
      else if (!target.group_empty && ob->parent && is_sdf_group_empty(ob->parent)) {
        /* Dragged OUT of group to top level — unparent */
        Object *old_group = ob->parent;
        const float *world_loc = ob->object_to_world().location();
        ob->parent = nullptr;
        ob->loc[0] = world_loc[0];
        ob->loc[1] = world_loc[1];
        ob->loc[2] = world_loc[2];
        DEG_id_tag_update(&ob->id, ID_RECALC_TRANSFORM);
        /* Delete old group if now empty */
        bool still_has_children = false;
        for (Object &check : bmain->objects) {
          if (&check != ob && check.parent == old_group) {
            still_has_children = true;
            break;
          }
        }
        if (!still_has_children) {
          ed::object::base_free_and_unlink(bmain, scene, old_group);
        }
      }

      Object *parent_filter = target.group_empty;
      sdf_reindex_siblings(scene, view_layer, ob, target.target_ob,
                           target.insert_type == TE_INSERT_BEFORE, parent_filter);
      continue;
    }

    /* Ungroup: drop on empty space / Scene Collection */
    if (!target.group_empty && !target.target_ob &&
        ob->parent && is_sdf_group_empty(ob->parent))
    {
      Object *old_group = ob->parent;
      const SDF *grp_sdf = reinterpret_cast<const SDF *>(old_group->data);
      int grp_idx = grp_sdf ? grp_sdf->sdf_index : 0;

      const float *world_loc = ob->object_to_world().location();
      ob->parent = nullptr;
      ob->loc[0] = world_loc[0];
      ob->loc[1] = world_loc[1];
      ob->loc[2] = world_loc[2];

      SDF *sdf = reinterpret_cast<SDF *>(ob->data);
      if (sdf) {
        sdf->sdf_index = grp_idx + 1;
      }

      DEG_id_tag_update(&ob->id, ID_RECALC_TRANSFORM);
      DEG_id_tag_update(static_cast<ID *>(ob->data), ID_RECALC_GEOMETRY);

      /* Delete old group if now empty */
      bool still_has_children = false;
      for (Object &check : bmain->objects) {
        if (&check != ob && check.parent == old_group) {
          still_has_children = true;
          break;
        }
      }
      if (!still_has_children) {
        ed::object::base_free_and_unlink(bmain, scene, old_group);
      }
      continue;
    }

    /* Add to existing group (drop INTO) */
    if (target.group_empty) {
      if (ob->parent && is_sdf_group_empty(ob->parent) && ob->parent != target.group_empty) {
        /* Restore world-space loc before re-parenting */
        const float *old_parent_loc = ob->parent->object_to_world().location();
        ob->loc[0] += old_parent_loc[0];
        ob->loc[1] += old_parent_loc[1];
        ob->loc[2] += old_parent_loc[2];
        ob->parent = nullptr;
      }

      if (ob->parent != target.group_empty) {
        const float *world_loc = ob->object_to_world().location();
        const float *grp_loc = target.group_empty->object_to_world().location();
        ob->parent = target.group_empty;
        ob->loc[0] = world_loc[0] - grp_loc[0];
        ob->loc[1] = world_loc[1] - grp_loc[1];
        ob->loc[2] = world_loc[2] - grp_loc[2];
      }
      DEG_id_tag_update(&ob->id, ID_RECALC_TRANSFORM);
      continue;
    }
  }

  /* Auto-delete empty groups */
  {
    Vector<Object *> empty_groups;
    for (Object &ob_iter : bmain->objects) {
      if (!is_sdf_group_empty(&ob_iter)) {
        continue;
      }
      bool has_children = false;
      for (Object &child_iter : bmain->objects) {
        if (child_iter.parent == &ob_iter) {
          has_children = true;
          break;
        }
      }
      if (!has_children) {
        empty_groups.append(&ob_iter);
      }
    }
    for (Object *grp : empty_groups) {
      ed::object::base_free_and_unlink(bmain, scene, grp);
    }
  }

  DEG_relations_tag_update(bmain);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_LAYER_CONTENT, nullptr);

  return OPERATOR_FINISHED;
}

void OUTLINER_OT_sdf_group_drop(wmOperatorType *ot)
{
  ot->name = "SDF Group Drop";
  ot->description = "Drag SDF object to group, reorder, or create new group in Outliner";
  ot->idname = "OUTLINER_OT_sdf_group_drop";

  ot->invoke = sdf_group_drop_invoke;
  ot->poll = ED_operator_outliner_active;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Drop Boxes
 * \{ */

void outliner_dropboxes()
{
  ListBaseT<wmDropBox> *lb = WM_dropboxmap_find("Outliner", SPACE_OUTLINER, RGN_TYPE_WINDOW);

  /* SDF group drop must be first to intercept SDF-on-SDF before parent_drop */
  WM_dropbox_add(lb,
                 "OUTLINER_OT_sdf_group_drop",
                 sdf_group_drop_poll,
                 nullptr,
                 nullptr,
                 sdf_group_drop_tooltip);
  WM_dropbox_add(lb, "OUTLINER_OT_parent_drop", parent_drop_poll, nullptr, nullptr, nullptr);
  WM_dropbox_add(lb, "OUTLINER_OT_parent_clear", parent_clear_poll, nullptr, nullptr, nullptr);
  WM_dropbox_add(lb, "OUTLINER_OT_scene_drop", scene_drop_poll, nullptr, nullptr, nullptr);
  WM_dropbox_add(lb, "OUTLINER_OT_material_drop", material_drop_poll, nullptr, nullptr, nullptr);
  WM_dropbox_add(lb,
                 "OUTLINER_OT_datastack_drop",
                 datastack_drop_poll,
                 nullptr,
                 nullptr,
                 datastack_drop_tooltip);
  WM_dropbox_add(lb,
                 "OUTLINER_OT_collection_drop",
                 collection_drop_poll,
                 nullptr,
                 nullptr,
                 collection_drop_tooltip);
}

/** \} */

}  // namespace blender::ed::outliner
