/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edobj
 */

#include "DNA_object_types.h"
#include "DNA_sdf_types.h"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "BKE_context.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_object.hh"
#include "ED_screen.hh"

#include "object_intern.hh"

namespace blender::ed::object {

static const EnumPropertyItem rna_enum_sdf_type_items[] = {
    {SDF_TYPE_BOX, "BOX", 0, "Cube", "Box SDF primitive"},
    {SDF_TYPE_SPHERE, "SPHERE", 0, "Sphere", "Sphere SDF primitive"},
    {SDF_TYPE_CAPSULE, "CAPSULE", 0, "Capsule", "Capsule SDF primitive"},
    {SDF_TYPE_TORUS, "TORUS", 0, "Torus", "Torus SDF primitive"},
    {0, nullptr, 0, nullptr, nullptr},
};

/* SDF Add */

static const char *sdf_type_name(int type)
{
  switch (type) {
    case SDF_TYPE_BOX:
      return "SDF Cube";
    case SDF_TYPE_SPHERE:
      return "SDF Sphere";
    case SDF_TYPE_CAPSULE:
      return "SDF Capsule";
    case SDF_TYPE_TORUS:
      return "SDF Torus";
    default:
      return "SDF";
  }
}

static Object *object_sdf_add(bContext *C, wmOperator *op, const char *name)
{
  ushort local_view_bits;
  float loc[3], rot[3];

  add_generic_get_opts(C, op, 'Z', loc, rot, nullptr, nullptr, &local_view_bits, nullptr);

  const int type = RNA_enum_get(op->ptr, "type");
  if (!name) {
    name = sdf_type_name(type);
  }

  Object *ob = add_type(C, OB_SDF, name, loc, rot, false, local_view_bits);
  if (ob && ob->data) {
    SDF *sdf_data = static_cast<SDF *>(ob->data);
    sdf_data->sdf_type = type;
  }
  return ob;
}

static wmOperatorStatus object_sdf_add_exec(bContext *C, wmOperator *op)
{
  return (object_sdf_add(C, op, nullptr) != nullptr) ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
}

void OBJECT_OT_sdf_add(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Add SDF";
  ot->description = "Add an SDF object to the scene";
  ot->idname = "OBJECT_OT_sdf_add";

  /* API callbacks */
  ot->exec = object_sdf_add_exec;
  ot->poll = ED_operator_objectmode;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  add_generic_props(ot, false);

  RNA_def_enum(ot->srna,
               "type",
               rna_enum_sdf_type_items,
               SDF_TYPE_BOX,
               "Type",
               "SDF primitive type");
}

}  // namespace blender::ed::object
