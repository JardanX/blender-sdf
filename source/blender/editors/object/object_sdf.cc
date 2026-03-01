/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edobj
 */

#include "DNA_object_types.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "BKE_context.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_object.hh"
#include "ED_screen.hh"

#include "object_intern.hh"

namespace blender::ed::object {

/* SDF Add */

static Object *object_sdf_add(bContext *C, wmOperator *op, const char *name)
{
  ushort local_view_bits;
  float loc[3], rot[3];

  add_generic_get_opts(C, op, 'Z', loc, rot, nullptr, nullptr, &local_view_bits, nullptr);

  return add_type(C, OB_SDF, name, loc, rot, false, local_view_bits);
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
}

}  // namespace blender::ed::object
