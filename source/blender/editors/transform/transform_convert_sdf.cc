/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.h"

#include "DNA_sdf_types.h"

#include "BKE_context.hh"
#include "BKE_editmesh.hh"
#include "BKE_sdf.hh"

#include "bmesh.hh"

#include "MEM_guardedalloc.h"

#include "DEG_depsgraph.hh"

#include "transform.hh"
#include "transform_convert.hh"

namespace blender::ed::transform::sdf {

static void createTransSDFVerts(bContext * /*C*/, TransInfo *t)
{
  MutableSpan<TransDataContainer> trans_data_containers(t->data_container,
                                                        t->data_container_len);

  for (const int i : trans_data_containers.index_range()) {
    TransDataContainer &tc = trans_data_containers[i];
    SDF *sdf_data = static_cast<SDF *>(tc.obedit->data);
    BMEditMesh *em = sdf_data->runtime->edit_mesh;
    if (!em || !em->bm) {
      tc.data_len = 0;
      continue;
    }

    BMesh *bm = em->bm;
    int count_selected = 0;
    BMIter iter;
    BMVert *v;

    BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
      if (BM_elem_flag_test(v, BM_ELEM_SELECT)) {
        count_selected++;
      }
    }

    const bool use_proportional = (t->flag & T_PROP_EDIT_ALL) != 0;
    tc.data_len = use_proportional ? bm->totvert : count_selected;

    if (tc.data_len == 0) {
      continue;
    }

    tc.data = MEM_calloc_arrayN<TransData>(tc.data_len, __func__);

    const float4x4 &transform = tc.obedit->object_to_world();
    const float3x3 mtx_base = transform.view<3, 3>();
    const float3x3 smtx_base = math::pseudo_invert(mtx_base);

    int idx = 0;
    BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
      bool is_selected = BM_elem_flag_test(v, BM_ELEM_SELECT);
      if (!use_proportional && !is_selected) {
        continue;
      }

      TransData &td = tc.data[idx];
      copy_v3_v3(td.iloc, v->co);
      copy_v3_v3(td.center, v->co);
      td.loc = v->co;
      td.flag = is_selected ? TD_SELECTED : 0;

      copy_m3_m3(td.smtx, smtx_base.ptr());
      copy_m3_m3(td.mtx, mtx_base.ptr());

      idx++;
    }
  }
}

static void recalcData_sdf(TransInfo *t)
{
  const Span<TransDataContainer> trans_data_containers(t->data_container, t->data_container_len);
  for (const TransDataContainer &tc : trans_data_containers) {
    SDF *sdf_data = static_cast<SDF *>(tc.obedit->data);
    DEG_id_tag_update(&sdf_data->id, ID_RECALC_GEOMETRY);
  }
}

TransConvertTypeInfo TransConvertType_SDF = {
    /*flags*/ (T_EDIT | T_POINTS),
    /*create_trans_data*/ sdf::createTransSDFVerts,
    /*recalc_data*/ sdf::recalcData_sdf,
    /*special_aftertrans_update*/ nullptr,
};

}  // namespace blender::ed::transform::sdf
