/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "MEM_guardedalloc.h"

#include "DNA_defaults.h"
#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_group_types.h"
#include "DNA_sdf_types.h"

#include "BLI_listbase.h"
#include "BLI_math_vector.h"
#include "BLI_string.h"
#include "BLI_string_utils.hh"
#include "BLI_utildefines.h"
#include "BLI_vector.hh"

#include "BKE_editmesh.hh"

#include "bmesh.hh"

#include "BKE_anim_data.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_main.hh"
#include "BKE_sdf.hh"

#include "BLT_translation.hh"

#include "BLO_read_write.hh"

static void sdf_init_data(ID *id)
{
  SDF *sdf = (SDF *)id;
  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(sdf, id));

  MEMCPY_STRUCT_AFTER(sdf, DNA_struct_default_get(SDF), id);

  sdf->runtime = new blender::bke::SDFRuntime();
}

static void sdf_copy_data(Main *bmain,
                          std::optional<Library *> /*owner_library*/,
                          ID *id_dst,
                          const ID *id_src,
                          const int /*flag*/)
{
  SDF *sdf_dst = (SDF *)id_dst;
  const SDF *sdf_src = (const SDF *)id_src;

  sdf_dst->mat = static_cast<Material **>(MEM_dupallocN(sdf_src->mat));
  BLI_duplicatelist(&sdf_dst->modifiers, &sdf_src->modifiers);
  BLI_duplicatelist(&sdf_dst->polygon_points, &sdf_src->polygon_points);
  sdf_dst->runtime = new blender::bke::SDFRuntime();

  if (bmain != nullptr && sdf_src->sdf_index > 0) {
    sdf_dst->sdf_index = sdf_src->sdf_index + 1;
    BKE_sdf_shift_indices_from(bmain, sdf_dst->sdf_index, sdf_dst);
  }
}

static void sdf_free_data(ID *id)
{
  SDF *sdf = (SDF *)id;

  BKE_animdata_free(&sdf->id, false);
  BLI_freelistN(&sdf->modifiers);
  BLI_freelistN(&sdf->polygon_points);
  MEM_SAFE_FREE(sdf->mat);
  delete sdf->runtime;
}

static void sdf_foreach_id(ID *id, LibraryForeachIDData *data)
{
  SDF *sdf = (SDF *)id;
  for (int i = 0; i < sdf->totcol; i++) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, sdf->mat[i], IDWALK_CB_USER);
  }
  BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, sdf->sdf_group, IDWALK_CB_USER);

  LISTBASE_FOREACH (SDFModifier *, mod, &sdf->modifiers) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, mod->mirror_ob, IDWALK_CB_NOP);
  }
}

static void sdf_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  SDF *sdf = (SDF *)id;

  BLO_write_id_struct(writer, SDF, id_address, &sdf->id);
  BKE_id_blend_write(writer, &sdf->id);
  BLO_write_pointer_array(writer, sdf->totcol, sdf->mat);
  BLO_write_struct_list(writer, SDFModifier, &sdf->modifiers);
  BLO_write_struct_list(writer, SDFPolygonPoint, &sdf->polygon_points);
}

static void sdf_blend_read_data(BlendDataReader *reader, ID *id)
{
  SDF *sdf = (SDF *)id;

  BLO_read_pointer_array(reader, sdf->totcol, (void **)&sdf->mat);
  BLO_read_struct_list(reader, SDFModifier, &sdf->modifiers);
  BLO_read_struct_list(reader, SDFPolygonPoint, &sdf->polygon_points);

  sdf->runtime = new blender::bke::SDFRuntime();
}

IDTypeInfo IDType_ID_SF = {
    /*id_code*/ SDF::id_type,
    /*id_filter*/ FILTER_ID_SF,
    /*dependencies_id_types*/ FILTER_ID_MA | FILTER_ID_SG,
    /*main_listbase_index*/ INDEX_ID_SF,
    /*struct_size*/ sizeof(SDF),
    /*name*/ "SDF",
    /*name_plural*/ N_("sdfs"),
    /*translation_context*/ BLT_I18NCONTEXT_ID_SDF,
    /*flags*/ IDTYPE_FLAGS_APPEND_IS_REUSABLE,
    /*asset_type_info*/ nullptr,

    /*init_data*/ sdf_init_data,
    /*copy_data*/ sdf_copy_data,
    /*free_data*/ sdf_free_data,
    /*make_local*/ nullptr,
    /*foreach_id*/ sdf_foreach_id,
    /*foreach_cache*/ nullptr,
    /*foreach_path*/ nullptr,
    /*foreach_working_space_color*/ nullptr,
    /*owner_pointer_get*/ nullptr,

    /*blend_write*/ sdf_blend_write,
    /*blend_read_data*/ sdf_blend_read_data,
    /*blend_read_after_liblink*/ nullptr,

    /*blend_read_undo_preserve*/ nullptr,

    /*lib_override_apply_post*/ nullptr,
};

SDF *BKE_sdf_add(Main *bmain, const char *name)
{
  SDF *sdf = BKE_id_new<SDF>(bmain, name);
  return sdf;
}

void BKE_sdf_data_update(Depsgraph * /*depsgraph*/, Scene * /*scene*/, Object * /*ob*/) {}

static const char *sdf_modifier_type_name(int type)
{
  switch (type) {
    case SDF_MOD_MIRROR:
      return "Mirror";
    case SDF_MOD_TWIST:
      return "Twist";
    case SDF_MOD_BEND:
      return "Bend";
    case SDF_MOD_ELONGATE:
      return "Elongate";
    case SDF_MOD_HOLLOW:
      return "Hollow";
    case SDF_MOD_ROUND:
      return "Round";
    case SDF_MOD_ONION:
      return "Onion";
    case SDF_MOD_BEVEL:
      return "Bevel";
    case SDF_MOD_ARRAY:
      return "Array";
    default:
      return "Modifier";
  }
}

SDFModifier *BKE_sdf_modifier_add(SDF *sdf, int type)
{
  SDFModifier *mod = static_cast<SDFModifier *>(MEM_callocN(sizeof(SDFModifier), "SDFModifier"));
  mod->type = type;
  mod->show_viewport = 1;

  switch (type) {
    case SDF_MOD_MIRROR:
      mod->flag = SDF_MOD_MIRROR_X;
      mod->params[0] = 0.0f;
      break;
    case SDF_MOD_TWIST:
      mod->params[0] = 1.0f;
      break;
    case SDF_MOD_BEND:
      mod->params[0] = 1.0f;
      mod->params[1] = 0.0f;
      break;
    case SDF_MOD_ELONGATE:
      mod->params[0] = 0.5f;
      mod->params[1] = 0.0f;
      mod->params[2] = 0.0f;
      break;
    case SDF_MOD_HOLLOW:
      mod->params[0] = 0.1f;
      break;
    case SDF_MOD_ROUND:
      mod->params[0] = 0.05f;
      break;
    case SDF_MOD_ONION:
      mod->params[0] = 0.1f;
      break;
    case SDF_MOD_BEVEL:
      mod->params[0] = 0.1f;
      break;
    case SDF_MOD_ARRAY:
      mod->flag = SDF_MOD_ARRAY_LINEAR;
      mod->params[0] = 3.0f;
      mod->params[1] = 1.0f;
      mod->params[2] = 0.0f;
      mod->params[3] = 0.0f;
      break;
    default:
      break;
  }

  BLI_strncpy(mod->name, sdf_modifier_type_name(type), sizeof(mod->name));
  BLI_addtail(&sdf->modifiers, mod);
  sdf->totmodifier++;
  return mod;
}

void BKE_sdf_modifier_remove(SDF *sdf, SDFModifier *mod)
{
  BLI_remlink(&sdf->modifiers, mod);
  MEM_freeN(mod);
  sdf->totmodifier--;
}

void BKE_sdf_modifier_move(SDF *sdf, SDFModifier *mod, int direction)
{
  if (direction == -1) {
    SDFModifier *prev = mod->prev;
    if (prev) {
      BLI_remlink(&sdf->modifiers, mod);
      BLI_insertlinkbefore(&sdf->modifiers, prev, mod);
    }
  }
  else if (direction == 1) {
    SDFModifier *next = mod->next;
    if (next) {
      BLI_remlink(&sdf->modifiers, mod);
      BLI_insertlinkafter(&sdf->modifiers, next, mod);
    }
  }
}

int BKE_sdf_next_index(Main *bmain)
{
  int max_idx = 0;
  LISTBASE_FOREACH (ID *, id, &bmain->sdfs) {
    SDF *sdf = (SDF *)id;
    if (sdf->sdf_index > max_idx) {
      max_idx = sdf->sdf_index;
    }
  }
  return max_idx + 1;
}

void BKE_sdf_reindex_all(Main *bmain)
{
  /* Collect all SDFs, sort by current index, reassign 1-based */
  int count = BLI_listbase_count(&bmain->sdfs);
  if (count == 0) {
    return;
  }

  blender::Vector<SDF *> all_sdfs(count);
  int i = 0;
  LISTBASE_FOREACH (ID *, id, &bmain->sdfs) {
    all_sdfs[i++] = (SDF *)id;
  }

  std::stable_sort(all_sdfs.begin(), all_sdfs.end(), [](const SDF *a, const SDF *b) {
    return a->sdf_index < b->sdf_index;
  });

  for (int idx = 0; idx < count; idx++) {
    all_sdfs[idx]->sdf_index = idx + 1;
  }
}

void BKE_sdf_shift_indices_from(Main *bmain, int from_index, const SDF *skip)
{
  LISTBASE_FOREACH (ID *, id, &bmain->sdfs) {
    SDF *sdf = (SDF *)id;
    if (sdf == skip) {
      continue;
    }
    if (sdf->sdf_index >= from_index) {
      sdf->sdf_index++;
    }
  }
}

SDFPolygonPoint *BKE_sdf_polygon_point_add(SDF *sdf, float x, float y, float corner)
{
  SDFPolygonPoint *pt = static_cast<SDFPolygonPoint *>(
      MEM_callocN(sizeof(SDFPolygonPoint), "SDFPolygonPoint"));
  pt->co[0] = x;
  pt->co[1] = y;
  pt->corner = corner;
  BLI_addtail(&sdf->polygon_points, pt);
  sdf->totpolygon++;
  return pt;
}

void BKE_sdf_polygon_point_remove(SDF *sdf, SDFPolygonPoint *point)
{
  BLI_remlink(&sdf->polygon_points, point);
  MEM_freeN(point);
  sdf->totpolygon--;
}

void BKE_sdf_polygon_init_triangle(SDF *sdf)
{
  BLI_freelistN(&sdf->polygon_points);
  sdf->totpolygon = 0;
  BKE_sdf_polygon_point_add(sdf, 0.0f, 0.866f, 0.0f);
  BKE_sdf_polygon_point_add(sdf, -0.75f, -0.433f, 0.0f);
  BKE_sdf_polygon_point_add(sdf, 0.75f, -0.433f, 0.0f);
}

void BKE_sdf_editmode_enter(Object *ob)
{
  SDF *sdf = static_cast<SDF *>(ob->data);
  if (sdf->sdf_type != SDF_TYPE_POLYGON || sdf->totpolygon < 3) {
    return;
  }

  BMeshCreateParams create_params = {};
  create_params.use_toolflags = true;
  BMAllocTemplate allocsize = {sdf->totpolygon, sdf->totpolygon, 0, 0};
  BMesh *bm = BM_mesh_create(&allocsize, &create_params);

  /* Create vertices from polygon points */
  blender::Vector<BMVert *> verts;
  LISTBASE_FOREACH (SDFPolygonPoint *, pt, &sdf->polygon_points) {
    float co[3] = {pt->co[0], pt->co[1], 0.0f};
    BMVert *v = BM_vert_create(bm, co, nullptr, BM_CREATE_NOP);
    verts.append(v);
  }

  /* Create edge loop */
  for (int i = 0; i < int(verts.size()); i++) {
    int j = (i + 1) % int(verts.size());
    BM_edge_create(bm, verts[i], verts[j], nullptr, BM_CREATE_NOP);
  }

  BMEditMesh *em = BKE_editmesh_create(bm);
  sdf->runtime->edit_mesh = em;
}

void BKE_sdf_editmode_load(Object *ob)
{
  SDF *sdf = static_cast<SDF *>(ob->data);
  BMEditMesh *em = sdf->runtime->edit_mesh;
  if (!em || !em->bm) {
    return;
  }

  BMesh *bm = em->bm;

  /* Clear existing polygon points */
  BLI_freelistN(&sdf->polygon_points);
  sdf->totpolygon = 0;

  /* Read vertices back in index order */
  BMIter iter;
  BMVert *v;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    BKE_sdf_polygon_point_add(sdf, v->co[0], v->co[1], 0.0f);
  }
}

void BKE_sdf_editmode_exit(Object *ob)
{
  SDF *sdf = static_cast<SDF *>(ob->data);
  BMEditMesh *em = sdf->runtime->edit_mesh;
  if (em) {
    BKE_editmesh_free_data(em);
    MEM_delete(em);
    sdf->runtime->edit_mesh = nullptr;
  }
}
