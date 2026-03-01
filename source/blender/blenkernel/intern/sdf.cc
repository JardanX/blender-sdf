/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <optional>

#include "MEM_guardedalloc.h"

#include "DNA_defaults.h"
#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"

#include "BLI_utildefines.h"

#include "BKE_anim_data.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
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

static void sdf_copy_data(Main * /*bmain*/,
                          std::optional<Library *> /*owner_library*/,
                          ID *id_dst,
                          const ID *id_src,
                          const int /*flag*/)
{
  SDF *sdf_dst = (SDF *)id_dst;
  const SDF *sdf_src = (const SDF *)id_src;

  sdf_dst->mat = static_cast<Material **>(MEM_dupallocN(sdf_src->mat));
  sdf_dst->runtime = new blender::bke::SDFRuntime();
}

static void sdf_free_data(ID *id)
{
  SDF *sdf = (SDF *)id;
  BKE_animdata_free(&sdf->id, false);
  MEM_SAFE_FREE(sdf->mat);
  delete sdf->runtime;
}

static void sdf_foreach_id(ID *id, LibraryForeachIDData *data)
{
  SDF *sdf = (SDF *)id;
  for (int i = 0; i < sdf->totcol; i++) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, sdf->mat[i], IDWALK_CB_USER);
  }
}

static void sdf_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  SDF *sdf = (SDF *)id;

  BLO_write_id_struct(writer, SDF, id_address, &sdf->id);
  BKE_id_blend_write(writer, &sdf->id);

  /* Direct data */
  BLO_write_pointer_array(writer, sdf->totcol, sdf->mat);
}

static void sdf_blend_read_data(BlendDataReader *reader, ID *id)
{
  SDF *sdf = (SDF *)id;

  /* Materials */
  BLO_read_pointer_array(reader, sdf->totcol, (void **)&sdf->mat);

  sdf->runtime = new blender::bke::SDFRuntime();
}

IDTypeInfo IDType_ID_SF = {
    /*id_code*/ SDF::id_type,
    /*id_filter*/ FILTER_ID_SF,
    /*dependencies_id_types*/ FILTER_ID_MA,
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

void BKE_sdf_data_update(Depsgraph * /*depsgraph*/, Scene * /*scene*/, Object * /*ob*/)
{
  /* No-op for now — rendering is handled externally. */
}
