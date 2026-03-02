/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * MetaBall IDTypeInfo stub — kept for .blend file compatibility only.
 * All metaball functionality has been removed and replaced by SDF.
 */

#include <optional>

#include "MEM_guardedalloc.h"

/* Allow using deprecated functionality for .blend file I/O. */
#define DNA_DEPRECATED_ALLOW

#include "DNA_defaults.h"
#include "DNA_material_types.h"
#include "DNA_meta_types.h"
#include "DNA_object_types.h"

#include "BLI_listbase.h"
#include "BLI_utildefines.h"

#include "BLT_translation.hh"

#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"

#include "BLO_read_write.hh"

static void metaball_init_data(ID *id)
{
  MetaBall *metaball = (MetaBall *)id;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(metaball, id));

  MEMCPY_STRUCT_AFTER(metaball, DNA_struct_default_get(MetaBall), id);
}

static void metaball_copy_data(Main * /*bmain*/,
                               std::optional<Library *> /*owner_library*/,
                               ID *id_dst,
                               const ID *id_src,
                               const int /*flag*/)
{
  MetaBall *metaball_dst = (MetaBall *)id_dst;
  const MetaBall *metaball_src = (const MetaBall *)id_src;

  BLI_duplicatelist(&metaball_dst->elems, &metaball_src->elems);

  metaball_dst->mat = static_cast<Material **>(MEM_dupallocN(metaball_src->mat));

  metaball_dst->editelems = nullptr;
  metaball_dst->lastelem = nullptr;
}

static void metaball_free_data(ID *id)
{
  MetaBall *metaball = (MetaBall *)id;

  MEM_SAFE_FREE(metaball->mat);

  BLI_freelistN(&metaball->elems);
}

static void metaball_foreach_id(ID *id, LibraryForeachIDData *data)
{
  MetaBall *metaball = reinterpret_cast<MetaBall *>(id);
  for (int i = 0; i < metaball->totcol; i++) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, metaball->mat[i], IDWALK_CB_USER);
  }
}

static void metaball_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  MetaBall *mb = (MetaBall *)id;

  /* Clean up, important in undo case to reduce false detection of changed datablocks. */
  mb->editelems = nullptr;
  /* Must always be cleared (meta's don't have their own edit-data). */
  mb->needs_flush_to_id = 0;
  mb->lastelem = nullptr;

  /* write LibData */
  BLO_write_id_struct(writer, MetaBall, id_address, &mb->id);
  BKE_id_blend_write(writer, &mb->id);

  /* direct data */
  BLO_write_pointer_array(writer, mb->totcol, mb->mat);

  LISTBASE_FOREACH (MetaElem *, ml, &mb->elems) {
    BLO_write_struct(writer, MetaElem, ml);
  }
}

static void metaball_blend_read_data(BlendDataReader *reader, ID *id)
{
  MetaBall *mb = (MetaBall *)id;

  BLO_read_pointer_array(reader, mb->totcol, (void **)&mb->mat);

  BLO_read_struct_list(reader, MetaElem, &(mb->elems));

  mb->editelems = nullptr;
  /* Must always be cleared (meta's don't have their own edit-data). */
  mb->needs_flush_to_id = 0;
  mb->lastelem = nullptr;
}

IDTypeInfo IDType_ID_MB = {
    /*id_code*/ MetaBall::id_type,
    /*id_filter*/ FILTER_ID_MB,
    /*dependencies_id_types*/ FILTER_ID_MA,
    /*main_listbase_index*/ INDEX_ID_MB,
    /*struct_size*/ sizeof(MetaBall),
    /*name*/ "Metaball",
    /*name_plural*/ N_("metaballs"),
    /*translation_context*/ BLT_I18NCONTEXT_ID_METABALL,
    /*flags*/ IDTYPE_FLAGS_APPEND_IS_REUSABLE,
    /*asset_type_info*/ nullptr,

    /*init_data*/ metaball_init_data,
    /*copy_data*/ metaball_copy_data,
    /*free_data*/ metaball_free_data,
    /*make_local*/ nullptr,
    /*foreach_id*/ metaball_foreach_id,
    /*foreach_cache*/ nullptr,
    /*foreach_path*/ nullptr,
    /*foreach_working_space_color*/ nullptr,
    /*owner_pointer_get*/ nullptr,

    /*blend_write*/ metaball_blend_write,
    /*blend_read_data*/ metaball_blend_read_data,
    /*blend_read_after_liblink*/ nullptr,

    /*blend_read_undo_preserve*/ nullptr,

    /*lib_override_apply_post*/ nullptr,
};
