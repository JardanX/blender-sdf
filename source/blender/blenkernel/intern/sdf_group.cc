/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "MEM_guardedalloc.h"

#include "DNA_defaults.h"
#include "DNA_object_types.h"
#include "DNA_sdf_group_types.h"
#include "DNA_sdf_types.h"

#include "BLI_listbase.h"
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "BKE_anim_data.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_main.hh"
#include "BKE_sdf_group.hh"

#include "BLT_translation.hh"

#include "BLO_read_write.hh"

static void sdf_group_init_data(ID *id)
{
  SDFGroup *group = (SDFGroup *)id;
  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(group, id));

  MEMCPY_STRUCT_AFTER(group, DNA_struct_default_get(SDFGroup), id);

  group->runtime = new blender::bke::SDFGroupRuntime();
}

static void sdf_group_copy_data(Main * /*bmain*/,
                                std::optional<Library *> /*owner_library*/,
                                ID *id_dst,
                                const ID *id_src,
                                const int /*flag*/)
{
  SDFGroup *group_dst = (SDFGroup *)id_dst;
  const SDFGroup *group_src = (const SDFGroup *)id_src;

  /* Deep-copy the member list (not the Object pointers themselves). */
  BLI_duplicatelist(&group_dst->members, &group_src->members);

  group_dst->runtime = new blender::bke::SDFGroupRuntime();
}

static void sdf_group_free_data(ID *id)
{
  SDFGroup *group = (SDFGroup *)id;
  BKE_animdata_free(&group->id, false);
  BLI_freelistN(&group->members);
  delete group->runtime;
}

static void sdf_group_foreach_id(ID *id, LibraryForeachIDData *data)
{
  SDFGroup *group = (SDFGroup *)id;
  LISTBASE_FOREACH (SDFGroupMember *, member, &group->members) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, member->object, IDWALK_CB_NOP);
  }
}

static void sdf_group_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  SDFGroup *group = (SDFGroup *)id;

  BLO_write_id_struct(writer, SDFGroup, id_address, &group->id);
  BKE_id_blend_write(writer, &group->id);

  /* Write member list. */
  LISTBASE_FOREACH (SDFGroupMember *, member, &group->members) {
    BLO_write_struct(writer, SDFGroupMember, member);
  }
}

static void sdf_group_blend_read_data(BlendDataReader *reader, ID *id)
{
  SDFGroup *group = (SDFGroup *)id;

  /* Read member list. */
  BLO_read_struct_list(reader, SDFGroupMember, &group->members);

  group->runtime = new blender::bke::SDFGroupRuntime();
}

static void sdf_group_blend_read_after_liblink(BlendLibReader * /*reader*/, ID *id)
{
  /* Clean up zombie members (object was deleted before save, or link failed). */
  SDFGroup *group = (SDFGroup *)id;
  BKE_sdf_group_cleanup_null_members(group);
}

IDTypeInfo IDType_ID_SG = {
    /*id_code*/ SDFGroup::id_type,
    /*id_filter*/ FILTER_ID_SG,
    /*dependencies_id_types*/ FILTER_ID_OB,
    /*main_listbase_index*/ INDEX_ID_SG,
    /*struct_size*/ sizeof(SDFGroup),
    /*name*/ "SDFGroup",
    /*name_plural*/ N_("sdf_groups"),
    /*translation_context*/ BLT_I18NCONTEXT_ID_SDF,
    /*flags*/ IDTYPE_FLAGS_APPEND_IS_REUSABLE,
    /*asset_type_info*/ nullptr,

    /*init_data*/ sdf_group_init_data,
    /*copy_data*/ sdf_group_copy_data,
    /*free_data*/ sdf_group_free_data,
    /*make_local*/ nullptr,
    /*foreach_id*/ sdf_group_foreach_id,
    /*foreach_cache*/ nullptr,
    /*foreach_path*/ nullptr,
    /*foreach_working_space_color*/ nullptr,
    /*owner_pointer_get*/ nullptr,

    /*blend_write*/ sdf_group_blend_write,
    /*blend_read_data*/ sdf_group_blend_read_data,
    /*blend_read_after_liblink*/ sdf_group_blend_read_after_liblink,

    /*blend_read_undo_preserve*/ nullptr,

    /*lib_override_apply_post*/ nullptr,
};

SDFGroup *BKE_sdf_group_add(Main *bmain, const char *name)
{
  SDFGroup *group = BKE_id_new<SDFGroup>(bmain, name);
  return group;
}

SDFGroupMember *BKE_sdf_group_member_add(SDFGroup *group, Object *ob)
{
  SDFGroupMember *member = static_cast<SDFGroupMember *>(
      MEM_callocN(sizeof(SDFGroupMember), "SDFGroupMember"));
  member->object = ob;
  member->order = group->totmember;
  BLI_addtail(&group->members, member);
  group->totmember++;

  /* Set back-pointer on the SDF data (SDF holds a USER reference to the group,
   * matching IDWALK_CB_USER in sdf_foreach_id). */
  if (ob && ob->type == OB_SDF && ob->data) {
    SDF *sdf = static_cast<SDF *>(ob->data);
    sdf->sdf_group = group;
    sdf->group_order = member->order;
    id_us_plus(&group->id);
  }

  return member;
}

void BKE_sdf_group_member_remove(SDFGroup *group, SDFGroupMember *member)
{
  /* Clear back-pointer on the SDF data and release group user reference. */
  if (member->object && member->object->type == OB_SDF && member->object->data) {
    SDF *sdf = static_cast<SDF *>(member->object->data);
    if (sdf->sdf_group == group) {
      id_us_min(&group->id);
      sdf->sdf_group = nullptr;
      sdf->group_order = 0;
    }
  }

  BLI_remlink(&group->members, member);
  MEM_freeN(member);
  group->totmember--;

  BKE_sdf_group_reindex_members(group);
}

void BKE_sdf_group_member_move(SDFGroup *group, SDFGroupMember *member, int direction)
{
  if (direction == -1) {
    SDFGroupMember *prev = member->prev;
    if (prev) {
      BLI_remlink(&group->members, member);
      BLI_insertlinkbefore(&group->members, prev, member);
    }
  }
  else if (direction == 1) {
    SDFGroupMember *next = member->next;
    if (next) {
      BLI_remlink(&group->members, member);
      BLI_insertlinkafter(&group->members, next, member);
    }
  }

  BKE_sdf_group_reindex_members(group);
}

void BKE_sdf_group_cleanup_null_members(SDFGroup *group)
{
  bool changed = false;
  LISTBASE_FOREACH_MUTABLE (SDFGroupMember *, member, &group->members) {
    if (member->object == nullptr || member->object->type != OB_SDF || member->object->data == nullptr) {
      /* Properly remove the member to ensure user counts and back-pointers are updated. */
      BKE_sdf_group_member_remove(group, member);
      changed = true;
    }
  }
}

void BKE_sdf_groups_cleanup_all_null_members(Main *bmain)
{
  LISTBASE_FOREACH (SDFGroup *, group, &bmain->sdf_groups) {
    BKE_sdf_group_cleanup_null_members(group);
  }
}

void BKE_sdf_groups_remove_object(Main *bmain, Object *ob)
{
  LISTBASE_FOREACH (SDFGroup *, group, &bmain->sdf_groups) {
    LISTBASE_FOREACH_MUTABLE (SDFGroupMember *, member, &group->members) {
      if (member->object == ob) {
        /* BKE_sdf_group_member_remove handles user counts and SDF back-pointer. */
        BKE_sdf_group_member_remove(group, member);
      }
    }
  }
}

void BKE_sdf_group_reindex_members(SDFGroup *group)
{
  int i = 0;
  LISTBASE_FOREACH (SDFGroupMember *, member, &group->members) {
    member->order = i;
    /* Sync back-pointer order field. */
    if (member->object && member->object->type == OB_SDF && member->object->data) {
      SDF *sdf = static_cast<SDF *>(member->object->data);
      sdf->group_order = i;
    }
    i++;
  }
}
