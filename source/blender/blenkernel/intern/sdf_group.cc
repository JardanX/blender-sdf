/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "MEM_guardedalloc.h"

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

namespace blender {

static void sdf_group_init_data(ID *id)
{
  SDFGroup *group = id_cast<SDFGroup *>(id);
  INIT_DEFAULT_STRUCT_AFTER(group, id);

  group->runtime = new bke::SDFGroupRuntime();
}

static void sdf_group_copy_data(Main * /*bmain*/,
                                std::optional<Library *> /*owner_library*/,
                                ID *id_dst,
                                const ID *id_src,
                                const int /*flag*/)
{
  SDFGroup *group_dst = id_cast<SDFGroup *>(id_dst);
  const SDFGroup *group_src = id_cast<const SDFGroup *>(id_src);

  BLI_duplicatelist(&group_dst->members, &group_src->members);
  BLI_duplicatelist(&group_dst->modifiers, &group_src->modifiers);

  group_dst->runtime = new bke::SDFGroupRuntime();
}

static void sdf_group_free_data(ID *id)
{
  SDFGroup *group = id_cast<SDFGroup *>(id);
  BKE_animdata_free(&group->id, false);
  BLI_freelistN(&group->members);
  BLI_freelistN(&group->modifiers);
  delete group->runtime;
}

static void sdf_group_foreach_id(ID *id, LibraryForeachIDData *data)
{
  SDFGroup *group = id_cast<SDFGroup *>(id);
  for (SDFGroupMember *member = (SDFGroupMember *)group->members.first; member;
       member = member->next)
  {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, member->object, IDWALK_CB_NOP);
  }
  for (SDFModifier *mod = (SDFModifier *)group->modifiers.first; mod; mod = mod->next) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, mod->mirror_ob, IDWALK_CB_NOP);
  }
}

static void sdf_group_blend_write(BlendWriter *writer, ID *id, const void *id_address)
{
  SDFGroup *group = id_cast<SDFGroup *>(id);

  writer->write_id_struct(id_address, group);
  BKE_id_blend_write(writer, &group->id);

  writer->write_struct_list_by_id(dna::sdna_struct_id_get<SDFGroupMember>(), &group->members);
  writer->write_struct_list_by_id(dna::sdna_struct_id_get<SDFModifier>(), &group->modifiers);
}

static void sdf_group_blend_read_data(BlendDataReader *reader, ID *id)
{
  SDFGroup *group = id_cast<SDFGroup *>(id);

  BLO_read_struct_list(reader, SDFGroupMember, &group->members);
  BLO_read_struct_list(reader, SDFModifier, &group->modifiers);

  group->runtime = new bke::SDFGroupRuntime();
}

static void sdf_group_blend_read_after_liblink(BlendLibReader * /*reader*/, ID *id)
{
  SDFGroup *group = id_cast<SDFGroup *>(id);
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
  SDFGroupMember *member = MEM_new<SDFGroupMember>(__func__);
  member->object = ob;
  member->order = group->totmember;
  BLI_addtail(&group->members, member);
  group->totmember++;

  if (ob && ob->type == OB_SDF && ob->data) {
    SDF *sdf = id_cast<SDF *>(ob->data);
    sdf->sdf_group = group;
    sdf->group_order = member->order;
    id_us_plus(&group->id);
  }

  return member;
}

SDFGroupMember *BKE_sdf_group_member_insert_after(SDFGroup *group,
                                                   Object *after_ob,
                                                   Object *new_ob)
{
  SDFGroupMember *after_member = nullptr;
  for (SDFGroupMember *member = (SDFGroupMember *)group->members.first; member;
       member = member->next)
  {
    if (member->object == after_ob) {
      after_member = member;
      break;
    }
  }

  SDFGroupMember *new_member = MEM_new<SDFGroupMember>(__func__);
  new_member->object = new_ob;

  if (after_member) {
    BLI_insertlinkafter(&group->members, after_member, new_member);
  }
  else {
    BLI_addtail(&group->members, new_member);
  }
  group->totmember++;

  if (new_ob && new_ob->type == OB_SDF && new_ob->data) {
    SDF *sdf = id_cast<SDF *>(new_ob->data);
    sdf->sdf_group = group;
    id_us_plus(&group->id);
  }

  BKE_sdf_group_reindex_members(group);
  return new_member;
}

void BKE_sdf_group_member_remove(SDFGroup *group, SDFGroupMember *member)
{
  if (member->object && member->object->type == OB_SDF && member->object->data) {
    SDF *sdf = id_cast<SDF *>(member->object->data);
    if (sdf->sdf_group == group) {
      id_us_min(&group->id);
      sdf->sdf_group = nullptr;
      sdf->group_order = 0;
    }
  }

  BLI_remlink(&group->members, member);
  MEM_delete(member);
  group->totmember--;

  BKE_sdf_group_reindex_members(group);
}

void BKE_sdf_group_member_move_to(SDFGroup *group,
                                   SDFGroupMember *member,
                                   SDFGroupMember *target,
                                   bool before)
{
  if (member == target) {
    return;
  }
  BLI_remlink(&group->members, member);
  if (before) {
    BLI_insertlinkbefore(&group->members, target, member);
  }
  else {
    BLI_insertlinkafter(&group->members, target, member);
  }
  BKE_sdf_group_reindex_members(group);
}

SDFGroupMember *BKE_sdf_group_member_find_by_object(SDFGroup *group, Object *ob)
{
  for (SDFGroupMember *member = static_cast<SDFGroupMember *>(group->members.first); member;
       member = member->next)
  {
    if (member->object == ob) {
      return member;
    }
  }
  return nullptr;
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
  SDFGroupMember *member_next;
  for (SDFGroupMember *member = (SDFGroupMember *)group->members.first; member;
       member = member_next)
  {
    member_next = member->next;
    if (member->object == nullptr) {
      BLI_remlink(&group->members, member);
      MEM_delete(member);
      group->totmember--;
    }
  }
  int i = 0;
  for (SDFGroupMember *member = (SDFGroupMember *)group->members.first; member;
       member = member->next)
  {
    member->order = i++;
  }
}

void BKE_sdf_groups_cleanup_all_null_members(Main *bmain)
{
  for (SDFGroup *group = (SDFGroup *)bmain->sdf_groups.first; group;
       group = (SDFGroup *)group->id.next)
  {
    BKE_sdf_group_cleanup_null_members(group);
  }
}

void BKE_sdf_groups_remove_object(Main *bmain, Object *ob)
{
  for (SDFGroup *group = (SDFGroup *)bmain->sdf_groups.first; group;
       group = (SDFGroup *)group->id.next)
  {
    SDFGroupMember *member_next;
    for (SDFGroupMember *member = (SDFGroupMember *)group->members.first; member;
         member = member_next)
    {
      member_next = member->next;
      if (member->object == ob) {
        BKE_sdf_group_member_remove(group, member);
      }
    }
  }
}

void BKE_sdf_groups_after_lib_link(Main *bmain)
{
  for (SDF *sdf = static_cast<SDF *>(bmain->sdfs.first); sdf;
       sdf = static_cast<SDF *>(sdf->id.next))
  {
    sdf->sdf_group = nullptr;
    sdf->group_order = 0;
  }

  for (SDFGroup *group = (SDFGroup *)bmain->sdf_groups.first; group;
       group = (SDFGroup *)group->id.next)
  {
    int member_order = 0;
    for (SDFGroupMember *member = (SDFGroupMember *)group->members.first; member;
         member = member->next)
    {
      member->order = member_order;
      if (member->object && member->object->type == OB_SDF && member->object->data) {
        SDF *sdf = id_cast<SDF *>(member->object->data);
        sdf->sdf_group = group;
        sdf->group_order = member_order;
      }
      member_order++;
    }
    group->totmember = member_order;
  }
}

void BKE_sdf_group_reindex_members(SDFGroup *group)
{
  int i = 0;
  for (SDFGroupMember *member = (SDFGroupMember *)group->members.first; member;
       member = member->next)
  {
    member->order = i;
    if (member->object && member->object->type == OB_SDF && member->object->data) {
      SDF *sdf = id_cast<SDF *>(member->object->data);
      sdf->group_order = i;
    }
    i++;
  }
}

static const char *sdf_group_modifier_type_name(int type)
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
    case SDF_MOD_SOLIDIFY:
      return "Solidify";
    case SDF_MOD_ROUND:
      return "Expand/Shrink";
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

SDFModifier *BKE_sdf_group_modifier_add(SDFGroup *group, int type)
{
  SDFModifier *mod = MEM_new<SDFModifier>(__func__);
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
    case SDF_MOD_SOLIDIFY:
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

  BLI_strncpy(mod->name, sdf_group_modifier_type_name(type), sizeof(mod->name));
  BLI_addtail(&group->modifiers, mod);
  group->totmodifier++;
  return mod;
}

void BKE_sdf_group_modifier_remove(SDFGroup *group, SDFModifier *mod)
{
  BLI_remlink(&group->modifiers, mod);
  MEM_delete(mod);
  group->totmodifier--;
}

void BKE_sdf_group_modifier_move(SDFGroup *group, SDFModifier *mod, int direction)
{
  if (direction == -1) {
    SDFModifier *prev = mod->prev;
    if (prev) {
      BLI_remlink(&group->modifiers, mod);
      BLI_insertlinkbefore(&group->modifiers, prev, mod);
    }
  }
  else if (direction == 1) {
    SDFModifier *next = mod->next;
    if (next) {
      BLI_remlink(&group->modifiers, mod);
      BLI_insertlinkafter(&group->modifiers, next, mod);
    }
  }
}

}  // namespace blender
