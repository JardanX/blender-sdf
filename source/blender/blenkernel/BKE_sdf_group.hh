/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief SDFGroup (SDF evaluation group) data-block.
 */

namespace blender {

struct Main;
struct Object;
struct SDFGroup;
struct SDFGroupMember;
struct SDFModifier;

namespace bke {

struct SDFGroupRuntime {};

}  // namespace bke

SDFGroup *BKE_sdf_group_add(Main *bmain, const char *name);
SDFGroupMember *BKE_sdf_group_member_add(SDFGroup *group, Object *ob);
SDFGroupMember *BKE_sdf_group_member_insert_after(SDFGroup *group,
                                                   Object *after_ob,
                                                   Object *new_ob);
void BKE_sdf_group_member_remove(SDFGroup *group, SDFGroupMember *member);
void BKE_sdf_group_member_move(SDFGroup *group, SDFGroupMember *member, int direction);
void BKE_sdf_group_member_move_to(SDFGroup *group,
                                   SDFGroupMember *member,
                                   SDFGroupMember *target,
                                   bool before);
SDFGroupMember *BKE_sdf_group_member_find_by_object(SDFGroup *group, Object *ob);
void BKE_sdf_group_reindex_members(SDFGroup *group);
void BKE_sdf_group_cleanup_null_members(SDFGroup *group);
void BKE_sdf_groups_cleanup_all_null_members(Main *bmain);
void BKE_sdf_groups_remove_object(Main *bmain, Object *ob);

SDFModifier *BKE_sdf_group_modifier_add(SDFGroup *group, int type);
void BKE_sdf_group_modifier_remove(SDFGroup *group, SDFModifier *mod);
void BKE_sdf_group_modifier_move(SDFGroup *group, SDFModifier *mod, int direction);

void BKE_sdf_groups_after_lib_link(Main *bmain);

}  // namespace blender
