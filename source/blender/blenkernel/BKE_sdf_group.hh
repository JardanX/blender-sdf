/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief SDFGroup (SDF evaluation group) data-block.
 */

struct Main;
struct Object;
struct SDFGroup;
struct SDFGroupMember;

namespace blender::bke {

struct SDFGroupRuntime {};

}  // namespace blender::bke

SDFGroup *BKE_sdf_group_add(Main *bmain, const char *name);
SDFGroupMember *BKE_sdf_group_member_add(SDFGroup *group, Object *ob);
void BKE_sdf_group_member_remove(SDFGroup *group, SDFGroupMember *member);
void BKE_sdf_group_member_move(SDFGroup *group, SDFGroupMember *member, int direction);
void BKE_sdf_group_reindex_members(SDFGroup *group);
void BKE_sdf_group_cleanup_null_members(SDFGroup *group);
void BKE_sdf_groups_cleanup_all_null_members(Main *bmain);
void BKE_sdf_groups_remove_object(Main *bmain, Object *ob);

/* Rebuild sdf->sdf_group back-pointers from member lists. Call after lib-link. */
void BKE_sdf_groups_after_lib_link(Main *bmain);
