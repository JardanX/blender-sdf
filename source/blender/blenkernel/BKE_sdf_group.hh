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

struct SDFGroupRuntime {
  /* Placeholder for future runtime data. */
};

}  // namespace blender::bke

SDFGroup *BKE_sdf_group_add(Main *bmain, const char *name);

/** Add an OB_SDF object as a member at the end of the group. */
SDFGroupMember *BKE_sdf_group_member_add(SDFGroup *group, Object *ob);

/** Remove a member from the group. Frees the member. */
void BKE_sdf_group_member_remove(SDFGroup *group, SDFGroupMember *member);

/** Move a member up (-1) or down (+1) in the group and reindex. */
void BKE_sdf_group_member_move(SDFGroup *group, SDFGroupMember *member, int direction);

/** Reassign order values 0, 1, 2, ... to all members. */
void BKE_sdf_group_reindex_members(SDFGroup *group);

/** Remove members whose object pointer is NULL (e.g., after object deletion). */
void BKE_sdf_group_cleanup_null_members(SDFGroup *group);

/** Remove null members from ALL SDFGroups in bmain (called after ID remap). */
void BKE_sdf_groups_cleanup_all_null_members(Main *bmain);

/** Remove all group members referencing the given object from all SDFGroups.
 *  Called before object deletion to ensure clean removal. */
void BKE_sdf_groups_remove_object(Main *bmain, Object *ob);

/** Rebuild sdf->sdf_group back-pointers from SDFGroup member lists.
 *  Must be called after ALL library linking is complete (ob->data is resolved).
 *  Analogous to BKE_collections_after_lib_link(). */
void BKE_sdf_groups_after_lib_link(Main *bmain);
