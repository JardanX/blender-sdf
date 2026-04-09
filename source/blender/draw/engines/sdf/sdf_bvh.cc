/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "sdf_bvh.hh"

namespace blender::draw::sdf {

SdfAabbTree::SdfAabbTree()
{
  clear();
}

void SdfAabbTree::clear()
{
  nodes_.clear();
  nodes_.resize(16);
  root_ = null_node;

  for (int i = 0; i < nodes_.size() - 1; ++i) {
    nodes_[i].next_free = i + 1;
    nodes_[i].height = -1;
  }
  nodes_.last().next_free = null_node;
  nodes_.last().height = -1;
  free_list_ = 0;
}

int SdfAabbTree::allocate_node()
{
  if (free_list_ == null_node) {
    int old_size = nodes_.size();
    nodes_.resize(old_size * 2);

    for (int i = old_size; i < nodes_.size() - 1; ++i) {
      nodes_[i].next_free = i + 1;
      nodes_[i].height = -1;
    }
    nodes_.last().next_free = null_node;
    nodes_.last().height = -1;
    free_list_ = old_size;
  }

  int node = free_list_;
  free_list_ = nodes_[node].next_free;
  nodes_[node].parent = null_node;
  nodes_[node].child_a = null_node;
  nodes_[node].child_b = null_node;
  nodes_[node].height = 0;
  nodes_[node].next_free = -1;
  nodes_[node].shape_index = -1;
  nodes_[node].moved = false;
  return node;
}

void SdfAabbTree::free_node(int node)
{
  nodes_[node].next_free = free_list_;
  nodes_[node].height = -1;
  free_list_ = node;
}

int SdfAabbTree::create_proxy(const SdfAabb &bounds, int shape_index)
{
  int proxy = allocate_node();

  nodes_[proxy].bounds = bounds;
  nodes_[proxy].bounds.expand(fat_bounds_radius);
  nodes_[proxy].height = 0;
  nodes_[proxy].shape_index = shape_index;
  nodes_[proxy].moved = true;

  insert_leaf(proxy);

  return proxy;
}

void SdfAabbTree::destroy_proxy(int proxy)
{
  remove_leaf(proxy);
  free_node(proxy);
}

void SdfAabbTree::update_proxy(int proxy, const SdfAabb &bounds)
{
  if (nodes_[proxy].bounds.contains(bounds)) {
    return;
  }

  remove_leaf(proxy);

  nodes_[proxy].bounds = bounds;
  nodes_[proxy].bounds.expand(fat_bounds_radius);

  insert_leaf(proxy);

  nodes_[proxy].moved = true;
}

void SdfAabbTree::insert_leaf(int leaf)
{
  if (root_ == null_node) {
    root_ = leaf;
    nodes_[root_].parent = null_node;
    return;
  }

  SdfAabb leaf_bounds = nodes_[leaf].bounds;
  int index = root_;
  while (!nodes_[index].is_leaf()) {
    int child_a = nodes_[index].child_a;
    int child_b = nodes_[index].child_b;

    float area = nodes_[index].bounds.half_area();

    SdfAabb combined_bounds = SdfAabb::merge(nodes_[index].bounds, leaf_bounds);
    float combined_area = combined_bounds.half_area();

    float cost = 2.0f * combined_area;
    float inheritance_cost = 2.0f * (combined_area - area);

    float cost_a;
    if (nodes_[child_a].is_leaf()) {
      SdfAabb bounds = SdfAabb::merge(leaf_bounds, nodes_[child_a].bounds);
      cost_a = bounds.half_area() + inheritance_cost;
    }
    else {
      SdfAabb bounds = SdfAabb::merge(leaf_bounds, nodes_[child_a].bounds);
      float old_area = nodes_[child_a].bounds.half_area();
      float new_area = bounds.half_area();
      cost_a = (new_area - old_area) + inheritance_cost;
    }

    float cost_b;
    if (nodes_[child_b].is_leaf()) {
      SdfAabb bounds = SdfAabb::merge(leaf_bounds, nodes_[child_b].bounds);
      cost_b = bounds.half_area() + inheritance_cost;
    }
    else {
      SdfAabb bounds = SdfAabb::merge(leaf_bounds, nodes_[child_b].bounds);
      float old_area = nodes_[child_b].bounds.half_area();
      float new_area = bounds.half_area();
      cost_b = (new_area - old_area) + inheritance_cost;
    }

    if (cost < cost_a && cost < cost_b) {
      break;
    }

    index = (cost_a < cost_b) ? child_a : child_b;
  }

  int sibling = index;

  int old_parent = nodes_[sibling].parent;
  int new_parent = allocate_node();
  nodes_[new_parent].parent = old_parent;
  nodes_[new_parent].bounds = SdfAabb::merge(leaf_bounds, nodes_[sibling].bounds);
  nodes_[new_parent].height = nodes_[sibling].height + 1;

  if (old_parent != null_node) {
    if (nodes_[old_parent].child_a == sibling) {
      nodes_[old_parent].child_a = new_parent;
    }
    else {
      nodes_[old_parent].child_b = new_parent;
    }

    nodes_[new_parent].child_a = sibling;
    nodes_[new_parent].child_b = leaf;
    nodes_[sibling].parent = new_parent;
    nodes_[leaf].parent = new_parent;
  }
  else {
    nodes_[new_parent].child_a = sibling;
    nodes_[new_parent].child_b = leaf;
    nodes_[sibling].parent = new_parent;
    nodes_[leaf].parent = new_parent;
    root_ = new_parent;
  }

  index = nodes_[leaf].parent;
  while (index != null_node) {
    index = balance(index);

    int child_a = nodes_[index].child_a;
    int child_b = nodes_[index].child_b;
    nodes_[index].height = 1 + math::max(nodes_[child_a].height, nodes_[child_b].height);
    nodes_[index].bounds = SdfAabb::merge(nodes_[child_a].bounds, nodes_[child_b].bounds);

    index = nodes_[index].parent;
  }
}

void SdfAabbTree::remove_leaf(int leaf)
{
  if (leaf == root_) {
    root_ = null_node;
    return;
  }

  int parent = nodes_[leaf].parent;
  int grand_parent = nodes_[parent].parent;
  int sibling = nodes_[parent].child_a == leaf ? nodes_[parent].child_b : nodes_[parent].child_a;

  if (grand_parent != null_node) {
    if (nodes_[grand_parent].child_a == parent) {
      nodes_[grand_parent].child_a = sibling;
    }
    else {
      nodes_[grand_parent].child_b = sibling;
    }
    nodes_[sibling].parent = grand_parent;
    free_node(parent);

    int index = grand_parent;
    while (index != null_node) {
      index = balance(index);

      int child_a = nodes_[index].child_a;
      int child_b = nodes_[index].child_b;

      nodes_[index].bounds = SdfAabb::merge(nodes_[child_a].bounds, nodes_[child_b].bounds);
      nodes_[index].height = 1 + math::max(nodes_[child_a].height, nodes_[child_b].height);

      index = nodes_[index].parent;
    }
  }
  else {
    root_ = sibling;
    nodes_[sibling].parent = null_node;
    free_node(parent);
  }
}

int SdfAabbTree::balance(int a)
{
  if (nodes_[a].is_leaf() || nodes_[a].height < 2) {
    return a;
  }

  int b = nodes_[a].child_a;
  int c = nodes_[a].child_b;

  int balance = nodes_[c].height - nodes_[b].height;

  if (balance > 1) {
    int f = nodes_[c].child_a;
    int g = nodes_[c].child_b;

    nodes_[c].child_a = a;
    nodes_[c].parent = nodes_[a].parent;
    nodes_[a].parent = c;

    if (nodes_[c].parent != null_node) {
      if (nodes_[nodes_[c].parent].child_a == a) {
        nodes_[nodes_[c].parent].child_a = c;
      }
      else {
        nodes_[nodes_[c].parent].child_b = c;
      }
    }
    else {
      root_ = c;
    }

    if (nodes_[f].height > nodes_[g].height) {
      nodes_[c].child_b = f;
      nodes_[a].child_b = g;
      nodes_[g].parent = a;
      nodes_[a].bounds = SdfAabb::merge(nodes_[b].bounds, nodes_[g].bounds);
      nodes_[c].bounds = SdfAabb::merge(nodes_[a].bounds, nodes_[f].bounds);

      nodes_[a].height = 1 + math::max(nodes_[b].height, nodes_[g].height);
      nodes_[c].height = 1 + math::max(nodes_[a].height, nodes_[f].height);
    }
    else {
      nodes_[c].child_b = g;
      nodes_[a].child_b = f;
      nodes_[f].parent = a;
      nodes_[a].bounds = SdfAabb::merge(nodes_[b].bounds, nodes_[f].bounds);
      nodes_[c].bounds = SdfAabb::merge(nodes_[a].bounds, nodes_[g].bounds);

      nodes_[a].height = 1 + math::max(nodes_[b].height, nodes_[f].height);
      nodes_[c].height = 1 + math::max(nodes_[a].height, nodes_[g].height);
    }

    return c;
  }

  if (balance < -1) {
    int d = nodes_[b].child_a;
    int e = nodes_[b].child_b;

    nodes_[b].child_a = a;
    nodes_[b].parent = nodes_[a].parent;
    nodes_[a].parent = b;

    if (nodes_[b].parent != null_node) {
      if (nodes_[nodes_[b].parent].child_a == a) {
        nodes_[nodes_[b].parent].child_a = b;
      }
      else {
        nodes_[nodes_[b].parent].child_b = b;
      }
    }
    else {
      root_ = b;
    }

    if (nodes_[d].height > nodes_[e].height) {
      nodes_[b].child_b = d;
      nodes_[a].child_a = e;
      nodes_[e].parent = a;
      nodes_[a].bounds = SdfAabb::merge(nodes_[c].bounds, nodes_[e].bounds);
      nodes_[b].bounds = SdfAabb::merge(nodes_[a].bounds, nodes_[d].bounds);

      nodes_[a].height = 1 + math::max(nodes_[c].height, nodes_[e].height);
      nodes_[b].height = 1 + math::max(nodes_[a].height, nodes_[d].height);
    }
    else {
      nodes_[b].child_b = e;
      nodes_[a].child_a = d;
      nodes_[d].parent = a;
      nodes_[a].bounds = SdfAabb::merge(nodes_[c].bounds, nodes_[d].bounds);
      nodes_[b].bounds = SdfAabb::merge(nodes_[a].bounds, nodes_[e].bounds);

      nodes_[a].height = 1 + math::max(nodes_[c].height, nodes_[d].height);
      nodes_[b].height = 1 + math::max(nodes_[a].height, nodes_[e].height);
    }

    return b;
  }

  return a;
}

Vector<SdfAabbNodeGPU> SdfAabbTree::build_gpu_nodes(float tighten_radius) const
{
  Vector<SdfAabbNodeGPU> gpu_nodes(nodes_.size());
  for (int i = 0; i < nodes_.size(); ++i) {
    gpu_nodes[i] = {};
  }

  for (int i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].is_free()) {
      continue;
    }

    SdfAabb tight_bounds = nodes_[i].bounds;
    tight_bounds.expand(-tighten_radius);

    gpu_nodes[i].bounds_min = float4(tight_bounds.min, 0.0f);
    gpu_nodes[i].bounds_max = float4(tight_bounds.max, 0.0f);
    gpu_nodes[i].parent = nodes_[i].parent;
    gpu_nodes[i].child_a = nodes_[i].child_a;
    gpu_nodes[i].child_b = nodes_[i].child_b;
    gpu_nodes[i].shape_index = nodes_[i].shape_index;
  }

  return gpu_nodes;
}

/* -------------------------------------------------------------------- */
/* SdfBlobTree */

int SdfBlobTree::add_leaf(int prim_index, const SDFObjectGPU &obj)
{
  int idx = int(nodes_.size());
  Node n;
  n.node_type = 0;
  n.prim_index = prim_index;
  n.original_index = obj.original_index;
  n.color = obj.color;
  n.height = 0;
  nodes_.append(n);
  return idx;
}

int SdfBlobTree::add_operator(int left, int right, const SDFObjectGPU &right_obj)
{
  int idx = int(nodes_.size());
  Node n;
  n.node_type = 1;
  n.left = left;
  n.right = right;
  n.prim_index = -1;
  n.csg_operation = right_obj.csg_operation;
  n.blend_type = right_obj.blend_type;
  n.blend = right_obj.blend;
  n.shell_distance = right_obj.shell_distance;
  n.shell_mode = right_obj.shell_mode;
  n.shell_op = right_obj.shell_op;
  n.shell_blend_top = right_obj.shell_blend_top;
  n.shell_blend_bottom = right_obj.shell_blend_bottom;
  n.chamfer_k2 = right_obj.chamfer_k2;
  n.chamfer_k3 = right_obj.chamfer_k3;
  n.chamfer_k4 = right_obj.chamfer_k4;
  n.chamfer_k5 = right_obj.chamfer_k5;
  n.flip_blend = right_obj.flip_blend;
  n.flip_blend_end = right_obj.flip_blend_end;
  n.original_index = -1;
  n.color = float4(1.0f);
  n.height = std::max(nodes_[left].height, nodes_[right].height) + 1;
  nodes_[left].parent = idx;
  nodes_[right].parent = idx;
  nodes_.append(n);
  return idx;
}

int SdfBlobTree::add_operator_from_group(int left, int right, const SDFGroupGPU &grp)
{
  int idx = int(nodes_.size());
  Node n;
  n.node_type = 1;
  n.left = left;
  n.right = right;
  n.prim_index = -1;
  n.csg_operation = grp.csg_operation;
  n.blend_type = grp.blend_type;
  n.blend = grp.blend;
  n.shell_distance = grp.shell_distance;
  n.shell_mode = grp.shell_mode;
  n.shell_op = grp.shell_op;
  n.shell_blend_top = grp.shell_blend_top;
  n.shell_blend_bottom = grp.shell_blend_bottom;
  n.chamfer_k2 = grp.chamfer_k2;
  n.chamfer_k3 = grp.chamfer_k3;
  n.chamfer_k4 = grp.chamfer_k4;
  n.chamfer_k5 = grp.chamfer_k5;
  n.flip_blend = grp.flip_blend;
  n.flip_blend_end = grp.flip_blend_end;
  n.original_index = -1;
  n.color = grp.color;
  n.height = std::max(nodes_[left].height, nodes_[right].height) + 1;
  nodes_[left].parent = idx;
  nodes_[right].parent = idx;
  nodes_.append(n);
  return idx;
}

int SdfBlobTree::add_warp(int child, const SDFGroupGPU &grp)
{
  int idx = int(nodes_.size());
  Node n;
  n.node_type = 2;
  n.left = child;
  n.right = -1;
  n.prim_index = -1;
  n.modifier_start = grp.modifier_start;
  n.modifier_count = grp.modifier_count;
  n.original_index = -1;
  n.color = float4(1.0f);
  n.height = nodes_[child].height + 1;
  nodes_[child].parent = idx;
  nodes_.append(n);
  return idx;
}

int SdfBlobTree::add_union_op(int left, int right, float blend, int blend_type)
{
  int idx = int(nodes_.size());
  Node n;
  n.node_type = 1;
  n.left = left;
  n.right = right;
  n.prim_index = -1;
  n.csg_operation = 0; /* SDF_CSG_UNION */
  n.blend_type = blend_type;
  n.blend = blend;
  n.original_index = -1;
  n.color = float4(1.0f);
  n.height = std::max(nodes_[left].height, nodes_[right].height) + 1;
  nodes_[left].parent = idx;
  nodes_[right].parent = idx;
  nodes_.append(n);
  return idx;
}

/* Balanced spatial BVH for a set of union objects.
 * Median-split along the longest AABB axis. */
int SdfBlobTree::build_spatial_bvh(const SDFObjectGPU *objects,
                                   const int *indices,
                                   int count,
                                   float max_blend,
                                   int blend_type)
{
  if (count == 1) {
    return add_leaf(indices[0], objects[indices[0]]);
  }
  if (count == 2) {
    int l = add_leaf(indices[0], objects[indices[0]]);
    int r = add_leaf(indices[1], objects[indices[1]]);
    return add_union_op(l, r, max_blend, blend_type);
  }

  /* Find combined AABB and longest axis */
  float3 combined_min(1e30f), combined_max(-1e30f);
  for (int i = 0; i < count; i++) {
    const SDFObjectGPU &obj = objects[indices[i]];
    combined_min = math::min(combined_min, float3(obj.orig_bbox_min));
    combined_max = math::max(combined_max, float3(obj.orig_bbox_max));
  }
  float3 extent = combined_max - combined_min;
  int axis = 0;
  if (extent.y > extent.x && extent.y > extent.z) { axis = 1; }
  else if (extent.z > extent.x) { axis = 2; }

  /* Sort by AABB center along the longest axis */
  Vector<std::pair<float, int>> sorted(count);
  for (int i = 0; i < count; i++) {
    const SDFObjectGPU &obj = objects[indices[i]];
    float center = (float3(obj.orig_bbox_min)[axis] + float3(obj.orig_bbox_max)[axis]) * 0.5f;
    sorted[i] = {center, indices[i]};
  }
  std::sort(sorted.begin(), sorted.end());

  /* Split at median */
  int mid = count / 2;
  Vector<int> left_idx(mid), right_idx(count - mid);
  for (int i = 0; i < mid; i++) { left_idx[i] = sorted[i].second; }
  for (int i = mid; i < count; i++) { right_idx[i - mid] = sorted[i].second; }

  int l = build_spatial_bvh(objects, left_idx.data(), mid, max_blend, blend_type);
  int r = build_spatial_bvh(objects, right_idx.data(), count - mid, max_blend, blend_type);
  return add_union_op(l, r, max_blend, blend_type);
}

/* Build subtree from a range of sorted objects with spatial BVH for unions.
 * Collects consecutive union objects into spatial BVH subtrees.
 * Non-union operations are applied at the correct hierarchy level. */
int SdfBlobTree::build_left_fold(const SDFObjectGPU *objects,
                                 const int *indices,
                                 int count)
{
  if (count == 0) {
    return -1;
  }
  if (count == 1) {
    return add_leaf(indices[0], objects[indices[0]]);
  }

  /* Collect pending union objects, flush as spatial BVH when hitting non-union */
  Vector<int> pending_unions;
  int accumulated = -1;

  auto flush_unions = [&]() {
    if (pending_unions.is_empty()) {
      return;
    }
    float max_blend = 0.0f;
    int blend_type = 0;
    for (int idx : pending_unions) {
      if (objects[idx].blend > max_blend) {
        max_blend = objects[idx].blend;
        blend_type = objects[idx].blend_type;
      }
    }
    int subtree;
    if (int(pending_unions.size()) >= 3) {
      subtree = build_spatial_bvh(
          objects, pending_unions.data(), int(pending_unions.size()), max_blend, blend_type);
    }
    else if (int(pending_unions.size()) == 2) {
      int l = add_leaf(pending_unions[0], objects[pending_unions[0]]);
      int r = add_leaf(pending_unions[1], objects[pending_unions[1]]);
      subtree = add_union_op(l, r, max_blend, blend_type);
    }
    else {
      subtree = add_leaf(pending_unions[0], objects[pending_unions[0]]);
    }

    if (accumulated < 0) {
      accumulated = subtree;
    }
    else {
      accumulated = add_union_op(accumulated, subtree, max_blend, blend_type);
    }
    pending_unions.clear();
  };

  for (int i = 0; i < count; i++) {
    int obj_idx = indices[i];
    bool is_union = (objects[obj_idx].csg_operation == 0);

    if (is_union) {
      pending_unions.append(obj_idx);
    }
    else {
      /* Non-union: flush pending unions first, then apply this operation */
      flush_unions();
      int leaf = add_leaf(obj_idx, objects[obj_idx]);
      if (accumulated < 0) {
        /* Nothing to apply to — just a leaf (shouldn't happen in practice) */
        accumulated = leaf;
      }
      else {
        accumulated = add_operator(accumulated, leaf, objects[obj_idx]);
      }
    }
  }

  /* Flush remaining unions */
  flush_unions();
  return accumulated;
}

void SdfBlobTree::build_from_scene(const Vector<SDFObjectGPU> &objects,
                                   const Vector<SDFGroupGPU> &groups)
{
  clear();
  if (objects.is_empty()) {
    return;
  }

  /* Collect scene-level entities: group subtrees and ungrouped objects.
   * Process in sorted order to preserve CSG evaluation semantics. */
  Vector<int> scene_roots;

  int i = 0;
  int n = int(objects.size());
  while (i < n) {
    int gid = objects[i].group_id;

    if (gid >= 0 && gid < int(groups.size())) {
      /* Group: build subtree from contiguous members */
      int grp_start = i;
      while (i < n && objects[i].group_id == gid) {
        i++;
      }
      int grp_count = i - grp_start;

      Vector<int> member_indices(grp_count);
      for (int m = 0; m < grp_count; m++) {
        member_indices[m] = grp_start + m;
      }

      int grp_root = build_left_fold(objects.data(), member_indices.data(), grp_count);
      if (grp_root < 0) {
        continue;
      }

      /* Apply group tint to the subtree root */
      if (nodes_[grp_root].is_operator()) {
        nodes_[grp_root].color = groups[gid].color;
      }

      /* Wrap in warp node if group has modifiers */
      if (groups[gid].modifier_count > 0) {
        grp_root = add_warp(grp_root, groups[gid]);
      }

      scene_roots.append(grp_root);
    }
    else {
      /* Ungrouped object */
      int leaf = add_leaf(i, objects[i]);
      scene_roots.append(leaf);
      i++;
    }
  }

  if (scene_roots.is_empty()) {
    return;
  }

  /* Combine scene-level entities: collect union entities for spatial BVH,
   * apply non-union entities (groups with subtract/intersect) in order. */
  Vector<int> pending_union_roots;
  int scene_root = -1;

  auto find_group_id = [&](int entity_idx) -> int {
    int walk = entity_idx;
    while (walk >= 0 && !nodes_[walk].is_leaf()) {
      walk = nodes_[walk].left;
    }
    if (walk >= 0) {
      return objects[nodes_[walk].prim_index].group_id;
    }
    return -1;
  };

  auto get_entity_csg = [&](int entity_idx) -> int {
    Node &en = nodes_[entity_idx];
    if (en.is_leaf()) {
      return objects[en.prim_index].csg_operation;
    }
    int gid = find_group_id(entity_idx);
    if (gid >= 0 && gid < int(groups.size())) {
      return groups[gid].csg_operation;
    }
    return 0;
  };

  auto flush_union_roots = [&]() {
    if (pending_union_roots.is_empty()) {
      return;
    }
    int subtree;
    if (int(pending_union_roots.size()) == 1) {
      subtree = pending_union_roots[0];
    }
    else {
      /* Union the pending roots with a simple binary chain
       * (they're already spatial subtrees internally) */
      subtree = pending_union_roots[0];
      for (int j = 1; j < int(pending_union_roots.size()); j++) {
        subtree = add_union_op(subtree, pending_union_roots[j], 0.0f, 0);
      }
    }
    if (scene_root < 0) {
      scene_root = subtree;
    }
    else {
      scene_root = add_union_op(scene_root, subtree, 0.0f, 0);
    }
    pending_union_roots.clear();
  };

  for (int s = 0; s < int(scene_roots.size()); s++) {
    int entity = scene_roots[s];
    int csg = (s == 0) ? 0 : get_entity_csg(entity);

    if (csg == 0) {
      /* Union: collect for spatial grouping */
      pending_union_roots.append(entity);
    }
    else {
      /* Non-union: flush pending unions, then apply this operation */
      flush_union_roots();
      Node &en = nodes_[entity];
      if (en.is_leaf()) {
        if (scene_root < 0) {
          scene_root = entity;
        }
        else {
          scene_root = add_operator(scene_root, entity, objects[en.prim_index]);
        }
      }
      else {
        int gid = find_group_id(entity);
        if (scene_root < 0) {
          scene_root = entity;
        }
        else if (gid >= 0 && gid < int(groups.size())) {
          scene_root = add_operator_from_group(scene_root, entity, groups[gid]);
        }
        else {
          SDFObjectGPU dummy = {};
          dummy.csg_operation = 0;
          scene_root = add_operator(scene_root, entity, dummy);
        }
      }
    }
  }
  flush_union_roots();

  /* Compute subtree AABBs bottom-up */
  compute_subtree_aabbs(scene_root, objects);

  /* Left-heavy optimization */
  make_left_heavy(scene_root);
}

void SdfBlobTree::compute_subtree_aabbs(int node_idx,
                                        const Vector<SDFObjectGPU> &objects)
{
  if (node_idx < 0) {
    return;
  }
  Node &n = nodes_[node_idx];

  if (n.is_leaf()) {
    const SDFObjectGPU &obj = objects[n.prim_index];
    n.subtree_aabb = SdfAabb(float3(obj.orig_bbox_min), float3(obj.orig_bbox_max));
    return;
  }

  if (n.left >= 0) {
    compute_subtree_aabbs(n.left, objects);
  }
  if (n.right >= 0) {
    compute_subtree_aabbs(n.right, objects);
  }

  if (n.is_warp()) {
    n.subtree_aabb = nodes_[n.left].subtree_aabb;
    /* Conservative expansion for warps */
    n.subtree_aabb.expand(0.5f);
  }
  else {
    /* Binary op: union of children AABBs, expanded by blend */
    n.subtree_aabb = SdfAabb::merge(
        (n.left >= 0) ? nodes_[n.left].subtree_aabb : SdfAabb(),
        (n.right >= 0) ? nodes_[n.right].subtree_aabb : SdfAabb());

    float blend_pad = std::max({fabsf(n.blend),
                                fabsf(n.shell_distance),
                                n.shell_blend_top,
                                n.shell_blend_bottom});
    if (blend_pad > 0.001f) {
      n.subtree_aabb.expand(blend_pad * 2.0f);
    }
  }
}

void SdfBlobTree::make_left_heavy(int node_idx)
{
  if (node_idx < 0) {
    return;
  }
  Node &n = nodes_[node_idx];
  if (n.is_leaf() || n.is_warp()) {
    if (n.is_warp() && n.left >= 0) {
      make_left_heavy(n.left);
    }
    return;
  }

  make_left_heavy(n.left);
  make_left_heavy(n.right);

  /* Swap if right subtree is taller and operator is commutative */
  if (n.right >= 0 && n.left >= 0) {
    bool commutative = (n.csg_operation == 0 /* UNION */ ||
                        n.csg_operation == 2 /* INTERSECT */);
    if (commutative && nodes_[n.right].height > nodes_[n.left].height) {
      std::swap(n.left, n.right);
      n.height = std::max(nodes_[n.left].height, nodes_[n.right].height) + 1;
    }
  }
}

int SdfBlobTree::linearize_node(int node_idx,
                                Vector<BlobTreeNodeGPU> &out,
                                int warp_depth) const
{
  if (node_idx < 0) {
    return -1;
  }
  const Node &n = nodes_[node_idx];

  int cur_warp = warp_depth;
  if (n.is_warp()) {
    cur_warp++;
  }

  /* Post-order: left, right, self */
  int left_po = -1, right_po = -1;
  if (n.left >= 0) {
    left_po = linearize_node(n.left, out, n.is_warp() ? cur_warp : warp_depth);
  }
  if (n.right >= 0) {
    right_po = linearize_node(n.right, out, warp_depth);
  }

  int my_po = int(out.size());
  BlobTreeNodeGPU gpu = {};
  gpu.subtree_aabb_min = float4(n.subtree_aabb.min, 0.0f);
  gpu.subtree_aabb_max = float4(n.subtree_aabb.max, 0.0f);
  gpu.node_type = n.node_type;
  gpu.prim_index = n.prim_index;
  gpu.left_child = left_po;
  gpu.right_child = right_po;
  gpu.csg_operation = n.csg_operation;
  gpu.blend_type = n.blend_type;
  gpu.blend = n.blend;
  gpu.shell_distance = n.shell_distance;
  gpu.shell_mode = n.shell_mode;
  gpu.shell_op = n.shell_op;
  gpu.shell_blend_top = n.shell_blend_top;
  gpu.shell_blend_bottom = n.shell_blend_bottom;
  gpu.chamfer_k2 = n.chamfer_k2;
  gpu.chamfer_k3 = n.chamfer_k3;
  gpu.chamfer_k4 = n.chamfer_k4;
  gpu.chamfer_k5 = n.chamfer_k5;
  gpu.flip_blend = n.flip_blend;
  gpu.flip_blend_end = n.flip_blend_end;
  gpu.modifier_start = n.modifier_start;
  gpu.modifier_count = n.modifier_count;
  gpu.original_index = n.original_index;
  gpu.warp_depth = n.is_warp() ? cur_warp : warp_depth;
  gpu.color = n.color;

  /* Compute max_subtree_blend for AABB skip threshold */
  float max_blend = std::max({fabsf(n.blend),
                              fabsf(n.shell_distance),
                              n.shell_blend_top,
                              n.shell_blend_bottom});
  if (n.left >= 0 && left_po >= 0) {
    max_blend = std::max(max_blend, out[left_po].subtree_aabb_min.w);
  }
  if (n.right >= 0 && right_po >= 0) {
    max_blend = std::max(max_blend, out[right_po].subtree_aabb_min.w);
  }
  gpu.subtree_aabb_min.w = max_blend;

  out.append(gpu);
  return my_po;
}

Vector<BlobTreeNodeGPU> SdfBlobTree::linearize_postorder(int &out_root_index) const
{
  Vector<BlobTreeNodeGPU> result;
  if (nodes_.is_empty()) {
    out_root_index = -1;
    return result;
  }

  /* Find root (node with no parent) */
  int root = -1;
  for (int i = int(nodes_.size()) - 1; i >= 0; i--) {
    if (nodes_[i].parent < 0) {
      root = i;
      break;
    }
  }
  if (root < 0) {
    out_root_index = -1;
    return result;
  }

  result.reserve(int(nodes_.size()));
  out_root_index = linearize_node(root, result, 0);
  return result;
}

}  // namespace blender::draw::sdf
