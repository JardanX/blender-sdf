/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "sdf_shader_shared.hh"

namespace blender::draw::sdf {

struct SdfAabb {
  float3 min;
  float3 max;

  SdfAabb() : min(1e30f), max(-1e30f) {}
  SdfAabb(const float3 &min_, const float3 &max_) : min(min_), max(max_) {}

  static SdfAabb merge(const SdfAabb &a, const SdfAabb &b)
  {
    return SdfAabb(math::min(a.min, b.min), math::max(a.max, b.max));
  }

  bool contains(const SdfAabb &b) const
  {
    return min.x <= b.min.x && min.y <= b.min.y && min.z <= b.min.z && max.x >= b.max.x &&
           max.y >= b.max.y && max.z >= b.max.z;
  }

  void include(const float3 &p)
  {
    min = math::min(min, p);
    max = math::max(max, p);
  }

  void expand(float r)
  {
    min -= float3(r);
    max += float3(r);
  }

  float half_area() const
  {
    float3 e = max - min;
    return e.x * e.y + e.y * e.z + e.z * e.x;
  }

  float3 center() const
  {
    return 0.5f * (min + max);
  }

  float3 extents() const
  {
    return max - min;
  }
};

struct SdfAabbNodeGPU {
  float4 bounds_min; /* xyz = min, w = unused */
  float4 bounds_max; /* xyz = max, w = unused */
  int parent;
  int child_a;
  int child_b;
  int shape_index;
};

class SdfAabbTree {
 public:
  static constexpr int null_node = -1;
  static constexpr float fat_bounds_radius = 0.25f;

  struct Node {
    SdfAabb bounds;
    int parent;
    int next_free;
    int child_a;
    int child_b;
    int height;
    int shape_index;
    bool moved;

    bool is_leaf() const
    {
      return child_a == null_node;
    }
    bool is_free() const
    {
      return height < 0;
    }
  };

 private:
  Vector<Node> nodes_;
  int root_;
  int free_list_;

  int allocate_node();
  void free_node(int node);
  void insert_leaf(int leaf);
  void remove_leaf(int leaf);
  int balance(int a);

 public:
  SdfAabbTree();

  int create_proxy(const SdfAabb &bounds, int shape_index);
  void destroy_proxy(int proxy);
  void update_proxy(int proxy, const SdfAabb &bounds);

  void clear();

  int root() const
  {
    return root_;
  }
  const Vector<Node> &nodes() const
  {
    return nodes_;
  }

  Vector<SdfAabbNodeGPU> build_gpu_nodes(float tighten_radius = 0.0f) const;
};

/* Blobtree: binary CSG tree built from the flat object list + groups.
 * Linearized in post-order for GPU evaluation with O(log N) subtree pruning. */
class SdfBlobTree {
 public:
  struct Node {
    int left = -1;
    int right = -1;
    int parent = -1;
    int height = 0;

    /* Leaf: index into objects_ array. Operator: -1. */
    int prim_index = -1;
    /* 0=leaf, 1=binary_op, 2=unary_warp */
    int node_type = 0;

    /* CSG params (for operators) */
    int csg_operation = 0;
    int blend_type = 0;
    float blend = 0.0f;
    float shell_distance = 0.0f;
    int shell_mode = 0;
    int shell_op = 0;
    float shell_blend_top = 0.0f;
    float shell_blend_bottom = 0.0f;
    float chamfer_k2 = 0.0f;
    float chamfer_k3 = 0.0f;
    float chamfer_k4 = 0.0f;
    float chamfer_k5 = 0.0f;
    int flip_blend = 0;
    int flip_blend_end = 0;

    /* Modifier refs (for warp nodes) */
    int modifier_start = 0;
    int modifier_count = 0;

    int original_index = -1;
    int warp_depth = 0;
    float4 color = float4(1.0f);

    SdfAabb subtree_aabb;

    bool is_leaf() const { return node_type == 0; }
    bool is_operator() const { return node_type == 1; }
    bool is_warp() const { return node_type == 2; }
  };

  void clear() { nodes_.clear(); }

  /* Build left-fold subtree from a range of sorted objects. Returns root node index. */
  int build_left_fold(const SDFObjectGPU *objects, const int *indices, int count);

  /* Build entire scene tree from objects + groups. */
  void build_from_scene(const Vector<SDFObjectGPU> &objects,
                        const Vector<SDFGroupGPU> &groups);

  /* Linearize to post-order GPU array. Returns the array and sets root index. */
  Vector<BlobTreeNodeGPU> linearize_postorder(int &out_root_index) const;

  int node_count() const { return int(nodes_.size()); }
  const Vector<Node> &nodes() const { return nodes_; }

 private:
  Vector<Node> nodes_;

  int add_leaf(int prim_index, const SDFObjectGPU &obj);
  int add_union_op(int left, int right, float blend, int blend_type);
  int add_operator(int left, int right, const SDFObjectGPU &right_obj);
  int add_operator_from_group(int left, int right, const SDFGroupGPU &grp);
  int add_warp(int child, const SDFGroupGPU &grp);
  int build_spatial_bvh(const SDFObjectGPU *objects,
                        const int *indices,
                        int count,
                        float max_blend,
                        int blend_type);
  void compute_subtree_aabbs(int node_idx, const Vector<SDFObjectGPU> &objects);
  void make_left_heavy(int node_idx);
  int linearize_node(int node_idx,
                     Vector<BlobTreeNodeGPU> &out,
                     int warp_depth) const;
};

}  // namespace blender::draw::sdf
