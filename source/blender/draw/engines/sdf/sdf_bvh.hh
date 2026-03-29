/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

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

}  // namespace blender::draw::sdf
