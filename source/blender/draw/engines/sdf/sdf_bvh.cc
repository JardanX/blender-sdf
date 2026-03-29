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

}  // namespace blender::draw::sdf
