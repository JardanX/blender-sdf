# Blobtree Implementation — Status & Next Steps

## What's Done (Working)

### Per-object AABB Skip (ACTIVE — in flat eval)
- `sdf_engine.cc`: Per-object `max_group_blend` threshold instead of group-wide max
- `sdf_engine.cc`: Individual `orig_bbox_min/max` preserved for group members (not overwritten to combined group AABB)
- `sdf_trace_comp.glsl`: Subtract operations no longer force-evaluated (only intersect forces eval)
- `sdf_trace_comp.glsl`: AABB skip added inside `evalSceneBVH` group loop
- Same fixes in `sdf_color_resolve_comp.glsl`, `sdf_cone_march_comp.glsl`

### Compact Blend Operators (ACTIVE — in `sdf_lib.glsl`)
- `compactBlendFactor()` function: smoothly reduces blend k to 0 at distance d_o = k from surface
- Applied in `combineCSG()` for union, subtract, intersect, push, avoid
- Uses t^6 falloff for C5 smoothness
- Prevents field compression from distant objects

### Blobtree Infrastructure (BUILT — not active in rendering)
- **GPU struct**: `BlobTreeNodeGPU` in `sdf_shader_shared.hh` (~160 bytes, 16-aligned)
- **CPU tree builder**: `SdfBlobTree` class in `sdf_bvh.hh/.cc`
  - `build_from_scene()`: builds tree from flat object list + groups
  - `build_left_fold()`: detects union runs, builds spatial BVH (median-split) for 3+ consecutive unions
  - `build_spatial_bvh()`: balanced median-split BVH for union objects
  - `compute_subtree_aabbs()`: bottom-up AABB propagation with blend expansion
  - `make_left_heavy()`: swaps children for commutative ops to minimize stack depth
  - `linearize_postorder()`: serializes to flat GPU array
- **SSBO**: `tree_nodes_ssbo_` uploaded to slot 13, bound in `bind_ssbos()`
- **Push constant**: `tree_node_count` added to trace and cone march shader infos
- **Dump operator**: `OBJECT_OT_sdf_blobtree_dump` (F3: "SDF Dump Blobtree") exports tree to `sdf_blobtree.txt`
- **Debug format**: `sdf_blobtree_format_text()` recursive indented tree printer

### GPU Eval Function (BUILT — reverted, not active)
- `evalSceneTree()` in `sdf_lib.glsl`: top-down recursive with explicit stack
- **Currently disabled** — trace and cone march use flat `evalSceneTile`/`evalSceneCone`
- Switching is one-line: replace `evalSceneTile(...)` with `evalSceneTree(pos, tree_node_count, tree_node_count - 1, ...)`

## What's Wrong with the Current GPU Eval

The `evalSceneTree()` function uses a recursive simulation with 4 arrays × 32 entries:
```glsl
int frame_node[32], frame_state[32];
float frame_left_dist[32], frame_left_id[32];
```

This causes:
1. **128 registers reserved** for the stack arrays — massive register pressure, kills GPU occupancy
2. **`BlobTreeNodeGPU` is ~160 bytes** — reading it from SSBO per stack frame visit is expensive
3. **`for (iter = 0; iter < 512)` loop** — GPU compiler allocates resources for worst-case iteration count
4. **Result: 18 objects at 9.2 FPS** vs 60+ FPS with flat eval

## What Needs to Change

### 1. Slim Tree Node (CRITICAL)

Replace `BlobTreeNodeGPU` (~160 bytes) with a minimal struct (~32-48 bytes):

```cpp
struct BlobTreeNodeSlim {
    float4 aabb_min;      // .w = max_subtree_blend (skip threshold)
    float4 aabb_max;      // .w = packed flags (node_type + blend_type)
    int left_child;       // -1 for leaf
    int right_child;      // -1 for leaf
    int prim_or_op;       // leaf: index into objects[]. op: packed CSG params index
    int blend_data;       // packed: csg_operation(8) | blend_type(8) | blend_f16(16)
};
```

Full CSG params (shell, chamfer, flip_blend etc.) only needed when ACTUALLY COMBINING — read from a separate small SSBO or from `objects[]` at that point. The tree navigation only needs AABB + child pointers + basic op type.

### 2. Iterative Traversal Without State Machine (CRITICAL)

Replace the 4-array recursive simulation with one of:

**Option A: Post-order scan with skip bitmap**
- Single forward scan of linearized nodes (post-order)
- Small bitmap (2-4 uints for up to 128 nodes) marks active subtrees
- Pre-scan: walk tree_nodes top-down checking AABBs, set bits
- Main scan: iterate post-order, skip unset nodes, use a small float stack (depth 8-12)
- Total: ~16 floats for eval stack + 4 uints for bitmap = ~80 bytes

**Option B: Iterative top-down with minimal stack**  
- Stack holds only `{node_index, left_result}` pairs (8 bytes per frame)
- Max depth 16 = 128 bytes
- Visit node → check AABB → push left → when left returns → push right → when right returns → combine → pop
- No state array needed, just a 2-element frame

**Option C (recommended): Hybrid — top-down AABB pre-scan + post-order eval**
- Phase 1 (cheap): iterate tree nodes, check AABBs top-down, build small bitmask of active nodes. Cost: ~1 SSBO read per node, just AABB fields (32 bytes each)
- Phase 2 (eval): iterate post-order, skip inactive nodes, evaluate only active leaves. Cost: 1 SSBO read per active node from `objects[]`
- The bitmask eliminates the recursive stack entirely
- Key: Phase 1 reads only the slim tree node (32 bytes). Phase 2 reads full object data (280 bytes) only for active leaves.

### 3. Spatial BVH for Scene Level (DONE but needs testing)

The current `build_left_fold` already detects union runs and builds spatial BVH. The scene-level combining in `build_from_scene` also collects union entities for spatial grouping. This is correct but untested with the working GPU eval.

### 4. Per-Object Blend in Spatial BVH (MINOR)

Currently spatial BVH union nodes use `max_blend` across all objects in the run. This means two objects with different blends get the same operator blend. Ideally each spatial BVH node should carry the max blend of objects in its subtree, not the global max. This is already close — just needs the `build_spatial_bvh` to propagate per-subtree max blend.

## File Map

| File | What's There | What Needs Change |
|------|-------------|------------------|
| `sdf_shader_shared.hh` | `BlobTreeNodeGPU` struct | Replace with slim version |
| `sdf_bvh.hh` | `SdfBlobTree` class declaration | Add slim node builder |
| `sdf_bvh.cc` | Full tree construction + linearization | Update linearization for slim nodes |
| `sdf_engine.cc:2062` | Tree build + SSBO upload | Update for slim struct size |
| `sdf_engine.cc:bind_ssbos` | Binds slot 13 `tree_nodes` | Keep |
| `sdf_engine.cc:draw_trace` | Sets `tree_node_count` uniform | Keep |
| `sdf_lib.glsl:evalSceneTree` | Top-down recursive eval (BROKEN perf) | **Rewrite** with Option B or C |
| `sdf_trace_comp.glsl:766` | Currently `evalSceneTile` (flat) | Switch to tree when eval is fixed |
| `sdf_cone_march_comp.glsl:197` | Currently `evalSceneCone` (flat) | Switch to tree when eval is fixed |
| `sdf_color_resolve_comp.glsl` | Flat eval | Switch to tree (needs tree SSBO in shader info) |
| `sdf_shader_infos.hh` | Slot 13 on trace + cone march | Add slot 13 to color_resolve, grid_eval, dc_vertex_color |
| `object_sdf.cc` | `OBJECT_OT_sdf_blobtree_dump` operator | Keep |

## Activation Checklist (when GPU eval is fixed)

1. In `sdf_trace_comp.glsl` line 766: replace `evalSceneTile(...)` with tree eval call
2. In `sdf_cone_march_comp.glsl` line 197: replace `evalSceneCone(...)` with tree eval call  
3. Add tree_nodes SSBO + tree_node_count to `sdf_color_resolve_comp`, `sdf_grid_eval_comp`, `sdf_dc_vertex_color_comp` shader infos
4. Update those shaders' eval loops to use tree
5. Update `sdf_cpu_eval.hh` for picking/meshing consistency
6. Remove `evalSceneTile`, `evalSceneBVH`, `evalSceneCone` after full migration
7. Remove `groups[]` SSBO (params now in tree nodes)
8. Remove `object_aabbs[]` SSBO (AABBs now in tree nodes)

## Performance Targets

| Scene | Current (flat eval) | Target (tree eval) |
|-------|--------------------|--------------------|
| 3 SDFs | 60+ FPS | 60+ FPS (no regression) |
| 18 objects, 6 groups | ~30 FPS | 30+ FPS |
| 50 objects, 1 group (compact) | ~6 FPS | 10+ FPS |
| 222 objects (scattered) | ~16 FPS | 30+ FPS |

## Key Papers

- **Grasberger 2016** ("Efficient Data-Parallel Tree-Traversal for BlobTrees"): Post-order linearization, bottom-up traversal, left-heavy optimization. Source: `webhome.cs.uvic.ca/~blob/publications/gdspm.pdf`
- **Zanni 2023** ("Synchronized-tracing of implicit surfaces"): Sparse bottom-up traversal with ancestor pointers, compact blend operators (implemented), tile-based A-buffer. Source: `inria.hal.science/hal-04071989v1`  
- **Galin 2020** ("Segment Tracing Using Local Lipschitz Bounds"): Local Lipschitz bounds per segment for larger trace steps. Planned as follow-up after blobtree. Source: `hal.science/hal-02507361`
