# SDF Changes — Blender 5.0 Fork

> What we changed and why, so you don't have to dig through 30+ files.
> Base: Blender `v5.0-release` tag.

---

## Quick Summary

We added **SDF (Signed Distance Field)** as a new native object type in Blender.
This means SDF objects live alongside Meshes, Curves, Volumes, etc. in Blender's core —
not as an addon hack. No rendering yet, just the data foundation.

**What works:** Create SDF from Add menu, shows in Outliner, properties panel,
save/load in .blend files, undo/redo, duplicate, Python API (`bpy.data.sdfs`).

**Stats:** 6 new files, 26 modified files, ~615 lines added.

---

## New Files (6)

### `source/blender/makesdna/DNA_sdf_types.h`
The SDF data structure. Defines what an SDF object stores:
- Primitive type (Box, Sphere, Capsule, Torus)
- Size, bevel, color
- CSG operation (Union, Subtract, Intersect) and blend settings
- Material slots

### `source/blender/makesdna/DNA_sdf_defaults.h`
Default values when creating a new SDF (size=1, white color, no bevel, etc.)

### `source/blender/blenkernel/BKE_sdf.hh`
Public C++ header. Declares `BKE_sdf_add()` (create new SDF) and `BKE_sdf_data_update()` (no-op for now).

### `source/blender/blenkernel/intern/sdf.cc`
The core SDF kernel code. Registers `IDType_ID_SF` with all the callbacks Blender
needs to manage SDF data-blocks:
- `sdf_init_data` — set defaults on creation
- `sdf_copy_data` — duplicate materials + runtime
- `sdf_free_data` — clean up memory
- `sdf_blend_write` / `sdf_blend_read_data` — save/load to .blend files
- `sdf_foreach_id` — iterate material references

### `source/blender/makesrna/intern/rna_sdf.cc`
Python API property definitions. Exposes all SDF fields to Python/UI:
`sdf_type`, `size`, `bevel`, `color`, `blend`, `blend_type`, `csg_operation`, `materials`.

### `source/blender/editors/object/object_sdf.cc`
The "Add SDF" operator (`OBJECT_OT_sdf_add`). Called from the Add menu.

---

## Modified Files (26)

### Data Layer (DNA) — How Blender knows SDF exists

| File | What we added |
|------|---------------|
| `makesdna/DNA_object_types.h` | `OB_SDF = 31` in the object type enum. Added `ID_SF` to the "which ID types can be object data" macros. |
| `makesdna/DNA_ID_enums.h` | `ID_SF = MAKE_ID2('S', 'F')` — the two-character ID code for SDF. |
| `makesdna/DNA_ID.h` | `FILTER_ID_SF` bit flag for filtering, `INDEX_ID_SF` for array indexing. |
| `makesdna/DNA_userdef_enums.h` | `USER_DUP_SDF` flag so "Duplicate Data" preferences include SDF. |
| `makesdna/intern/dna_defaults.c` | Registered SDF in the DNA defaults system (required for `MEMCPY_STRUCT_AFTER`). |

### Kernel (BKE) — How Blender manages SDF data

| File | What we added |
|------|---------------|
| `blenkernel/BKE_idtype.hh` | `extern IDTypeInfo IDType_ID_SF` declaration. |
| `blenkernel/intern/idtype.cc` | Registered SDF in three places: `INIT_TYPE(ID_SF)`, plus `CASE_IDINDEX(SF)` in both the id-code-to-index and id-filter-to-index switch statements. Missing these caused a crash. |
| `blenkernel/intern/object.cc` | Added `case OB_SDF` in 4 switch statements: object name defaults, object data creation, ID-to-type mapping, and duplication. |
| `blenkernel/BKE_main.hh` | `ListBase sdfs = {}` — the linked list that stores all SDF data-blocks in memory. |
| `blenkernel/intern/main.cc` | `lb[INDEX_ID_SF] = &bmain.sdfs` in `BKE_main_lists_get()`. Without this, freeing Main crashes (NULL pointer in the list array). |
| `blenkernel/CMakeLists.txt` | Added `sdf.cc` and `BKE_sdf.hh` to the build. |

### Python API (RNA) — How Python/UI accesses SDF

| File | What we added |
|------|---------------|
| `makesrna/intern/rna_internal.hh` | Function prototypes for `RNA_def_sdf` and `RNA_def_main_sdfs`. |
| `makesrna/intern/rna_object.cc` | SDF entry in the object type dropdown enum. |
| `makesrna/intern/rna_main.cc` | `bpy.data.sdfs` collection (list/iterate SDF data-blocks). |
| `makesrna/intern/rna_main_api.cc` | `bpy.data.sdfs.new()` / `.remove()` / `.tag()` functions. Also added `#include "DNA_sdf_types.h"` (missing this caused a build error). |
| `makesrna/intern/makesrna.cc` | Registered `rna_sdf.cc` in the RNA code generator. |
| `makesrna/intern/rna_ID.cc` | Added `case ID_SF: return &RNA_SDF;` in `ID_code_to_RNA_type()` so SDF data resolves to proper RNA type (fixes empty Object Data Properties panel). |
| `makesrna/intern/CMakeLists.txt` | Added `rna_sdf.cc` to build. |

### Editor — The "Add SDF" button

| File | What we added |
|------|---------------|
| `editors/object/object_intern.hh` | Operator function declaration. |
| `editors/object/object_ops.cc` | Registered the operator so Blender knows about it. |
| `editors/object/CMakeLists.txt` | Added `object_sdf.cc` to build. |

### Draw System — Prevent crashes when rendering SDF objects

| File | What we added |
|------|---------------|
| `draw/intern/draw_cache.cc` | `case OB_SDF: break;` in batch cache validation switch. |
| `draw/engines/overlay/overlay_sculpt.hh` | Added missing `default: break;` to an `ob->type` switch. |

### Depsgraph — Dependency tracking

| File | What we added |
|------|---------------|
| `depsgraph/intern/builder/deg_builder_nodes.cc` | `case OB_SDF:` and `case ID_SF:` so the depsgraph builds nodes for SDF objects. |
| `depsgraph/intern/builder/deg_builder_relations.cc` | Same — added SDF cases for dependency relation building. |

### Other

| File | What we added |
|------|---------------|
| `blentranslation/BLT_translation.hh` | Translation context string `BLT_I18NCONTEXT_ID_SDF`. |
| `source/blender/CMakeLists.txt` | Added `DNA_sdf_types.h` and `DNA_sdf_defaults.h` to the top-level DNA header lists. Missing this caused a build error (`_SDNA_TYPE_SDF` undefined). |
| `scripts/startup/bl_ui/space_view3d.py` | Added "SDF" entry to the Add menu (between Volume and Grease Pencil). |
| `release/datafiles/userdef/userdef_default.c` | Added `USER_DUP_SDF` to factory default `dupflag` so duplicated SDF objects get independent data blocks. |
| `blenloader/intern/versioning_userdef.cc` | Added versioning block (500.122) to set `USER_DUP_SDF` in existing user preferences. |

---

## Bugs Found & Fixed

1. **Startup crash** — `BKE_main_lists_get()` didn't map `INDEX_ID_SF` to `bmain.sdfs`. The ListBase array had a NULL slot, crashing during `Main::~Main`.
2. **Add SDF crash** — `BKE_idtype_idcode_to_index()` was missing `CASE_IDINDEX(SF)`. ID_SF lookups returned -1, so `BKE_libblock_alloc_notest` returned NULL.
3. **Build error** — `DNA_sdf_types.h` wasn't in `SRC_DNA_INC`, so `makesdna` didn't generate `_SDNA_TYPE_SDF`.
4. **Build error** — `rna_main_api.cc` used `SDF` type without `#include "DNA_sdf_types.h"`.
5. **Empty Object Data Properties panel** — `ID_code_to_RNA_type()` in `rna_ID.cc` was missing `case ID_SF: return &RNA_SDF;`. SDF data resolved as generic `ID`, making `context.sdf` always None.
6. **SDF data shared on duplicate** — `USER_DUP_SDF` was defined but missing from both factory defaults (`userdef_default.c`) and user preferences versioning (`versioning_userdef.cc`). Duplicated SDF objects shared a single data block.

---

## Metaball Removal (Phase 2 — SDF replaces Metaball)

SDF replaces Metaball in the fork. All metaball runtime code is removed. DNA enum values
(`OB_MBALL=5`, `ID_MB`) are kept as tombstones for .blend file compatibility.
The SDF entry now occupies metaball's former position in the Add menu.

**Stats:** ~70 files modified, ~15 files deleted, ~4000 lines removed.

### Deleted Files (15)

| File | Was |
|------|-----|
| `blenkernel/BKE_mball.hh` | Public metaball API header |
| `blenkernel/BKE_mball_tessellate.hh` | Tessellation API header |
| `blenkernel/intern/mball_tessellate.cc` | Marching cubes tessellation (~2000 lines) |
| `editors/metaball/mball_edit.cc` | Metaball edit mode operators |
| `editors/metaball/mball_ops.cc` | Metaball operator registration |
| `editors/metaball/editmball_undo.cc` | Metaball edit mode undo |
| `editors/metaball/mball_intern.hh` | Internal metaball editor header |
| `editors/include/ED_mball.hh` | Public metaball editor header |
| `editors/transform/transform_convert_mball.cc` | Transform for metaball elements |
| `editors/space_outliner/tree/tree_element_id_metaball.cc` | Outliner metaball tree element |
| `editors/space_outliner/tree/tree_element_id_metaball.hh` | Outliner metaball tree header |
| `makesrna/intern/rna_meta.cc` | MetaBall RNA property definitions |
| `makesrna/intern/rna_meta_api.cc` | MetaBall Python API functions |
| `io/alembic/exporter/abc_writer_mball.cc/.h` | Alembic metaball exporter |
| `io/usd/intern/usd_writer_metaball.cc/.hh` | USD metaball exporter |
| `scripts/startup/bl_ui/properties_data_metaball.py` | Metaball properties panel |

### Gutted Files (1)

| File | What remains |
|------|-------------|
| `blenkernel/intern/mball.cc` | IDTypeInfo stub only (~140 lines): init/copy/free/foreach_id/blend_write/blend_read |

### Modified Files — DNA

| File | Change |
|------|--------|
| `makesdna/DNA_object_types.h` | Deprecated `OB_MBALL=5` (comment only). Removed from `OB_TYPE_IS_GEOMETRY`, `OB_TYPE_SUPPORT_EDITMODE`, `OB_TYPE_SUPPORT_MATERIAL`. Added `OB_SDF` to material macro. Removed `ID_MB` from `OB_DATA_SUPPORT_EDITMODE`, `OB_DATA_SUPPORT_ID`, `OB_DATA_SUPPORT_ID_CASE`. |
| `makesdna/DNA_ID_enums.h` | Deprecated `ID_MB` (comment only) |

### Modified Files — BKE

| File | Change |
|------|--------|
| `blenkernel/intern/object.cc` | Removed 7 `OB_MBALL` cases + 2 `ID_MB` edit mode cases. Kept `ID_MB` texspace case (file compat). Added `OB_SDF` to `BKE_object_supports_material_slots`. |
| `blenkernel/intern/object_update.cc` | Removed `BKE_mball_data_update` dispatch and batch cache case |
| `blenkernel/intern/object_dupli.cc` | Removed metaball instance guard. Kept `ID_MB` no-draw guard (file compat). |
| `blenkernel/intern/material.cc` | Kept 6 `ID_MB` cases (file compat). Added `OB_SDF`/`ID_SF` support: `BKE_object_material_array_p`, `BKE_object_material_len_p`, `BKE_id_material_array_p`, `BKE_id_material_len_p`, `material_data_index_remove_id`, `material_data_index_clear_id`, `BKE_object_material_slot_used`, `BKE_object_material_remap` — enables material slot assignment on SDF objects |
| `blenkernel/intern/mesh_convert.cc` | Removed `mesh_new_from_mball_object` function and case |
| `blenkernel/intern/lib_remap.cc` | Removed `BKE_mball_is_basis` remapping logic |
| `blenkernel/intern/lib_id.cc` | Removed `OB_MBALL` geometry tag |
| `blenkernel/intern/context.cc` | Removed `CTX_MODE_EDIT_METABALL` case (kept string array entry as tombstone) |
| `blenkernel/CMakeLists.txt` | Removed `mball_tessellate.cc`, `BKE_mball.hh`, `BKE_mball_tessellate.hh` |

### Modified Files — Depsgraph

| File | Change |
|------|--------|
| `depsgraph/intern/builder/deg_builder_nodes.cc` | Removed `ID_MB`/`OB_MBALL` cases, `GEOMETRY_EVAL` operation node |
| `depsgraph/intern/builder/deg_builder_relations.cc` | Removed motherball basis finding, metaball dupli handling, particle mball visualization |
| `depsgraph/intern/eval/deg_eval.cc` | Removed `is_metaball_object_operation` single-thread workaround |
| `depsgraph/intern/eval/deg_eval_copy_on_write.cc` | Removed metaball edit mode COW pointer management |
| `depsgraph/intern/depsgraph_tag.cc` | Removed `ID_MB`/`OB_MBALL` geometry component mapping |
| `depsgraph/intern/depsgraph_query_iter.cc` | Removed `OB_MBALL` visibility override, `ID_MB` dupli check |

### Modified Files — Draw

| File | Change |
|------|--------|
| `draw/engines/overlay/overlay_instance.hh` | Removed `overlay_metaball.hh` include and `Metaballs metaballs` member |
| `draw/engines/overlay/overlay_instance.cc` | Removed 6 metaballs method calls, `OB_MBALL` cases, mball color definitions |
| `draw/engines/overlay/overlay_metaball.hh` | DELETED |
| `draw/engines/overlay/overlay_shader_shared.hh` | Removed 4 mball color float4 fields |
| `draw/engines/overlay/overlay_shape.cc` | Removed `metaball_wire_circle` batch |
| `draw/engines/overlay/overlay_bounds.hh` | Removed `OB_MBALL` basis check, `ID_MB` texspace case |
| `draw/engines/overlay/overlay_private.hh` | Removed `BatchPtr metaball_wire_circle` |
| `draw/intern/draw_resource.hh` | Removed `ID_MB` texspace case |
| `draw/intern/draw_handle.hh` | Removed `OB_MBALL` edit mode check |
| `draw/intern/draw_context.cc` | Removed `OB_MBALL` edit mode block |
| `draw/CMakeLists.txt` | Removed `overlay_metaball.hh` |

### Modified Files — Editors (~30 files)

Removed `OB_MBALL`/`ID_MB` cases from transform, view3d, outliner, object, animation,
screen, info, buttons, render modules. `ED_operator_editmball` gutted to stub (returns false).
Updated 5 CMakeLists.txt files.

| File | Change |
|------|--------|
| `editors/transform/transform_snap_object.cc` | Added `case OB_SDF:` alongside `OB_EMPTY`/`OB_LAMP` for center-snapping support |

### Modified Files — RNA (14 files)

Removed `rna_meta.cc`/`rna_meta_api.cc` from build. Cleaned `ID_MB`/`OB_MBALL`/metaball
references from `rna_object.cc`, `rna_main_api.cc`, `rna_main.cc`, `rna_ID.cc`, `rna_scene.cc`,
`rna_space.cc`, `rna_userdef.cc`, `rna_space_api.cc`, `rna_object_api.cc`, `rna_action.cc`.

| File | Change |
|------|--------|
| `makesrna/intern/rna_ID.cc` | Added `case ID_SF: return &RNA_SDF;` in `ID_code_to_RNA_type()` — required for `ob.data` to resolve as `SDF` type instead of generic `ID`, enabling `context.sdf` in Properties panels |

### Modified Files — IO

Removed Alembic/USD metaball writers. Cleaned hierarchy iterators and hydra object file.

### Modified Files — Other

| File | Change |
|------|--------|
| `windowmanager/intern/wm_keymap_utils.cc` | Removed `CTX_MODE_EDIT_METABALL` keymap and `MBALL_OT` keybindings |
| `windowmanager/intern/wm_init_exit.cc` | Removed `BKE_mball_tessellate.hh` include and `BKE_mball_cubeTable_free()` call |
| `io/freestyle/intern/blender_interface/BlenderFileLoader.cpp` | Removed metaball check |
| `gpencil_modifiers_legacy/intern/MOD_lineart.cc` | Removed metaball check |
| `gpencil_modifiers_legacy/intern/lineart/lineart_cpu.cc` | Removed metaball check |
| `blenloader/tests/blendfile_loading_base_test.cc` | Removed deleted header include and `BKE_mball_cubeTable_free()` call |
| `editors/CMakeLists.txt` | Commented out `add_subdirectory(metaball)` |

### Modified Files — Python UI

| File | Change |
|------|--------|
| `bl_ui/space_view3d.py` | Replaced metaball Add menu entry with SDF (`VIEW3D_MT_sdf_add`). Removed 5 metaball menu classes, mball_edit popover, META selectability filter. |
| `bl_ui/__init__.py` | Removed `properties_data_metaball` module |
| `bl_ui/space_userpref.py` | Removed `duplicate_metaball` preference |
| `bl_ui/space_dopesheet.py` | Removed metaball filter block |

### Kept for File Compatibility

These references remain intentionally — needed to load old .blend files with metaball data:
- `DNA_meta_types.h` — MetaBall struct definition (DNA tombstone)
- `mball.cc` — IDTypeInfo stub with blend_write/blend_read
- `main.cc` — `bmain->metaballs` listbase mapping
- `material.cc` — 6 `ID_MB` cases for material array access
- `object_dupli.cc` — `ID_MB` no-draw guard
- `object.cc` — `ID_MB` texspace accessor
- `anim_sys.cc`, `anim_data_bmain_utils.cc` — animation data iteration
- `idtype.cc` — IDType_ID_MB registration
- `context.cc`/`rna_context.cc` — CTX_MODE_EDIT_METABALL enum mapping
- `versioning_legacy.cc` — .blend file versioning
- `BLT_translation.hh` — translation context string

---

## EEVEE Removal (Phase 3 — MathOPS uses custom SDF renderer)

MathOPS uses its own GPU compute-shader SDF renderer, so EEVEE (Blender's PBR rasterizer)
is dead weight. Entire engine deleted. Workbench (solid/wireframe viewport) kept for mesh preview.

**Stats:** 297 source files deleted, ~100,000 lines removed. Draw CMakeLists.txt lost ~370 lines.

### What was removed

| Category | Scope |
|----------|-------|
| `draw/engines/eevee/` | All 297 C++/header files — the full EEVEE engine |
| `draw/engines/eevee/shaders/CMakeLists.txt` | EEVEE GLSL shader build rules (228 lines) |
| `draw/CMakeLists.txt` | All EEVEE source file entries (~370 lines commented out) |
| `draw/intern/draw_view_data.hh` | `eevee::Instance eevee` member + include removed |
| `draw/intern/draw_context.cc` | EEVEE engine enable logic, `eevee::Instance::init_static()` / `free_static()` calls |
| `draw/engines/workbench/workbench_engine.cc` | Removed `eevee_engine.h` include |
| `draw/engines/workbench/workbench_resources.cc` | Removed `eevee_engine.h` include |

### Python UI changes for EEVEE removal

| File | Change |
|------|--------|
| `properties_render.py` | Removed all EEVEE render settings panels (sampling, color management, film, SSS, volumetrics, AO, bloom, DoF, motion blur, ray tracing, clamping, fast GI). ~700 lines removed. |
| `properties_view_layer.py` | Removed EEVEE view layer/render pass panels (~130 lines) |
| `properties_world.py` | Removed EEVEE world settings (volumes, light probe, sun shadow) (~190 lines) |
| `properties_material.py` | Removed EEVEE material settings (surface, displacement, volume, thickness, backface, SSS) (~220 lines) |
| `properties_data_light.py` | Removed EEVEE light settings (shadow, contact shadow, soft shadow) (~190 lines) |
| `properties_data_lightprobe.py` | Removed EEVEE light probe panels (reflection, irradiance, volumes) (~250 lines) |
| `properties_output.py` | Removed EEVEE output format settings (~45 lines) |
| `properties_particle.py` | Removed EEVEE particle render settings (~155 lines) |
| `properties_physics_*.py` | Removed EEVEE-specific physics settings across cloth, fluid, softbody, rigidbody, etc. |
| `properties_data_camera.py` | Removed EEVEE DoF settings (~47 lines) |
| `properties_data_mesh.py` | Removed EEVEE mesh remesh settings (~33 lines) |
| `properties_texture.py` | Removed EEVEE texture settings (~75 lines) |
| `properties_freestyle.py` | Removed EEVEE freestyle settings (~36 lines) |
| `properties_scene.py` | Removed EEVEE scene settings (~30 lines) |
| `properties_object.py` | Removed EEVEE holdout/shadow catcher per-object settings |
| `node_add_menu_shader.py` | Removed EEVEE-only shader node menu entries |
| `space_node.py` | Removed EEVEE node editor settings |

### Other EEVEE-related changes

| File | Change |
|------|--------|
| `gpu/CMakeLists.txt` | Removed 2 EEVEE GPU shader entries |
| `gpu/intern/gpu_shader_create_info.cc` | Removed EEVEE shader create info registrations |
| `gpu/shaders/infos/gpu_shader_test_infos.hh` | Removed EEVEE test shader definitions |
| `render/intern/engine.cc` | Changed default render engine check |
| `render/intern/pipeline.cc` | Changed render pipeline EEVEE references |
| `nodes/shader/node_shader_util.cc` | Removed EEVEE shader utility references |
| `nodes/shader/nodes/node_shader_tex_sky.cc` | Removed EEVEE sky texture references |
| `nodes/composite/nodes/node_composite_cryptomatte.cc` | Removed EEVEE cryptomatte references |
| `nodes/composite/nodes/node_composite_image.cc` | Removed EEVEE image node references |
| `creator/creator_args.cc` | Removed EEVEE command-line engine references |
| `blenkernel/BKE_blender_version.h` | Version string updated |
| `blenkernel/intern/scene.cc` | Removed EEVEE scene defaults |
| `blenloader/intern/versioning_500.cc` | Added versioning to migrate EEVEE scenes |
| `blenloader/intern/versioning_defaults.cc` | Changed default render engine |

---

## Feature Removal (Phase 4 — Unused Blender editors)

Removed UI access to features MathOPS doesn't need. Approach: editor **libraries** that provide
utility functions used by core modules (screen, transform, object) are kept compiled but their
**space types, operators, and keymaps are NOT registered** in `spacetypes.cc`.

### Effectively removed (no UI access, no draw output)

| Feature | How |
|---------|-----|
| **Grease Pencil legacy** | Editor fully removed from build (`gpencil_legacy/` not compiled) |
| **GP draw engine** | All `engines/gpencil/` source files commented out of CMakeLists.txt |
| **GP overlays** | Overlay grease_pencil calls/members removed from overlay_instance, prepass, outline |
| **Sound editor** | Editor fully removed from build (`sound/` not compiled) |
| **Video Sequence Editor UI** | Space type not registered (library compiled for screen module deps) |
| **Movie Clip/Motion Tracking UI** | Space type not registered (library compiled for screen module deps) |
| **Mask operators** | Operators/keymaps not registered (library compiled for transform deps) |
| **Grease Pencil operators** | Operators/keymaps not registered (library compiled for object/draw deps) |

### Source-level changes

#### `editors/space_api/spacetypes.cc` — Central registration file
Commented out with `/* MATHOPS: Removed */` markers:
- **Includes**: `ED_clip.hh`, `ED_grease_pencil.hh`, `ED_mask.hh`, `ED_sequencer.hh`, `ED_sound.hh`
- **Space types**: `vse::ED_spacetype_sequencer()`, `ED_spacetype_clip()`
- **Operators**: `ED_operatortypes_grease_pencil()`, `ED_operatortypes_sound()`, `ED_operatortypes_mask()`
- **Macros**: `ED_operatormacros_clip()`, `ED_operatormacros_mask()`, `vse::ED_operatormacros_sequencer()`, `ED_operatormacros_grease_pencil()`
- **Keymaps**: `ED_keymap_grease_pencil()`, `ED_keymap_mask()`

**Kept for annotations** (gpencil_legacy ≠ Grease Pencil drawing):
- `ED_gpencil_legacy.hh` include, `ED_operatortypes_gpencil_legacy()`, `ED_keymap_gpencil_legacy(keyconf)` — all restored with `/* Annotations still needed */` comments

#### `draw/intern/draw_view_data.hh`
- Commented out `gpencil_engine.hh` include, `gpencil::Engine grease_pencil` member, `callback(grease_pencil)` in `foreach_engine`

#### `draw/intern/draw_context.cc`
- Commented out `ED_gpencil_legacy.hh` and `gpencil_engine.hh` includes
- Stubbed 4 public API functions: `gpencil_object_is_excluded` → return true, `gpencil_any_exists` → return false, `DRW_gpencil_engine_needed_viewport` → return false, `DRW_render_check_grease_pencil` → return false
- Stubbed `DRW_render_gpencil` as no-op
- Commented out `grease_pencil.set_used()` calls in `enable_engines()`
- Commented out `gpencil::Engine::free_static()` in `DRW_engines_free()`

#### `draw/engines/overlay/overlay_instance.hh`
- Commented out `overlay_grease_pencil.hh` include and `GreasePencil grease_pencil` member

#### `draw/engines/overlay/overlay_instance.cc`
- Commented out 7 `layer.grease_pencil.*` calls (begin_sync, paint/sculpt/edit/object_sync, draw_line, draw_color_only)

#### `draw/engines/overlay/overlay_prepass.hh`
- Commented out GP overlay include, `grease_pencil_ps_` member, sub-pass setup, `OB_GREASE_PENCIL` case

#### `draw/engines/overlay/overlay_outline.hh`
- Commented out GP overlay include, `prepass_gpencil_ps_` member, sub-pass setup, `OB_GREASE_PENCIL` case

### CMake-level changes

| File | Change |
|------|--------|
| `editors/CMakeLists.txt` | `gpencil_legacy` and `sound` subdirs removed. `grease_pencil`, `mask`, `space_clip`, `space_sequencer` kept with MATHOPS comments. |
| `editors/space_api/CMakeLists.txt` | `bf_editor_space_clip` and `bf_editor_space_sequencer` kept (screen module deps) |
| `editors/screen/CMakeLists.txt` | `bf_editor_space_sequencer` kept (screen module deps) |
| `makesrna/intern/CMakeLists.txt` | `bf_editor_gpencil_legacy` and `bf_editor_sound` links removed |
| `draw/CMakeLists.txt` | All `engines/gpencil/` source files commented out |
| `source/blender/CMakeLists.txt` | `draw/engines/gpencil/shaders` subdir commented out |
| `io/CMakeLists.txt` | `grease_pencil` IO subdir commented out |

### Startup Fixes (Phase 4b — Cascading registration failures)

Removing space types in C without updating all Python UI modules caused cascading failures:
`register_class()` failing for one panel halted ALL subsequent `bl_ui` module registrations,
breaking every addon that depends on `TOPBAR_MT_file_import` etc.

#### `scripts/startup/bl_ui/__init__.py`
Removed 4 modules whose space types are no longer registered:
- `properties_strip`, `properties_strip_modifier` (sequencer panels)
- `space_clip` (clip editor — root cause of cascading failure)
- `space_sequencer` (sequencer editor)

#### `scripts/startup/bl_ui/properties_render.py`
Restored `draw_curves_settings()` function accidentally deleted with EEVEE panels.
Cycles addon imports this function; its absence crashed Cycles registration.

#### `scripts/startup/bl_ui/space_toolsystem_toolbar.py`
Removed `SEQUENCER_PT_tools_active` class (uses `bl_space_type = 'SEQUENCE_EDITOR'`
which is no longer registered).

#### `scripts/startup/bl_ui/space_view3d.py`
Changed non-existent `VIEW3D_MT_sdf_add` menu reference to direct operator call:
`layout.operator("object.sdf_add", text="SDF", icon='OUTLINER_OB_SDF')`.

#### `windowmanager/intern/wm_operators.cc`
Removed modal keymap assignments for 13 removed operators in gesture circle/box/lasso
functions: `SEQUENCER_OT_*`, `CLIP_OT_*`, `MASK_OT_*`, `GREASE_PENCIL_OT_erase_*`.

#### `editors/space_view3d/space_view3d.cc`
Removed keymap handler registrations:
- 2 metaball keymap handlers (`CTX_MODE_EDIT_METABALL`)
- ~30 lines of Grease Pencil keymap handlers (Selection, Edit, Paint, Sculpt, Weight Paint,
  Vertex Paint, Brush Stroke, Fill Tool)

#### `scripts/modules/bl_keymap_utils/keymap_hierarchy.py`
Commented out `Video Sequence Editor` keymap hierarchy block — it uses
`_km_expand_from_toolsystem('SEQUENCE_EDITOR', ...)` which crashes because
the `SEQUENCE_EDITOR` `ToolSelectPanelHelper` subclass was removed in Phase 4.

#### `scripts/presets/keyconfig/keymap_data/blender_default.py`
Commented out all keymap registrations in `generate_keymaps()` for removed features:
- Mask editing, Sequencer (generic + main + preview + channels)
- Clip editor (main + editor + graph + dopesheet)
- All Grease Pencil keymaps (8 mode keymaps + 3 modal maps)
- Edit Metaball, Grease Pencil tool keymaps

### Design decisions

1. **Why keep space_sequencer/space_clip compiled?** The `screen` module calls `vse::sync_active_scene_and_time_with_scene_strip()` and `ED_clip_update_frame()`. Removing these libraries would require invasive surgery to core window management.

2. **Why keep grease_pencil editor compiled?** The `object` editor and `draw` batch cache infrastructure depend on `bf_editor_grease_pencil` utility functions.

3. **Why keep mask editor compiled?** The `transform` module links `bf_editor_mask` and uses mask functions in `transform_convert_mask.cc` and `transform_gizmo_2d.cc`.

4. **Why not just stub the functions?** Dozens of functions across multiple modules — commenting out registrations in one file (`spacetypes.cc`) is surgical and reversible.

---

## Proximity + Cycles Engine Merge (Phase 5 — Single engine entry)

Users now see only "Proximity" in the engine dropdown. Cycles is transparently delegated
for Rendered viewport and F12 final render. Solid/Material Preview stays Workbench.

### Modified Files — C++

| File | Change |
|------|--------|
| `editors/space_view3d/view3d_draw.cc` | `ED_view3d_engine_type()`: when engine is `BLENDER_PROXIMITY` and `drawtype == OB_RENDER`, return Cycles engine type. Falls back to Workbench if Cycles not loaded. |
| `render/intern/pipeline.cc` | `do_render_engine()`: swap engine to `CYCLES` before `RE_engine_render()`, swap back after. Moved `change_renderdata_engine()` outside `#ifdef WITH_FREESTYLE` block for reuse. |
| `makesrna/intern/rna_scene.cc` | `rna_RenderSettings_engine_itemf()`: skip `CYCLES` when building dropdown enum items. `rna_RenderSettings_multiple_engines_get()`: count only visible (non-Cycles) engines. |
| `blenloader/intern/versioning_500.cc` | New versioning block (500,121): remap `engine="CYCLES"` to Proximity for old .blend files. |
| `blenkernel/BKE_blender_version.h` | Bumped `BLENDER_FILE_SUBVERSION` from 120 to 121. |

### Modified Files — Python

| File | Change |
|------|--------|
| `intern/cycles/blender/addon/ui.py` | `CyclesButtonsPanel.poll()`: shows for `BLENDER_PROXIMITY` engine. `register()`: adds `BLENDER_PROXIMITY` to `COMPAT_ENGINES` on all Cycles panel classes; dynamically sets `bl_parent_id = "RENDER_PT_proximity_cycles"` on top-level Cycles render panels to nest them under the wrapper. `unregister()`: cleans up dynamic `bl_parent_id` and removes `BLENDER_PROXIMITY` from `get_panels()`. |
| `intern/cycles/blender/addon/properties.py` | Removed `bpy.types.MetaBall.cycles` registration — MetaBall was removed in Phase 3. Fixes Cycles addon crash on load. |
| `scripts/startup/bl_ui/properties_render.py` | Added 3 wrapper panels: `RENDER_PT_proximity_workbench` ("Workbench", bl_order=10), `RENDER_PT_proximity_cycles` ("Path Tracer (Cycles)", bl_order=11), `RENDER_PT_proximity_raymarcher` ("SDF Ray Marcher", bl_order=12). Workbench settings re-parented as children. Cycles panels dynamically nested at addon load. Also hid Hydra Storm from engine dropdown. |
| `build_files/windows/parse_arguments.cmd` | Changed default build directory to `%BLENDER_DIR%build` so builds output to `blender-sdf/build/`. |

### Design decisions

1. **Why hide Cycles instead of removing it?** Cycles is still the render backend — it's just hidden from the user. The addon must load to provide `view_update`/`view_draw` callbacks and `Scene.cycles` properties.

2. **Why version old CYCLES .blend files?** Files saved with `engine="CYCLES"` would show no engine dropdown entry after hiding. Versioning silently migrates them to Proximity.

3. **Why check `view_update`/`view_draw` in viewport delegation?** If Cycles addon isn't loaded, these callbacks are NULL — graceful fallback to Workbench rendering.

4. **Why dynamic `bl_parent_id` in register()?** Cycles panels are defined in the addon (not core). Using a dynamic approach in `register()` avoids hardcoding the parent ID in every panel class, and cleanly removes it on `unregister()`.

5. **Why wrapper panels?** Groups Render Properties into logical sections: Workbench (solid/matcap viewport), Path Tracer (Cycles settings for rendered/F12), and SDF Ray Marcher (placeholder for MathOPS addon). All three always visible for the Proximity engine.

---

## Cycles GPU Kernel Automation

### `prebuilt/cycles_kernels/` (New — 51 files)
Precompiled Cycles GPU kernels from official Blender 5.0.1 release. Includes CUDA `.cubin`, OptiX `.ptx`, HIP `.fatbin`, and HIPRT `.hipfb` files (zstd-compressed). These are needed because source builds default to `WITH_CYCLES_CUDA_BINARIES=OFF`, which means no GPU kernels are compiled without a CUDA Toolkit install.

### `intern/cycles/kernel/CMakeLists.txt` (Modified)
Added fallback: when `WITH_CYCLES_CUDA_BINARIES=OFF`, auto-installs prebuilt kernels from `prebuilt/cycles_kernels/` via `delayed_install`. No CUDA Toolkit required.

### `intern/cycles/CMakeLists.txt` (Modified)
Added auto-detection of OptiX SDK from standard Windows install paths (`C:/ProgramData/NVIDIA Corporation/OptiX SDK 8.*/9.*`). Eliminates manual `OPTIX_ROOT_DIR` configuration.

---

## Cycles as Default Engine (Phase 6 — Remove Workbench from UI)

Replaced the delegation architecture (Phase 5) with a simpler approach: the scene engine
IS Cycles. Solid/Material Preview viewport still uses Workbench internally via
`ED_view3d_engine_type()`, but users never see or configure Workbench. World lights,
materials, and lights all work natively because the engine is Cycles.

### Architecture Change

| Before (Phase 5) | After (Phase 6) |
|-------------------|------------------|
| `scene.r.engine = "BLENDER_PROXIMITY"` | `scene.r.engine = "CYCLES"` |
| `ED_view3d_engine_type()` delegates Rendered→Cycles | Cycles is the engine; no delegation needed |
| `do_render_engine()` swaps engine for F12 | Cycles handles F12 natively |
| `context.engine == 'BLENDER_PROXIMITY'` | `context.engine == 'CYCLES'` |
| Engine dropdown hidden (only Proximity visible) | Engine dropdown hidden (only Cycles visible) |
| Workbench panels in Render Properties | No Workbench panels — Cycles panels always shown |
| BLENDER_PROXIMITY added to Cycles COMPAT_ENGINES | Not needed — engine IS Cycles |

### Modified Files — C++

| File | Change |
|------|--------|
| `blenloader/intern/versioning_defaults.cc` | Default engine for new files changed from `RE_engine_id_BLENDER_WORKBENCH` to `RE_engine_id_CYCLES`. |
| `blenloader/intern/versioning_500.cc` | Merged versioning blocks (500,120 + 500,121) into single (500,122): remaps EEVEE, old Workbench, EEVEE Next, and BLENDER_PROXIMITY all to CYCLES. |
| `blenkernel/BKE_blender_version.h` | Bumped `BLENDER_FILE_SUBVERSION` from 121 to 122. |
| `makesrna/intern/rna_scene.cc` | Engine dropdown now hides `BLENDER_PROXIMITY` + `HYDRA_STORM` (was hiding `CYCLES` + `HYDRA_STORM`). |
| `editors/space_view3d/view3d_draw.cc` | Removed Proximity→Cycles viewport delegation in `ED_view3d_engine_type()`. |
| `render/intern/pipeline.cc` | Removed F12 engine swap in `do_render_engine()`. |
| `draw/engines/overlay/overlay_instance.cc` | Updated comment on `viewport_uses_workbench`. Logic unchanged (correct for engine=CYCLES). |

### Modified Files — Python

| File | Change |
|------|--------|
| `scripts/startup/bl_ui/properties_render.py` | Removed Workbench wrapper + 5 children panels. Removed viewport helpers. Path Tracer always visible for CYCLES engine. Color Management moved to bl_order=200 with `layout.separator(type='LINE')` contour. |
| `scripts/startup/bl_ui/__init__.py` | Removed msgbus shading subscription (no longer needed — no viewport-dependent panels). |
| `scripts/startup/bl_ui/space_view3d.py` | Removed `BLENDER_PROXIMITY` engine checks in lighting panel poll and studio light rotation. |
| `intern/cycles/blender/addon/ui.py` | Removed all BLENDER_PROXIMITY from COMPAT_ENGINES, register(), unregister(), draw_pause(). Device selector still shown in Path Tracer wrapper. Kept `bl_parent_id` nesting for Cycles panels under `RENDER_PT_proximity_cycles`. |

### Design decisions

1. **Why Cycles as scene engine?** World lights, materials, and light objects are Cycles-specific properties. With engine=BLENDER_PROXIMITY (Workbench), these properties weren't accessible through `context.engine` checks, requiring COMPAT_ENGINES hacks. Making Cycles the scene engine gives natural access.

2. **Why keep Workbench?** `ED_view3d_engine_type()` hardcodes Workbench return for `shading.type <= OB_SOLID`. Solid/Wireframe modes always use Workbench regardless of scene engine. This is a viewport-level concern, not a scene engine concern.

3. **Why remove viewport-dependent panel polling?** With Cycles as the sole engine, all Cycles panels should always be visible. Users don't need panels appearing/disappearing based on viewport mode.

4. **Why hide BLENDER_PROXIMITY from dropdown?** It's still registered as an engine type (needed for Solid viewport), but users should never manually select it.

---

## Native SDF Draw Engine (Phase 7 — C++ voxel renderer)

Added a native `DrawEngine` that bakes SDF objects into a dense 3D atlas (256^3, R16F) via compute shader, then ray-marches it in a fullscreen fragment shader. Runs alongside Workbench — meshes rendered by Workbench, SDF objects rendered by SDF engine, sharing the depth buffer for correct occlusion.

**Scope:** Cubes only. No blend/CSG, no BVH, no sparse atlas. Flat shading with object color.

### New Files (5)

| File | Purpose |
|------|---------|
| `draw/engines/sdf/sdf_engine.h` | Engine pointer struct (`Engine : DrawEngine::Pointer`) |
| `draw/engines/sdf/sdf_engine.cc` | `Instance` class: init, sync, bake dispatch, ray-march draw |
| `draw/engines/sdf/sdf_private.hh` | Internal types, atlas resolution constant |
| `draw/engines/sdf/sdf_shader_shared.hh` | Shared C++/GLSL struct (`SDFObjectGPU`, 160 bytes) |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | `ShaderCreateInfo` for `sdf_bake` (compute) and `sdf_march` (fragment) |

### New Shader Files (3)

| File | Purpose |
|------|---------|
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Compute shader (8×8×4 workgroup): evaluates `sdBox` for all objects at each voxel, writes min distance |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Fullscreen fragment shader: sphere-traces the atlas, computes normals via central differences, writes color+depth |
| `draw/engines/sdf/shaders/sdf_lib.glsl` | SDF primitives library (`sdBox`) |

### Modified Files (3)

| File | Change |
|------|--------|
| `draw/intern/draw_view_data.hh` | Added `#include "engines/sdf/sdf_engine.h"`, `sdf::Engine sdf` member, `callback(sdf)` in `foreach_engine()` after workbench |
| `draw/intern/draw_context.cc` | Added `#include "engines/sdf/sdf_engine.h"`, `view_data.sdf.set_used(true)` wherever workbench is enabled. Also enabled SDF engine in `DEPTH`/`DEPTH_ACTIVE_OBJECT` modes for 3D cursor placement, orbit pivot, and view center pick on SDF surfaces |
| `draw/engines/sdf/sdf_engine.cc` | `draw_march()` uses `depth_only_fb` in depth mode (no color attachment), `default_fb` in normal mode. `GPU_flush()` after march pass ensures depth writes are visible to overlay grid texture reads during rapid camera movement |
| `draw/CMakeLists.txt` | Added SDF engine source, headers, shader info, GLSL files, and shared header to build |

### Architecture

- **Object sync**: For each `OB_SDF` object, decomposes `object_to_world()` into rotation + scale, bakes scale into `sdf_size`, packs into `SDFObjectGPU` SSBO
- **Dirty tracking**: Hash-based — if no objects changed, skip compute bake, just ray-march the cached atlas
- **Bake pass**: Compute dispatch writes to 3D `image3D` (R16F), each thread evaluates one voxel against all objects with AABB early-out
- **March pass**: Fullscreen triangle with `gpu_fullscreen` vertex shader. Reconstructs rays from `draw_view` UBO matrices, sphere-traces the atlas with hardware trilinear filtering, writes `gl_FragDepth` for mesh occlusion

### Bug Fixes

- **Shader info file naming**: Renamed `sdf_shader_info.hh` → `sdf_shader_infos.hh` (plural). Blender's GLSL preprocessor strips `#include` directives containing `"infos.hh"` as a substring — the singular name didn't match, causing runtime "Dependency not found" errors.
- **IMAGE macro syntax**: Changed `IMAGE(0, GPU_R16F, WRITE, image3D, sdf_atlas)` → `IMAGE(0, SFLOAT_16, write, image3D, sdf_atlas)`. The macro maps to `TextureFormat::format` and `Qualifier::qualifiers` (lowercase enum values, not GPU API constants).
- **STORAGE_BUF qualifier case**: Changed `READ` → `read` to match `Qualifier::read` enum.

### SDF Add Menu (Add > SDF > Cube)

| File | Change |
|------|--------|
| `editors/object/object_sdf.cc` | Added `type` RNA enum property to `OBJECT_OT_sdf_add` operator; sets `SDF.sdf_type` on created object. Added `sdf_type_name()` to give type-specific default names ("SDF Cube", "SDF Sphere", etc.) instead of generic "SDF". |
| `scripts/startup/bl_ui/space_view3d.py` | Added `VIEW3D_MT_sdf_add` submenu class with Cube entry. SDF menu moved to top of Add menu (before Mesh) with a separator line divider below it. |

### Analytic Voxel Intersection (Phase 8 — Replace sphere tracing with DDA + cubic solver)

Replaced the iterative sphere tracing ray marcher with DDA grid traversal + analytic cubic
intersection, based on "Ray Tracing of Signed Distance Function Grids"
(Hansson-Soderlund, Evans, Akenine-Moller 2022). The baked 3D atlas IS a voxel grid,
so this method applies directly and provides exact surface intersections.

**Benefits:**
- Exact surface intersections (no iterative convergence issues near silhouettes)
- Analytical normals from trilinear gradient (~12 FMAs vs 6 texture reads for central differences)
- Eliminates sphere tracing's slow convergence near surface tangents

### Modified Shader Files

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_lib.glsl` | Major expansion: added `computeTrilinearCoeffs()` (Eq. 3), `computeCubicCoeffs()` (Eq. 6-7), `evalCubic()`/`evalCubicDeriv()` helpers, `solveCubicFirstRoot()` (Vieta's trigonometric method), `solveCubicMarmittNR()` (Marmitt + Newton-Raphson fallback), `trilinearGradient()` (analytical normal from corner values). Original `sdBox` and `opSmoothUnion` kept unchanged. |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Complete rewrite: replaced sphere tracing loop with Amanatides & Woo DDA traversal (max 512 steps). Each voxel cell: fetch 8 corners via `texelFetch`, quick reject (all-positive skip / all-negative immediate hit), compute trilinear + cubic coefficients, solve cubic for exact surface intersection. Normals via analytical `trilinearGradient()` instead of central differences. `find_blended_color()` unchanged. |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `PUSH_CONSTANT(int3, atlas_resolution)` to `sdf_march` shader info for DDA bounds checking. |

### Modified Engine Files

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Added `GPU_shader_uniform_3iv(march_sh_, "atlas_resolution", res)` in `draw_march()` to pass atlas resolution to the march shader. |

### Technical Details

- **DDA traversal**: Amanatides & Woo algorithm traverses the 3D grid cell-by-cell along the ray. For a 256^3 grid, worst case is ~443 cells (diagonal), capped at 512 steps.
- **Cubic solver (primary)**: Vieta's trigonometric method — normalizes, depresses to u^3 + pu + q = 0, uses trigonometric solution for 3-root case (D <= 0) or Cardano for 1-root case (D > 0). Handles degenerate cases (quadratic/linear fallback when c3 ~ 0).
- **Cubic solver (fallback)**: Marmitt method — finds g'(t) = 0 roots to decompose [0, tfar] into monotone intervals, then Newton-Raphson in subintervals with sign changes.
- **Grid space**: Sample (i,j,k) maps to grid coordinate (i,j,k). Cell (i,j,k) spans [i,i+1]^3. Valid cells: [0, res-2]^3. Grid bounds are half-voxel inset from atlas bounds.
- **Empty cell rejection**: min/max of 8 corners — all positive skips, all negative is immediate hit.
- **Entry-inside detection**: If trilinear at ray entry point <= 0, report immediate hit (handles camera-inside-surface case).
- **Cubic reparametrization**: Raw grid direction D (~60 for 256^3 grid) causes catastrophic float32 conditioning. Fix: scale `d_scaled = D * T_max` so parameter u ∈ [0,1] with balanced O(0.1) coefficients. Uses Marmitt+NR solver (more robust than Vieta's on GPU — no transcendental functions).
- **Hybrid sphere-trace / DDA**: Pure DDA visits every cell (200-400 cells × 8 texelFetch = 1600-3200 texture reads). Fix: 1 `textureLod` probe per step; if `probe_dist > 2*voxel_size`, sphere-tracing jump (skip empty space in O(1)). Full 8-corner fetch + cubic solve only when within ~2 voxels of surface. Gives sphere tracing's logarithmic convergence in empty space + paper's exact intersection at the surface.

### Matcap / Studio / Flat Shading (Phase 8b — Viewport shading modes)

Made SDF objects respect the viewport's `View3DShading.light` setting (FLAT/STUDIO/MATCAP).
Previously the SDF engine used a hardcoded directional light (`vec3(0.5, 0.7, 1.0)` + 0.15 ambient).
Now SDF objects shade identically to mesh objects in Workbench.

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `SAMPLER(2, sampler2DArray, matcap_tx)`, `PUSH_CONSTANT(int, lighting_type)`, `PUSH_CONSTANT(int, use_specular)` to `sdf_march` shader info. |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Replaced hardcoded directional light with FLAT/STUDIO/MATCAP shading. FLAT outputs object color directly. STUDIO/MATCAP compute matcap UV from view-space normal (same formula as `workbench_matcap_lib.glsl:matcap_uv_compute()`), sample diffuse (layer 0) + specular (layer 1) from `matcap_tx`. |
| `draw/engines/sdf/sdf_engine.cc` | Added `sync_shading()` method: reads `v3d->shading.light/matcap/studio_light/flag`, calls `BKE_studiolight_find/ensure_flag`, builds 2-layer matcap array texture (SFLOAT_16_16_16_16) from `StudioLight::matcap_diffuse/specular` ImBuf data. Cached by matcap name. Push constants `lighting_type` and `use_specular` bound in `draw_march()`. 1×1 white fallback texture if no matcap loaded. |

### Dual Voxel Normals + Matcap Mapping Fix (Phase 8c — Normal quality + workbench parity)

**Normals**: Replaced single-voxel analytic gradient (C0 discontinuous at voxel boundaries) with the
**dual voxel normal** method from Hansson-Soderlund et al. 2022 (Section 3.2, Eq. 12). A dual voxel
shifted by half a voxel overlaps 2x2x2 primal voxels. For each of the 8 overlapping primal voxels,
the analytic gradient is computed at the hit point and normalized. The 8 normalized normals are then
trilinearly interpolated using the hit point's position within the dual voxel. This gives
C0-continuous normals across voxel boundaries. Cost: 27 texelFetch + 8 gradient evaluations per pixel
(only at the hit point, not per DDA step).

**Matcap mapping**: Fixed incident vector to use `drw_view_incident_vector()` (from `draw_view_lib.glsl`)
instead of `normalize(view_pos)`. This correctly handles orthographic cameras (`I = (0,0,1)` constant)
vs perspective (`I = normalize(-vP)`), and fixes the sign convention (I must point surface-to-camera
for the matcap basis formula to work). Added `V3D_SHADING_MATCAP_FLIP_X` support via `use_matcap_flip`
push constant, matching Workbench's `matcap_uv_compute(I, N, flipped)`.

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Added `computeDualVoxelNormal()` function (27 texel 3x3x3 fetch, 8 gradient evals, trilinear blend). Replaced single-voxel `trilinearGradient()` call with `computeDualVoxelNormal()`. Fixed matcap `I` vector to use `drw_view_incident_vector()`. Added `use_matcap_flip` support. |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `PUSH_CONSTANT(int, use_matcap_flip)` to `sdf_march` shader info. |
| `draw/engines/sdf/sdf_engine.cc` | Added `use_matcap_flip_` member. Set from `V3D_SHADING_MATCAP_FLIP_X` flag in `sync_shading()`. Pushed as uniform in `draw_march()`. |

### Baked Color Atlas (Phase 8d — O(1) rendering independent of object count)

Changed atlas from R16F to RGBA16F. The bake shader now computes blended color alongside distance
and writes both to the atlas. Layout: `.r` = signed distance, `.gba` = RGB color. The march shader
reads color via a single `textureLod` at the hit point instead of re-evaluating all N objects.

**Before**: `find_blended_color()` looped over all objects per pixel = O(N) per pixel.
**After**: Color comes from texture read = O(1) per pixel. 10 objects or 10,000 — same cost.

VRAM increase: R16F 256³ = ~32 MB → RGBA16F 256³ = ~128 MB. Acceptable for modern GPUs.

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Changed bake atlas IMAGE format from `SFLOAT_16` to `SFLOAT_16_16_16_16`. Removed `STORAGE_BUF(objects[])`, `object_count`, and `TYPEDEF_SOURCE` from march shader info (no longer needed). |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Accumulates blended color alongside distance using same smooth union weighting. `imageStore(sdf_atlas, voxel, float4(acc_dist, acc_color))`. |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Removed `find_blended_color()`. Color read from atlas: `textureLod(sdf_atlas, hit_uv, 0.0).gba` — one texture read, hardware trilinear interpolation, O(1). |
| `draw/engines/sdf/sdf_engine.cc` | Atlas format `SFLOAT_16` → `SFLOAT_16_16_16_16`. Removed SSBO binding and `object_count` push constant from `draw_march()`. |

### Sparse Brick-Based Voxel Grid (Phase 9 — 85-95% VRAM savings)

Replaced the dense 256³ atlas (~128 MB VRAM) with a sparse brick-based system. The volume
is subdivided into 8³ bricks. A classify pass identifies which bricks contain surface (typically
5-15%), and only those bricks are baked into a compact atlas. The march shader uses two-level
DDA (brick-level → voxel-level) to skip empty space efficiently.

**Benefits:**
- 85-95% VRAM reduction for typical scenes (only surface-containing bricks stored)
- Faster bake (fewer voxels to evaluate)
- Configurable resolution (64/128/256/512) via View3DShading properties
- Debug grid overlay shows brick boundaries and occupancy heatmap

### New Files (1)

| File | Purpose |
|------|---------|
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Classify compute shader: one thread per brick, evaluates SDF at brick center, writes slot index or void marker (-1/-2) to indirection texture, atomically counts active bricks |

### Modified Files (DNA)

| File | Change |
|------|--------|
| `makesdna/DNA_view3d_types.h` | Added `sdf_resolution` (int), `sdf_debug_grid` (short), `_sdf_pad` (short) to `View3DShading` struct |

### Modified Files (RNA)

| File | Change |
|------|--------|
| `makesrna/intern/rna_space.cc` | Added RNA enum properties `sdf_resolution` (64/128/256/512) and `sdf_debug_grid` (Off/Grid Lines/Occupancy) in `rna_def_space_view3d_shading()` |

### Modified Files (Editor/UI)

| File | Change |
|------|--------|
| `scripts/startup/bl_ui/properties_render.py` | Updated `RENDER_PT_proximity_raymarcher` panel to show `sdf_resolution` and `sdf_debug_grid` controls from `View3DShading` |

### Modified Files (Draw)

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_private.hh` | Replaced `SDF_ATLAS_RES = 256` with brick constants: `SDF_BRICK_SIZE = 8`, `SDF_BRICK_STORAGE = 10`, `SDF_MAX_BRICKS = 8192` |
| `draw/engines/sdf/sdf_shader_shared.hh` | Added `BrickCounter` SSBO struct, `SDFClassifyParams` push constants. Updated `SDFBakeParams` and `SDFMarchParams` with `bricks_per_axis`, `grid_resolution`, `debug_grid` |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `sdf_classify` shader info. Updated `sdf_bake` to use `indirection_tx` sampler + `compact_atlas` image + `bricks_per_axis` push constant (workgroup 10×10×1). Updated `sdf_march` to use `compact_atlas` + `indirection_tx` samplers + `bricks_per_axis`/`debug_grid` push constants |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Rewritten: each workgroup = one brick, reads indirection to skip void bricks, writes 10³ voxels (8 inner + 1 overlap each side) to compact atlas at slot-indexed origin |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Rewritten: two-level DDA (brick-level traversal + voxel-level DDA within active bricks). `fetchCornersCompact()` reads from slot-indexed compact atlas. `computeDualVoxelNormalCompact()` uses brick-local 3×3×3 neighborhood. Debug grid overlay (mode 1: cyan brick boundary lines, mode 2: green→red occupancy heatmap) |
| `draw/engines/sdf/sdf_engine.cc` | Major rewrite: added `indirection_tx_` (R32I), `compact_atlas_tx_` (RGBA16F), `brick_counter_` (SSBO), `classify_sh_`. New methods: `sync_sdf_settings()`, `ensure_indirection()`, `dispatch_classify()`, `ensure_compact_atlas()`. Pipeline: classify → ensure_compact_atlas → bake → march. GPU readback of active brick count. Console logging of brick statistics |
| `draw/CMakeLists.txt` | Added `sdf_classify_comp.glsl` to shader file list |

### Pipeline (3-pass)

1. **Classify** (new): One thread per brick (4×4×4 workgroup). Evaluates SDF at brick center; if `|distance| < brick_half_diagonal`, brick is active. Active bricks get atomic counter slot, written to R32I indirection texture. Void bricks marked -1 (outside) or -2 (inside).
2. **Bake** (modified): One workgroup per brick (10×10×1 threads, loop over Z). Skips void bricks via indirection lookup. Writes 10³ voxels (including overlap border) to compact atlas at `slot_origin + local_voxel`.
3. **March** (modified): Brick-level DDA through grid_res³ grid, then voxel-level DDA within active bricks. Two-level approach eliminates sphere-trace skip (brick DDA is already O(grid_res) vs O(total_res)).

---

## Mesh to SDF Grid — Convert Meshes to SDF via Geometry Nodes

Adds a "Mesh to SDF Grid" operator that converts mesh objects into SDF volume grids
via Blender's Geometry Nodes, then blends them into the SDF draw engine's atlas alongside
analytic SDF objects. Supports per-object blend control and 100+ mesh grid objects.

### New Files

| File | Purpose |
|------|---------|
| `scripts/startup/bl_operators/mesh_to_sdf.py` | Python operator `OBJECT_OT_mesh_to_sdf_grid`. Creates/reuses a "Mesh to SDF Grid" GN node group with `GeometryNodeMeshToSDFGrid` + `GeometryNodeStoreNamedGrid`. Interface sockets: Geometry (in/out), Voxel Size (0.3), Band Width (3), Blend (0.0). Blend socket is unconnected — it's a parameter read by the SDF engine from modifier IDProperties |
| `draw/engines/sdf/shaders/sdf_grid_blend_comp.glsl` | Compute shader that blends one grid SDF into the compact brick atlas. Same layout as bake shader (10×10×1 workgroup per brick). Reads indirection, skips void bricks. For each voxel: transforms world position to grid UVW via `grid_world_to_texture` matrix, samples `sdf_grid` texture, blends into `compact_atlas` via smooth union (imageLoad/imageStore) |

### Modified Files (Python)

| File | Change |
|------|--------|
| `scripts/startup/bl_operators/__init__.py` | Added `"mesh_to_sdf"` to `_modules` list for operator registration |
| `scripts/startup/bl_ui/space_view3d.py` | Added separator + "Mesh to SDF Grid" menu entry in `VIEW3D_MT_sdf_add` |

### Modified Files (Draw)

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `sdf_grid_blend` shader create info: `indirection_tx` + `sdf_grid` samplers, `compact_atlas` read-write image, push constants for `grid_world_to_texture` (mat4), `grid_color`, `grid_blend` |
| `draw/engines/sdf/sdf_engine.cc` | Added mesh volume detection in `object_sync()` via `ObjectRuntime::geometry_set_eval`. New structs: `PendingGridObject`, `GridObject`. New methods: `process_grid_object()` (extracts dense floats via `BKE_volume_grid_dense_floats`, creates R16F GPU 3D texture, computes `world_to_texture` transform, reads Blend from modifier IDProperties), `dispatch_grid_blends()` (one compute dispatch per grid object with memory barriers), `fill_indirection_for_grids()` (grid-only scenes), `clear_compact_atlas()`. Grid objects included in dirty-tracking hash |
| `draw/CMakeLists.txt` | Added `sdf_grid_blend_comp.glsl` to shader file list |

### Pipeline (updated)

1. **Classify** — unchanged (analytic SDF objects only)
2. **Bake** — unchanged (analytic SDF objects only)
3. **Grid Blend** (new) — one compute dispatch per grid object. Each dispatch reads the current atlas value (imageLoad), samples the grid's 3D texture at the transformed world position, blends distances via smooth union, writes back (imageStore). Memory barrier between dispatches. For grid-only scenes (no analytic objects), all bricks are marked active and atlas is initialized to large distance (1e10)
4. **March** — unchanged (reads same compact atlas)

---

## Session 7: Fix Surface Artifacts + 3D Wireframe Debug Grid

### Bug Fix: Voxel DDA Off-by-One

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Fixed off-by-one in voxel-level DDA: changed `BRICK_SIZE - 2` to `BRICK_SIZE - 1` in initial vcell clamp (line 245) and loop boundary check (line 270). This allows voxel cell 7 (the last interior cell) to be traversed. Previously, surfaces at the last voxel of each brick were missed, causing black streak artifacts on flat SDF surfaces |

### Feature: 3D Voxel Grid Debug Overlay

Replaced the shader-overlay debug grid (which painted on the SDF surface) with a single "3D Voxel Grid" mode: real 3D wireframe cubes around active bricks, drawn as `GPU_PRIM_LINES` after the march pass. Two-pass rendering: bright green lines in front of the SDF, faint ghosted lines behind.

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Removed debug grid overlay section and `debug_grid` variable usage |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Removed `debug_grid` push constant from `sdf_march` shader info |
| `draw/engines/sdf/sdf_engine.cc` | Added `draw_debug_grid()` with two-pass front/back rendering via `GPU_SHADER_3D_UNIFORM_COLOR`. Removed unused `rebuild_grid_batch_full()` (uniform grid) — only active-brick wireframes remain. Removed `debug_grid` uniform upload to march shader. New members: `grid_batch_`, `grid_batch_mode_`, `grid_batch_res_`. Helpers: `create_line_batch()`, `rebuild_grid_batch()`, `rebuild_grid_batch_active()`. Batch invalidated on rebake. Cleanup in destructor. Added includes for `MEM_guardedalloc.h` and `GPU_shader_builtin.hh` |
| `makesrna/intern/rna_space.cc` | Consolidated enum to 2 items: Off, "3D Voxel Grid" (value 1). Removed old "Grid Lines" (1) and "Occupancy Heatmap" (2) |
| `makesdna/DNA_view3d_types.h` | Updated `sdf_debug_grid` comment |
| `scripts/startup/bl_ui/properties_render.py` | Renamed UI label to "3D Voxel Grid" |

---

## Session 8: Fix Normal Transitions + Surface Margin Control

### Bug Fix: Dual Voxel Normal Discontinuities (Increased Brick Overlap)

Increased brick overlap padding from 1 to 2 on each side (`BRICK_STORAGE` 10 → 12). This gives the dual voxel normal method a full 3x3x3 neighborhood for **all** base values `[-1, 7]` without any fallback. Previously, base=7 (cell 7 hits) would read beyond the 10-wide storage causing hard normal seams at brick boundaries.

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_private.hh` | `SDF_BRICK_STORAGE` 10 → 12 (8 inner + 2 overlap each side) |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | `BRICK_STORAGE` 10→12. `gridToCompact` offset +1→+2. `computeDualVoxelNormalCompact` clamp range `[0,6]`→`[-1,7]`, atlas offset +1→+2. Color sampling offset +1→+2 |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | `BRICK_STORAGE` 10→12, world-pos overlap offset -1→-2 |
| `draw/engines/sdf/shaders/sdf_grid_blend_comp.glsl` | `BRICK_STORAGE` 10→12, world-pos overlap offset -1→-2 |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Bake and grid_blend workgroup sizes 10×10×1 → 12×12×1 |

### Feature: Surface Margin UI Parameter

Added a controllable "Surface Margin" slider (50%-300%, default 100%) that multiplies the brick classification threshold (`brick_half_diag`). Increasing this fills holes where voxels near the surface boundary were not classified as active.

| File | Change |
|------|--------|
| `makesdna/DNA_view3d_types.h` | Added `short sdf_surface_margin` field (percentage, 0 or 100 = default 1.0x) replacing `_pad3` |
| `makesrna/intern/rna_space.cc` | Added `sdf_surface_margin` RNA property (INT, PERCENTAGE, range 50-300, default 100) |
| `scripts/startup/bl_ui/properties_render.py` | Added "Surface Margin" slider to SDF Ray Marcher panel |
| `draw/engines/sdf/sdf_engine.cc` | Added `surface_margin_` member, read from `View3DShading::sdf_surface_margin` in `sync_sdf_settings()`, applied as multiplier to `brick_half_diag` in `dispatch_classify()`. Included in scene hash to trigger rebake on change |

### Bug Fix: Smooth Union Blend Cutoff at High k Values

With large blend factors (e.g. k=5.0), the blended surface extends beyond the atlas volume. The per-object AABB only accounted for `sdf_size + bevel`, not the blend radius. The blended region was clipped at the atlas boundary, visible as hard cutoff planes.

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Expanded per-object world AABB by blend value: `local_extent = sdf_size + bevel + blend`. This grows the atlas volume to cover the full smooth union region. The classify threshold (`brick_half_diag`) no longer needs the expensive `+ max_blend` — reverted to pure geometric half-diagonal × surface_margin |

---

## Cycles SDF Surface Primitive Integration

SDF objects are integrated into Cycles as a **surface primitive** (`PRIMITIVE_SDF`), giving them full path-traced material evaluation — GI, reflections, metallic, glass, SSS — exactly like triangle meshes. The existing draw engine's sparse brick atlas + two-level DDA ray march pipeline is ported to Cycles' kernel.

### New Files (5)

| File | Purpose |
|------|---------|
| `intern/cycles/scene/sdf.h` | `SDFGeometry` class extending `Geometry`. Stores grid parameters, CPU-baked atlas data (indirection, atlas as half4, matid as int16), per-object shader mapping. Overrides `compute_bounds()`, `primitive_type() -> PRIMITIVE_SDF` |
| `intern/cycles/scene/sdf.cpp` | `SDFGeometry` implementation: NODE_DEFINE registration, constructor with defaults (grid_res=32, voxel_size=1/256), `compute_bounds()` from origin + grid extent, `apply_transform()` no-op (world-space baked) |
| `intern/cycles/blender/sdf.cpp` | Blender-to-Cycles sync + CPU bake. Collects all SDF objects via depsgraph, reads transform/RNA (size, bevel, blend, color, sdf_type). Two-phase bake: classify bricks (active/inside/outside), then evaluate all objects with smooth union at each voxel. Stores distance+color in atlas, tracks closest object in matid for per-object materials |
| `intern/cycles/kernel/geom/sdf.h` | Kernel ray march: two-level DDA (brick-level skips empty space, voxel-level finds exact surface via cubic solver). Flat array access via `kernel_data_fetch`. Normal from dual voxel method (27-neighborhood trilinear gradient blend). `sdf_shader_setup()` for ShaderData initialization |
| `intern/cycles/kernel/geom/sdf_lib.h` | SDF math primitives ported from `sdf_lib.glsl`: trilinear coefficient computation, cubic polynomial construction, Marmitt+Newton-Raphson cubic solver, analytic trilinear gradient |

### Modified Files (Cycles)

| File | Change |
|------|--------|
| `intern/cycles/kernel/types.h` | Added `PRIMITIVE_SDF = (1 << 7)` to PrimitiveType enum. Updated `PRIMITIVE_ALL` and `PRIMITIVE_NUM_SHAPES` (6→7). Added `KernelSDF` struct (offsets into flat arrays, grid params, origin, voxel_size). Added `sdf` section to `KernelData` with `num_sdfs` |
| `intern/cycles/scene/geometry.h` | Added `SDF` to `Geometry::Type` enum, `is_sdf()` helper, `SDF_ADDED`/`SDF_REMOVED` flags to GeometryManager, updated `GEOMETRY_ADDED`/`GEOMETRY_REMOVED` composites |
| `intern/cycles/scene/devicescene.h` | Added `device_vector<KernelSDF> sdf_objects`, `device_vector<int> sdf_shader_map`, `device_vector<int> sdf_indirection`, `device_vector<float4> sdf_atlas`, `device_vector<int> sdf_matid` |
| `intern/cycles/scene/devicescene.cpp` | Constructor initializers for all 5 SDF device vectors |
| `intern/cycles/scene/scene.h` | Forward declaration `SDFGeometry`, `create_node`/`delete_node` template specializations |
| `intern/cycles/scene/scene.cpp` | `create_node<SDFGeometry>()` and `delete_node(SDFGeometry*)` implementations, SDF branch in generic `delete_node(Geometry*)` |
| `intern/cycles/scene/geometry.cpp` | SDF device upload: counts SDF geometries, allocates and fills flat arrays (indirection, atlas half4→float4, matid int16→int), copies per-object shader map, uploads all to device. Sets `dscene->data.sdf.num_sdfs` |
| `intern/cycles/kernel/data_arrays.h` | Added `KERNEL_DATA_ARRAY` entries for `sdf_objects`, `sdf_shader_map`, `sdf_indirection`, `sdf_atlas`, `sdf_matid` |
| `intern/cycles/kernel/bvh/bvh.h` | `scene_intersect()` rewritten to accumulate hits, added SDF check after BVH traversal via `sdf_intersect_all()` |
| `intern/cycles/kernel/geom/shader_data.h` | Added `PRIMITIVE_SDF` branch in `shader_setup_from_ray()` — calls `sdf_shader_setup()`, handles backfacing, ray differentials |
| `intern/cycles/blender/sync.h` | Added `SDFGeometry` forward declaration and `sync_sdf()` declaration |
| `intern/cycles/blender/object.cpp` | Added `type_SDF` to `object_is_geometry()` and `object_can_have_geometry()` |
| `intern/cycles/blender/geometry.cpp` | Added SDF detection in `determine_geom_type()`, SDF default shader (surface), `SDFGeometry` creation, `sync_sdf` dispatch, SDF no-op in motion sync |

### CMakeLists Changes (3)

| File | Change |
|------|--------|
| `intern/cycles/blender/CMakeLists.txt` | Added `sdf.cpp` |
| `intern/cycles/scene/CMakeLists.txt` | Added `sdf.h`, `sdf.cpp` |
| `intern/cycles/kernel/CMakeLists.txt` | Added `geom/sdf.h`, `geom/sdf_lib.h` |

### Cycles SDF Bug Fixes (Post-Integration)

Multiple fixes to SDF rendering quality, interaction, and real-time updates in Cycles rendered view.

#### Surface Normals ("Jeans Material" Fix)

Increased Cycles kernel's `SDF_BRICK_STORAGE` from 10 to 12 (matching draw engine), added `SDF_BRICK_BORDER=2`. This gives the dual voxel normal method a full 3×3×3 neighborhood for all base values `[-1, BRICK_SIZE-1]`. Also fixed baking to use `-BRICK_BORDER` offset, and fixed fully-inside bricks using slot=-2 instead of 0.

| File | Change |
|------|--------|
| `intern/cycles/kernel/geom/sdf.h` | `SDF_BRICK_STORAGE` 10→12, added `SDF_BRICK_BORDER=2`. Fixed `sdf_grid_to_compact` offset +1→+2. Fixed `sdf_compute_normal` base clamp `[0,6]`→`[-1,7]`, atlas offset +1→+2. Fixed fully-inside brick slot from 0 to -2. Encoded brick cell + slot in `isect->u/v` via `__int_as_float` for reliable decode in shader_setup. Initialized `sd->shader` to first shader in map before matid lookup |
| `intern/cycles/blender/sdf.cpp` | Grid resolution 32→64. `BRICK_STORAGE` 10→12, `BRICK_BORDER=2`. Bake offset -1→-2 |

#### SDF Object Bounding Box

Added `case OB_SDF` to `BKE_object_boundbox_get()` so viewport selection, transform gizmos, and interaction work correctly for SDF objects.

| File | Change |
|------|--------|
| `source/blender/blenkernel/intern/object.cc` | Added `OB_SDF` case returning bounds from `SDF::size + SDF::bevel` |

#### Real-Time Transform Updates

SDF atlas is world-space baked — moving any SDF object requires full atlas re-bake. Modified Cycles sync to trigger geometry re-sync when an SDF object's transform changes.

| File | Change |
|------|--------|
| `intern/cycles/blender/sync.cpp` | Added `(updated_transform && is_sdf_object)` to geometry re-sync condition at line 154 |

#### Deduplicated SDF Device Upload

All SDF objects bake into one shared atlas, so only the first valid SDFGeometry is uploaded to the device (num_sdfs=1). Also fixed `sd->object` — now finds the actual Cycles object index for the first SDF object instead of hardcoding 0.

| File | Change |
|------|--------|
| `intern/cycles/scene/geometry.cpp` | Deduplicates SDFGeometry upload. Searches `scene->objects` for first SDF geometry to set correct `object_id` |

#### Mesh-to-SDF and Narrow Band Optimization

Reduced default narrow band width from 3 to 2 voxels (~33% faster `meshToLevelSet` and less data). Replaced per-element vertex copy with `memcpy` in `mesh_to_sdf_grid`. Removed the aggressive band cutoff in the grid blend shader — inactive voxels at ±background provide valid conservative distance estimates, preventing the ray marcher from overshooting the surface due to 1e10 discontinuities. Removed `grid_background` push constant and `background_value` from GridObject since the cutoff is gone.

| File | Change |
|------|--------|
| `source/blender/geometry/intern/mesh_to_volume.cc` | Vertex copy via `memcpy` instead of parallel_for loop |
| `source/blender/nodes/geometry/nodes/node_geo_mesh_to_sdf_grid.cc` | Default band width 3→2, updated description |
| `scripts/startup/bl_operators/mesh_to_sdf.py` | Default band width 3→2, updated comment |
| `source/blender/draw/engines/sdf/shaders/sdf_grid_blend_comp.glsl` | Removed band cutoff (was causing surface truncation) |
| `source/blender/draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Removed `grid_background` push constant |
| `source/blender/draw/engines/sdf/sdf_engine.cc` | Removed `background_value` from GridObject, removed O(n) voxel scan, removed `grid_background` uniform upload |

---

### Incremental SDF Baking + Per-Brick Object Culling

Two complementary optimizations to the SDF bake pipeline:

**Per-brick object culling:** Each brick now skips SDF objects whose world-space AABB doesn't overlap the brick, in both classify and bake shaders. For scenes with many spread-out objects, this drastically reduces per-brick evaluation cost.

**Dirty-region incremental baking:** Per-object FNV-1a hashing detects which objects changed between frames. The dirty region is the union of changed objects' old+new expanded AABBs, converted to brick coordinates. Only bricks within this region are reclassified and rebaked. The classify shader has an `incremental_mode` that reads old indirection values to reuse existing atlas slots for bricks that were already active, and allocates new slots via `brick_counter.next_slot` for newly active bricks. The atlas and indirection textures persist across frames — no clear, no copy. Falls back to full bake on: first frame, grid param change (origin/res/voxel_size), object add/remove, group structure change, grid objects present, dirty region > 50% of grid, atlas overflow, or excessive fragmentation (allocated > 3x active).

Combined: moving 1 of 1000 spread-out objects → only dirty-region bricks reclassified+rebaked, each evaluating ~1-3 objects via BVH. Performance nearly identical for 10 or 1000 objects.

| File | Change |
|------|--------|
| `source/blender/draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Per-brick AABB culling + incremental mode: `incremental_mode` uniform dispatches only over dirty brick range, reads old indirection to reuse atlas slots, allocates new via `atomicAdd(brick_counter.next_slot)` |
| `source/blender/draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Per-brick AABB culling in object loop (expand by 2-voxel overlap border). No changes for incremental — bake shader operates on whatever active_bricks list is provided |
| `source/blender/draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Classify: `IMAGE` changed to `read_write`, added `incremental_mode`/`dirty_brick_min`/`dirty_brick_max` push constants |
| `source/blender/draw/engines/sdf/sdf_shader_shared.hh` | `BrickCounter._pad0` renamed to `next_slot` for incremental slot allocation |
| `source/blender/draw/engines/sdf/sdf_engine.cc` | Per-object hashing (`compute_object_hash()`), dirty region computation in `end_sync()`, `dispatch_classify()` supports incremental mode, `draw()` has incremental pipeline path that skips indirection/atlas clear. Tracking state: `prev_object_hashes_`, `prev_bbox_mins/maxs_`, `total_allocated_slots_`, `incremental_bake_`, `dirty_brick_min/max_` |

---

## Native Picking Support for SDF Objects

SDF objects were invisible to Blender's GPU picking system because they had no overlay handler writing to the selection buffer. Added a two-pass SDF overlay: wireframe shapes for viewport display, and a solid ray-march pass that sphere-traces each SDF primitive analytically for pixel-perfect picking with exact depth.

### New Files (4)

| File | Purpose |
|------|---------|
| `source/blender/draw/engines/overlay/overlay_sdf.hh` | SDF overlay class (`Sdfs`) with wireframe + solid ray-march passes, following the Speaker/Empty pattern |
| `source/blender/draw/engines/overlay/shaders/overlay_sdf_pick_vert.glsl` | Vertex shader: transforms bounding cube vertices to clip space, passes world position to fragment |
| `source/blender/draw/engines/overlay/shaders/overlay_sdf_pick_frag.glsl` | Fragment shader: sphere-traces analytical SDF (box/sphere/capsule/torus), writes exact `gl_FragDepth` + `select_id_output()` on hit, `discard` on miss |
| `source/blender/draw/engines/overlay/shaders/infos/overlay_sdf_infos.hh` | Shader create info with push constants (sdf_type, sdf_size, sdf_bevel), DEPTH_WRITE(ANY), OVERLAY_INFO_VARIATIONS_MODELMAT |

### Modified Files (4)

| File | Change |
|------|--------|
| `source/blender/draw/engines/overlay/overlay_instance.hh` | Added `#include "overlay_sdf.hh"`, `Sdfs sdfs = {selection_type_}` member in `OverlayLayer` |
| `source/blender/draw/engines/overlay/overlay_instance.cc` | Added `layer.sdfs` calls in `begin_sync`, `object_sync` (`case OB_SDF`), `end_sync`, `draw_v3d` (draw + draw_line lambdas) |
| `source/blender/draw/engines/overlay/overlay_private.hh` | Added `StaticShader sdf_pick = shader_selectable("overlay_sdf_pick")` + `ensure_compile_async()` |
| `source/blender/draw/CMakeLists.txt` | Added overlay_sdf.hh, overlay_sdf_infos.hh, overlay_sdf_pick_vert.glsl, overlay_sdf_pick_frag.glsl |

---

## Cycles GPU Crash Fix

SDF objects caused Blender to crash when Cycles was set to GPU compute (OptiX/CUDA/HIP). The crash occurred because SDF geometry has no hardware BVH representation — GPU backends (OptiX, Metal, HIP) tried to build acceleration structures for SDF objects, hit null pointers, and crashed. CPU rendering worked fine because the CPU BVH2 path already calls `sdf_intersect_all()` for distance-field ray marching.

**Fix strategy:** Exclude SDF objects from GPU BVH building entirely (they use analytical ray marching, not hardware BVH). Also added `sdf_intersect_all()` calls to GPU kernel headers for future PTX recompilation.

### Modified Files (Cycles Host — Crash Fix)

| File | Change |
|------|--------|
| `intern/cycles/scene/object.cpp` | `is_traceable()` returns false for SDF objects — excludes them from GPU top-level acceleration structure |
| `intern/cycles/scene/geometry.cpp` | `need_build_bvh()` returns false for SDF geometry — prevents bottom-level BVH build attempts |
| `intern/cycles/device/optix/device_impl.cpp` | Added `is_sdf()` early return in BLAS building + null check for `blas` in TLAS instance loop |
| `intern/cycles/device/hiprt/device_impl.cpp` | Added null checks for `current_bvh` before accessing `geom_input` and `hiprt_geom` |

### Modified Files (Cycles GPU Kernels — Future PTX)

These changes add `sdf_intersect_all()` calls to GPU ray intersection functions. They take effect only when PTX/GPU kernels are recompiled (requires CUDA SDK for OptiX).

| File | Change |
|------|--------|
| `intern/cycles/kernel/device/optix/bvh.h` | Added SDF intersection in `scene_intersect()` and `scene_intersect_shadow()` |
| `intern/cycles/kernel/device/metal/bvh.h` | Added SDF intersection in `scene_intersect()` (both hit/no-hit paths) and `scene_intersect_shadow()` |
| `intern/cycles/kernel/device/hiprt/bvh.h` | Added SDF intersection after HIPRT traversal + early SDF-only path when `device_bvh == 0` |

### Modified Files (Draw — Pre-existing Build Fix)

| File | Change |
|------|--------|
| `source/blender/draw/engines/overlay/overlay_sdf.hh` | Fixed stale draw API: `DRW_cache_cube_get()` → `res.shapes.cube_solid.get()`, `ResourceHandle` → `ResourceHandleRange`, removed `select_id` param from `draw()` |

---

## Fix SDF Blend Cutoff and Voxel Artifacts

When two SDF objects overlap with blend > 0 (smooth union), hard black cutoffs and voxel grid artifacts appeared at the intersection zone. Three root causes fixed:

### Bug Fix: -2 Brick Immediate Hit (Garbage Data)

When a ray hit a `-2` (fully inside) brick, the ray marcher set `hit_slot = 0`, reading SDF data from the **first allocated active brick** — a completely unrelated brick. This produced garbage normals/colors, causing dark/black rendering.

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Removed `-2` immediate-hit block (lines 227-234). Rays now skip `-2` bricks via DDA like `-1` bricks. Simplified normal/color computation — removed dead `else` branches since `hit_slot` is always >= 0 for any hit |

### Bug Fix: Classify Threshold Missing Blend Margin

The smooth union pushes the iso-surface outward by up to `k/4` from either object's hard surface. The brick classification threshold (`brick_half_diag`) didn't account for this, causing bricks at the blend boundary to be classified as `-2` (fully inside) instead of active.

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Compute `max_blend_` across all objects. Add `max_blend * 0.25` to `brick_half_diag` in `dispatch_classify()`. Push `max_blend` uniform to both classify and bake shaders. New member `max_blend_` |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `max_blend` push constant to both `sdf_classify` and `sdf_bake` shader create infos |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Expanded per-brick AABB by `max_blend` (in addition to `brick_half_diag`) for object culling |

### Bug Fix: Bake AABB Missing Blend Margin

Per-brick AABB in the bake shader only included a 2-voxel overlap border. Objects contributing to smooth union but outside this narrow AABB were skipped, causing SDF discontinuities at brick boundaries.

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Expanded per-brick AABB by `max_blend` so objects contributing to smooth union are always evaluated |

---

## Pixel-Perfect SDF Selection via Object ID Atlas

Replaced the bounding-box GPU picking overlay for SDF objects with pixel-perfect selection through the baked atlas. The previous approach used instanced solid cubes (`extra_shape` pattern) that produced false positives — clicking empty space inside a torus hole, or outside a beveled shape's actual surface, incorrectly selected the object.

The new approach: during baking, each voxel records which object is closest (by index into the `objects[]` SSBO). During selection draws, a fullscreen march reads the object ID atlas and outputs the correct `select_id` per pixel. This gives pixel-perfect picking for all SDF shapes with zero additional sphere-tracing cost — the atlas is already baked.

### New Files (1)

| File | Purpose |
|------|---------|
| `draw/engines/sdf/shaders/sdf_select_march_frag.glsl` | Fullscreen fragment shader: DDA brick-walk (same as `sdf_march_frag.glsl`), reads `object_id_tx` at hit position, maps object index to `select::ID` via SSBO, writes to selection output buffer with correct depth |

### Modified Files (Draw)

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Added `object_id_tx_` (R32I, same dims as compact atlas). Created alongside `compact_atlas_tx_` in `ensure_compact_atlas()`. Bound as IMAGE(1) in `dispatch_bake()`. Freed in destructor and on resolution change. Added static getters (`sdf_atlas_get()`, `sdf_indirection_get()`, `sdf_object_id_atlas_get()`, `sdf_atlas_params_get()`, `sdf_object_count_get()`) for cross-engine access from the overlay |
| `draw/engines/sdf/sdf_engine.h` | Declared static getter functions for atlas textures, parameters, and object count |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `IMAGE(1, SINT_32, write, iimage3D, object_id_atlas)` to `sdf_bake` info. Added `sdf_select_march` shader create info with atlas samplers, `select_id_map_buf` SSBO, selection UBO/SSBO bindings, `draw_view` and `gpu_fullscreen` additional infos |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Tracks closest object index (`acc_obj_id`) during smooth union loop. For hard union: winner by distance. For smooth union: winner by >50% influence factor. Writes to `object_id_atlas` via `imageStore` |
| `draw/engines/overlay/overlay_sdf.hh` | Rewritten: replaced instanced bbox approach (`ShapeInstanceBuf`, `extra_shape`) with fullscreen selection march. Collects `select::ID` per object in `object_sync()`, captures selection buffer pointers in `end_sync()`, draws fullscreen triangle with manual texture/SSBO/UBO binding in `draw()` |
| `draw/CMakeLists.txt` | Added `sdf_select_march_frag.glsl` to shader file list |

### Architecture

```
Bake pass:  objects[] → per-voxel smooth union → compact_atlas (dist,R,G,B)
                                                  object_id_atlas (int, closest obj index)

Select pass: overlay calls SDF engine static API to get textures →
             fullscreen march shader (DDA brick walk) →
             on hit: read object_id_atlas → map to select_id → atomicMin/atomicOr
```

- **Object ID = index into `objects[]` SSBO** as ordered during `object_sync()` — the overlay builds a parallel `object_index → select::ID` mapping SSBO
- **Selection depth**: The shader inlines `select_id_output` logic using the computed SDF hit depth (not `gl_FragCoord.z` from the fullscreen triangle) for correct depth-aware picking
- **Ordering guarantee**: Both the SDF engine and overlay iterate depsgraph objects filtering `OB_SDF` in the same order, verified by `BLI_assert` in `end_sync()`

---

## Uniform World-Aligned Chunk System

Added world-aligned chunk snapping and per-axis grid resolution to the SDF atlas system.

**Problem:** The old cubic grid was always centered on the scene centroid with uniform resolution on all axes. This wasted bricks on thin/flat scenes and meant the grid origin shifted with every object movement.

**Solution:** Grid bounds are now snapped to world-aligned chunk boundaries via `floor`/`ceil`. Resolution is per-axis (non-cubic), adapting to scene shape. Resolution semantics are unchanged (`sdf_resolution` = total voxels across longest axis). `voxel_size` is still scene-dependent but the chunk-snapped grid gives stable brick boundaries for incremental baking.

### Key Benefits
- World-aligned grid: chunk boundaries are stable across small movements
- Non-cubic grids: per-axis resolution adapts to scene shape (thin scenes use fewer bricks)
- Dirty hash uses chunk-snapped `atlas_origin_` + `grid_res_` instead of raw scene bounds
- Textures are only freed/reallocated when grid dimensions actually change

### Modified Files

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_private.hh` | Added `SDF_MAX_GRID_RES = 128` constant (max bricks per axis) |
| `draw/engines/sdf/sdf_engine.cc` | `grid_res_` changed from `int` to `int3` for per-axis resolution. `end_sync()` rewritten: `voxel_size_ = max_axis / sdf_resolution_`, chunk-snapped grid bounds via `floor`/`ceil`, per-axis `grid_res_`, textures freed only when dimensions change. `sync_sdf_settings()` simplified (grid computation moved to `end_sync()`). Dirty hash uses `grid_res_` + `atlas_origin_` instead of raw `scene_min_/max_`. ~25 usage sites updated for per-axis int3: `ensure_indirection`, `clear_indirection`, `dispatch_classify`, `dispatch_bake`, `draw_march`, `rebuild_grid_batch_active`, `augment_indirection_for_grids`, `dispatch_grid_blends`, `perf_end_frame`, static grid resolution assignment |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | `MAX_BRICK_STEPS` increased from 128 to 256 (non-cubic grids up to 128/axis → max diagonal ~222) |

---

## Object ID Debug Visualization

Added a debug view mode that colorizes the SDF surface by object ID, useful for verifying the object ID atlas used by pixel-perfect selection.

### How to Use

In the Render Properties panel under SDF Settings, change **Debug View** from "Off" to "Object IDs". Each SDF object is rendered with a distinct color (golden-ratio hue hash) with simple lambertian shading for depth cues. Unclaimed voxels (object ID = -1) render as dark gray.

### Modified Files

| File | Change |
|------|--------|
| `makesrna/intern/rna_space.cc` | Added `OBJECT_IDS` enum item (value 2) to `sdf_debug_grid_items` |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `SAMPLER(4, isampler3D, object_id_tx)` and `PUSH_CONSTANT(int, debug_mode)` to `sdf_march` shader info |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Added section 5b: when `debug_mode == 1`, reads object ID from atlas at hit position, maps to a distinct hue via golden-ratio hash, applies lambertian shading, outputs color and depth, then returns early (skipping normal shading) |
| `draw/engines/sdf/sdf_engine.cc` | In `draw_march()`: binds `object_id_tx_` at sampler slot 4, pushes `debug_mode` uniform (`1` when `debug_grid_ == 2`, else `0`), unbinds `object_id_tx_` after draw |
| `scripts/startup/bl_ui/properties_render.py` | Changed label from "3D Voxel Grid" to "Debug View" |

---

## Active-Brick-Only Dispatch + Fixed Voxel Density

Replaced the grid_res³ bake/grid_blend dispatch (one workgroup per grid brick, most skipped) with active-brick-only dispatch (one workgroup per active brick). This makes bake cost proportional to surface area, not grid volume, enabling fixed voxel density without performance constraints.

**Problem:** At high resolutions or with spread-out scenes, grid_res³ dispatch launched millions of empty workgroups (95%+ skipped by the `if (slot < 0) return;` check in bake/grid_blend shaders). With fixed voxel density (`voxel_size = 1/resolution`), a 4 BU scene at resolution 256 = 128³ = 2M workgroups × 144 threads each = wasted GPU time.

**Solution:** The classify shader now writes active brick coordinates to an SSBO alongside slot allocation. Bake and grid_blend shaders read brick coordinates from this SSBO instead of using `gl_WorkGroupID` as the brick coordinate. Dispatch is now `GPU_compute_dispatch(sh, active_brick_count, 1, 1)` — only active bricks get workgroups.

### Key Changes

- **Fixed voxel density**: `voxel_size = 1.0 / sdf_resolution` (was: `max_axis / sdf_resolution`). Resolution now means "voxels per Blender unit" — consistent quality regardless of scene size
- **Auto-coarsening**: If grid exceeds 128 bricks on any axis, voxel_size is doubled iteratively until it fits
- **Active brick SSBO**: `ActiveBrick` struct (int4 coord) written by classify, read by bake/grid_blend
- **2D dispatch**: Uses `gl_WorkGroupID.x + gl_WorkGroupID.y * dispatch_width` to handle >65535 active bricks
- **Removed indirection_tx from bake/grid_blend**: No longer needed — slot index IS the brick index in the active bricks list

### Modified Files

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_shader_shared.hh` | Added `ActiveBrick` struct (int4 coord) for SSBO |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `STORAGE_BUF(2, write, ActiveBrick, active_bricks[])` to `sdf_classify`. Added `STORAGE_BUF(1, read, ActiveBrick, active_bricks[])` + `active_brick_count`/`dispatch_width` push constants to `sdf_bake` and `sdf_grid_blend`. Removed `indirection_tx` sampler and `grid_resolution` push constant from `sdf_bake`/`sdf_grid_blend`. Added `TYPEDEF_SOURCE` to `sdf_grid_blend` |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Writes `active_bricks[slot].coord = int4(brick, 0)` when allocating a slot |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Reads brick coordinate from `active_bricks[brick_idx]` instead of `gl_WorkGroupID`. Uses 2D dispatch indexing. Removed indirection texture lookup |
| `draw/engines/sdf/shaders/sdf_grid_blend_comp.glsl` | Same as bake: reads from active_bricks SSBO, 2D dispatch indexing, removed indirection lookup |
| `draw/engines/sdf/sdf_engine.cc` | Added `active_bricks_` SSBO (created in `dispatch_classify()`, freed in destructor). Changed `dispatch_bake()` and `dispatch_grid_blends()` from grid_res³ to 2D active-brick-count dispatch. `augment_indirection_for_grids()` appends grid brick coords to the SSBO. Restored fixed voxel density (`1.0/sdf_resolution`) with auto-coarsening loop. Removed indirection bindings from bake/grid_blend dispatch |

---

## BVH Object Culling + Dirty Brick Partial Rebake

Two optimizations to make SDF viewport performance constant regardless of object count (10 or 10,000).

**Problem:** The bake shader's inner loop evaluated ALL objects per voxel (O(N) per brick). With 10K objects × 8K active bricks, this meant ~138 billion SDF evaluations per frame. Additionally, moving a single object triggered a full rebake of all bricks.

### Phase 1: BVH Object Culling

SAH (Surface Area Heuristic) BVH built on CPU, uploaded as SSBO. Classify and bake shaders traverse the binary tree with a stack, testing node AABBs against brick AABBs. Each brick evaluates only ~5-15 nearby objects instead of all N. Falls back to linear scan when BVH is empty.

### Phase 2: Dirty Brick Partial Rebake

Per-object state tracking (session_uid → hash of position/size/bevel/blend/rotation/color) detects which objects changed. CPU-managed persistent slot assignment ensures bricks keep stable atlas positions across frames. Only bricks in the dirty region (union of old + new AABB of changed objects) are re-baked; clean bricks preserve their atlas data.

### Modified Files

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_shader_shared.hh` | Added `BVHNodeGPU` struct (float4 min_and_left, float4 max_and_right). Updated `ActiveBrick.coord.w` to store slot index |
| `draw/engines/sdf/shaders/sdf_lib.glsl` | Added `BVH_MAX_STACK`, `bvh_decode_int()`, `aabb_overlap()` helpers |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `STORAGE_BUF(3, read, BVHNodeGPU, bvh_nodes[])` + `bvh_node_count` push constant to `sdf_classify`. Added `STORAGE_BUF(2, read, BVHNodeGPU, bvh_nodes[])` + `bvh_node_count` push constant to `sdf_bake` |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Replaced linear `for(i=0;i<object_count;i++)` with stack-based BVH traversal. Falls back to linear scan when `bvh_node_count == 0` |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Same BVH traversal replacing per-voxel object loop. Reads slot from `active_bricks[].coord.w` instead of using brick index |
| `draw/engines/sdf/shaders/sdf_grid_blend_comp.glsl` | Reads slot from `active_bricks[].coord.w` instead of brick index |
| `draw/engines/sdf/sdf_engine.cc` | Added `build_bvh()` (top-down SAH 8-bin partitioning), `upload_bvh()`, BVH SSBO bindings in classify/bake dispatches. Added persistent slot system (`brick_slot_map_`, `free_slots_`, `total_slots_allocated_`), per-object dirty tracking (`prev_object_states_`, `prev_object_aabbs_`), `assign_persistent_slots()`, `compute_dirty_objects()`, `compute_dirty_brick_set()`, `dispatch_bake_dirty()`. Atlas preserved across frames when dimensions unchanged. Full rebake fallback on resolution/grid changes |

---

## Fix Cycles GPU SDF Crash and Rendering

Cycles GPU (OptiX/CUDA) crashed with "Illegal address" when rendering scenes with SDF objects. Three root causes fixed:

### Bug Fix: OptiX Anyhit Reads curve_segments for SDF Hits (CRASH)

The `__anyhit__kernel_optix_visibility_test` didn't handle SDF custom primitives. When OptiX traced with `ENFORCE_ANYHIT` (overriding the SDF instance's `DISABLE_ANYHIT`), the anyhit fell through to the `#ifdef __HAIR__` curve branch because `SDF_OPTIX_HIT_KIND (64)` and `PRIMITIVE_MOTION (1<<6 = 64)` share the same value, making `(hitKind & ~PRIMITIVE_MOTION) != PRIMITIVE_POINT` evaluate to true. This caused `curve_segments[0]` to be read — illegal memory access if no curves exist.

### Bug Fix: OptiX Intersection Program Was Debug Stub

The `__intersection__sdf` program always reported a hit at AABB entry with zeroed brick coordinates, rendering SDF objects as solid bounding boxes. Replaced with actual two-level DDA ray marching via `sdf_intersect()`.

### Bug Fix: Shader Flags Missing PRIMITIVE_SDF

`intersection_get_shader_flags` and `intersection_get_shader_from_isect_prim` in `bvh/util.h` didn't handle `PRIMITIVE_SDF`, causing shadow rays to get wrong shader flags (defaulting to shader 0).

### Modified Files

| File | Change |
|------|--------|
| `intern/cycles/kernel/device/optix/bvh.h` | Added `SDF_OPTIX_HIT_KIND` check in `__anyhit__kernel_optix_visibility_test` before curve branch. Replaced `__intersection__sdf` debug stub with actual DDA ray march calling `sdf_intersect()` |
| `intern/cycles/kernel/bvh/util.h` | Added `PRIMITIVE_SDF` case to `intersection_get_shader_flags` and `intersection_get_shader_from_isect_prim` — looks up shader via `KernelSDF.shader_offset` into `sdf_shader_map` |
| `intern/cycles/device/optix/device_impl.cpp` | SDF BLAS AABB now uses union of ALL SDF object bounds (via `BoundBox::grow()`), not just the first object. Fixes clipping of SDF objects whose bounds extend beyond the first object |

---

## Per-Brick AABB Hardware BVH Optimization (Cycles OptiX)

Major performance optimization for Cycles GPU (OptiX) SDF path tracing. Instead of one AABB covering the entire SDF atlas (forcing every ray through 256-step brick-level DDA), we now build one AABB per active brick. OptiX hardware BVH handles brick-level traversal, reducing intersection cost from O(256+24) to O(24) DDA steps.

Based on the SBS (Sparse Brick Set) approach from "Ray Tracing of Signed Distance Function Grids" (Hansson-Soderlund, Evans, Akenine-Moller, JCGT 2022).

### Architecture

1. **Brick map**: During SDF data upload, scan the indirection grid for active bricks (slot >= 0 or -2). Build `sdf_brick_map` array where each entry maps `primitiveIndex → (brick_linear, atlas_slot, sdf_index)`.
2. **Per-brick AABBs**: Instead of 1 AABB, create N AABBs (one per active brick). Each AABB covers the world-space extent of that brick (8 voxels wide).
3. **Hardware BVH**: OptiX builds an efficient BVH over all brick AABBs. Rays that miss a brick never invoke the intersection program.
4. **Per-brick intersection**: New `sdf_intersect_brick()` function skips brick-level DDA entirely. Uses `optixGetPrimitiveIndex()` to look up the brick map, then only does voxel-level DDA (max 24 steps).

### New Data Structures

| Item | Description |
|------|-------------|
| `sdf_brick_map` (int4 array) | Per-brick mapping: `(brick_linear, atlas_slot, sdf_index, 0)`. Indexed by OptiX primitive index. |
| `num_sdf_bricks` (KernelData field) | Number of active bricks (entries in sdf_brick_map). |

### New Functions

| Function | File | Description |
|----------|------|-------------|
| `sdf_intersect_brick()` | `intern/cycles/kernel/geom/sdf.h` | Per-brick voxel-level DDA only (24 steps max). Used by OptiX intersection program. |

### Modified Files

| File | Change |
|------|--------|
| `intern/cycles/kernel/types.h` | Added `num_sdf_bricks` to `KernelData`, replaced `pad2` |
| `intern/cycles/kernel/data_arrays.h` | Added `KERNEL_DATA_ARRAY(int4, sdf_brick_map)` |
| `intern/cycles/kernel/geom/sdf.h` | Added `sdf_intersect_brick()` for per-brick voxel-only DDA |
| `intern/cycles/kernel/device/optix/bvh.h` | `__intersection__sdf` now uses `sdf_brick_map` lookup + `sdf_intersect_brick()` instead of full DDA |
| `intern/cycles/device/optix/device_impl.cpp` | SDF BLAS now builds N AABBs (one per active brick) instead of 1. Added `#include "scene/sdf.h"` to access `SDFGeometry` for grid parameters. |
| `intern/cycles/scene/devicescene.h` | Added `device_vector<int4> sdf_brick_map` to `DeviceScene` |
| `intern/cycles/scene/devicescene.cpp` | Added `sdf_brick_map` initialization in `DeviceScene` constructor |
| `intern/cycles/scene/geometry.cpp` | Build `sdf_brick_map` during SDF upload: scan indirection for active bricks, upload mapping array |

### Bug Fix: Multi-SDF Crash (Illegal Address)

When multiple SDF Blender objects are in the scene, `geometry.cpp` and `device_impl.cpp` could pick **different SDFGeometry instances** for building the brick_map and BLAS respectively. `geometry.cpp` iterated `scene->geometry` and preferred the instance with `need_update_rebuild`, while `device_impl.cpp` iterated `bvh->objects` and took the first with valid bounds. If they picked instances with different indirection data (one stale, one fresh), the brick_map entry count didn't match the BLAS AABB count, causing `optixGetPrimitiveIndex()` to return out-of-bounds indices for the brick_map → GPU illegal address crash.

**Fix:** Added `is_active_atlas` flag to `SDFGeometry`. `geometry.cpp` marks the uploaded instance; `device_impl.cpp` finds and uses that same instance. Also fixed contradictory OptiX geometry flags (`DISABLE_ANYHIT` + `REQUIRE_SINGLE_ANYHIT_CALL`) on the SDF BLAS.

| File | Change |
|------|--------|
| `intern/cycles/scene/sdf.h` | Added `bool is_active_atlas` field to SDFGeometry class |
| `intern/cycles/scene/geometry.cpp` | After picking first_sdf, marks it `is_active_atlas = true` (others false) |
| `intern/cycles/device/optix/device_impl.cpp` | Now iterates `bvh->geometry` to find the SDFGeometry with `is_active_atlas = true`, then finds an Object referencing it for TLAS instance ID. Removed contradictory `OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT` from SDF geometry flags. |

### Bug Fix: SDF BLAS/Data Ordering (CUDA Illegal Address)

**Problem:** `is_active_atlas` was set in the SDF data upload section of `device_update()`, which runs AFTER `device_update_bvh()`. The OptiX TLAS code uses `is_active_atlas` to find the correct SDFGeometry for building the SDF BLAS. On first render, `is_active_atlas` was never set → no SDF BLAS → SDF invisible. On subsequent renders, BLAS used stale data from previous frame, causing brick count mismatch → CUDA illegal address crash.

**Fix:** Moved `first_sdf` determination and `is_active_atlas` flag setting to BEFORE the BVH build step. Also fixed `sdf_object_id` lookup to find the object using the active atlas (matching `device_impl.cpp`), not just any SDF object.

| File | Change |
|------|--------|
| `intern/cycles/scene/geometry.cpp` | Moved `first_sdf` selection and `is_active_atlas` assignment before BVH build. Fixed `sdf_object_id` to match active atlas object. |
| `intern/cycles/kernel/device/optix/bvh.h` | Fixed `__anyhit__kernel_optix_shadow_all_hit`: SDF branch now resolves brick_prim → sdf_index via `sdf_brick_map` lookup (was passing raw brick index as `prim`, causing `sdf_objects[brick_idx]` out-of-bounds in `intersection_get_shader_flags`). |

### Bug Fix: Stale `need_update_rebuild` Causes Wrong Atlas Selection (Multi-SDF Update)

**Problem:** Only the last-added SDF object updates when moved; earlier SDFs appear frozen. Root cause: SDF geometries don't build individual BVHs (`need_build_bvh()` returns false), so `compute_bvh()` is never pushed for them when only `need_update_rebuild` is set (not `is_modified()` via socket changes). Since `sync_sdf()` modifies raw arrays (not Node sockets), `is_modified()` returns false. The `need_update_rebuild` flag was never cleared, accumulating as a stale flag. On subsequent frames, the stale flag on the WRONG SDFGeometry caused the `first_sdf` selection to pick stale data instead of the freshly synced atlas.

**Fix:** Two changes in `geometry.cpp`:
1. **Explicit flag clearing:** Added `geom->need_update_rebuild = false` in the cleanup phase at the end of `device_update()`. For non-SDF types this is redundant (already cleared by `compute_bvh()`); for SDF types it prevents stale flags from carrying over to the next frame.
2. **Improved `first_sdf` selection:** Changed from "last with `need_update_rebuild`" to a priority-based selection: (a) freshly synced (`need_update_rebuild`) always wins, (b) previously active atlas (`is_active_atlas`) preferred for stability when no SDF changed, (c) first non-empty as fallback.

| File | Change |
|------|--------|
| `intern/cycles/scene/geometry.cpp` | Fixed `first_sdf` selection to use priority-based logic (fresh > previously-active > fallback). Added explicit `need_update_rebuild = false` in cleanup loop to prevent stale flags. |

### Bug Fix: CUDA SDF Invisible — Uninitialized `isect->t` in `scene_intersect`

**Problem:** CUDA backend renders no SDF objects (completely invisible, no errors). OptiX works fine. Root cause: in `scene_intersect()` (bvh/bvh.h), `isect->t` was never initialized before the SDF intersection check. The BVH traversal templates set `isect->t = ray->tmax` internally, but when `have_bvh_nodes` is false (SDF-only scene with no mesh objects), BVH traversal is skipped entirely. On CUDA, `ccl_optional_struct_init` is a no-op (empty `#define`), leaving `isect->t` as uninitialized stack garbage (usually 0). The SDF ray march then computes `t_exit = min(t_exit_grid, isect->t)` and immediately exits because `t_enter >= t_exit`. OptiX is unaffected because it uses hardware RT with custom intersection programs that manage ray distances via the OptiX API.

**Fix:** Initialize `isect->t = ray->tmax` at the top of `scene_intersect()`, before any traversal. This is harmless for BVH traversal (which already sets it internally) but ensures SDF intersection always has a valid max distance.

| File | Change |
|------|--------|
| `intern/cycles/kernel/bvh/bvh.h` | Added `isect->t = ray->tmax;` after `bool hit = false;` in `scene_intersect()` |

---

## Adaptive Octree Classification — Reduce Brick Count Explosion

With 2 SDF objects at 128 resolution: blend=1 gave 3,796 bricks, blend=5 gave 137,000 bricks — a 36x explosion. Two fixes applied:

### Fix 1: Remove `max_blend * 0.25` from classify threshold

The classify threshold formula included `+ max_blend * 0.25` to account for smooth union surface push. This was unnecessary: the classify shader evaluates the **blended** smooth-union SDF, so `acc_dist=0` IS the blended surface. The threshold only needs the brick half-diagonal (whether the surface could pass through the brick). Removing this term reduces the active-brick shell from ~21 bricks thick to ~1 brick thick at blend=5.

### Fix 2: Super-brick coarse check (workgroup-level early-out)

The classify workgroup is 4×4×4 = 64 threads, each processing one brick. This naturally maps to a "super-brick" of 32×32×32 voxels. Thread 0 evaluates the blended SDF at the super-brick center. If `|distance| > coarse_threshold`, all 64 threads exit immediately (Lipschitz bound guarantees no brick in the workgroup can contain surface). This eliminates per-brick evaluation for 90%+ of workgroups.

The SDF evaluation logic was extracted into an `evaluateSDF(center, query_min, query_max)` function reused by both coarse and fine checks.

### Modified Files

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Removed `max_blend_ * 0.25f` from `brick_half_diag` computation. Added `coarse_threshold` calculation (super-brick half-diagonal × 2 Lipschitz safety + brick_half_diag) and `GPU_shader_uniform_1f` call in `dispatch_classify()` |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Extracted `evaluateSDF()` function from inline BVH traversal + SDF evaluation. Added `shared float coarse_dist` for workgroup communication. Thread 0 evaluates at super-brick center; all threads early-out if `abs(coarse_dist) > coarse_threshold`, marking bricks as void (-1) or inside (-2) based on sign |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `PUSH_CONSTANT(float, coarse_threshold)` to `sdf_classify` shader info |

---

### Rotation-Invariant Grid Sizing with OBB Culling

**Problem:** Rotating an SDF object changes the world-space AABB (a box rotated 45° has an AABB √2× larger per axis), which changes the grid resolution and triggers a full rebake. The grid "breathes" as objects rotate.

**Solution:** Two complementary changes:

1. **Grid sizing** — Use circumscribed sphere of each object's local AABB instead of tight world AABB for `scene_min_`/`scene_max_` accumulation. `radius = length(local_extent)` is the tightest rotation-invariant bound (Cauchy-Schwarz). Grid resolution stays constant regardless of rotation.

2. **Shader culling** — Replace world-AABB overlap tests in classify/bake fallback paths with OBB tests. Transform query center to object-local space, test against local extent. Tighter culling for rotated objects.

**Trade-off:** For a cubic object, the sphere AABB is √3 ≈ 1.73× per axis → ~5.2× indirection volume. At 128 res with a 1 BU cube: 36³ → 60³ bricks of indirection (184KB → 844KB). Negligible — active bricks unchanged, only indirection texture overhead increases. `SDF_MAX_GRID_RES = 128` cap prevents runaway.

**What stays the same:** Tight world AABBs in `gpu_obj.bbox_min/bbox_max` (for BVH construction), `object_aabbs_` (dirty brick tracking), BVH traversal in shaders.

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Replaced tight world AABB accumulation for `scene_min_`/`scene_max_` with circumscribed sphere: `sphere_radius = length(local_extent)`, centered at object position. Tight world AABB retained for BVH and dirty-brick tracking |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Fallback linear scan: replaced world AABB overlap test with OBB culling — transform query center to object local space via `inverse_matrix`, test `abs(local_pos) > obb_extent + expand_radius` where `expand_radius = length(query_half)` |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Same OBB culling pattern in bake fallback: transform brick center to local space, test against local extent + brick half-diagonal sphere radius |

---

### BVH Bounds Debug Visualization

**Purpose:** Visual inspection of the SAH BVH structure and per-object OBBs. No way to verify BVH quality or OBB correctness without a debug view.

**What it does:** New debug mode "BVH Bounds" (value 3) alongside Off/Voxel Grid/Object IDs. Renders:
1. **BVH node AABBs** — world-aligned wireframe boxes, color-coded by tree depth (red=root → blue=leaves via HSV hue ramp)
2. **Per-object OBBs** — rotated wireframe boxes at leaf level, in white. Shows the actual oriented bounding box including bevel+blend expansion.

Single-pass front-only rendering using `GPU_SHADER_3D_FLAT_COLOR` (built-in shader with per-vertex `pos` + `color` attributes). Batches rebuilt on bake invalidation.

| File | Change |
|------|--------|
| `makesrna/intern/rna_space.cc` | Added `{3, "BVH_BOUNDS", ...}` to `sdf_debug_grid_items[]` enum |
| `draw/engines/sdf/sdf_engine.cc` | Added `bvh_batch_` member, `create_colored_line_batch()` helper, `rebuild_bvh_batch()` (DFS depth computation + HSV-colored AABB edges + white OBB edges), `draw_debug_bvh()` draw method. Wired into draw path, invalidation, and destructor |

---

### Fix Half-Voxel Shift + Dual Voxel Normals

**Problem 1 — Voxel shift:** The bake shader sampled SDF values at cell centers (`+ 0.5f`), but the march shader treated them as corner samples for trilinear interpolation. This caused the rendered surface to be shifted by half a voxel from the true analytic SDF position. Most visible at low resolutions.

**Fix:** Removed `+ 0.5f` from the world position computation in the bake shader. Values are now sampled at voxel corners (integer grid positions), matching the march shader's interpretation.

**Problem 2 — Normal discontinuity:** The existing `computeNormalCompact` computed the analytic gradient from a single voxel's 8 corners. While C2-continuous inside each voxel, this produces visible C0 discontinuity (faceting) at voxel boundaries.

**Fix:** Implemented the dual voxel normal interpolation method from Section 3.2 of "Ray Tracing of Signed Distance Function Grids" (Hansson-Soderlund, Evans, Akenine-Moller, JCGT 2022). The new `computeNormalDualVoxel` function:
1. Shifts the grid by half a voxel to define dual cells
2. Fetches a 3x3x3 = 27 texel neighborhood (all corners of the 8 overlapping voxels)
3. Computes the analytic gradient (Eq. 9-11) in each of the 8 overlapping voxels at the hit point (evaluated outside [0,1]^3 for 7 of 8 — the trilinear field extends naturally)
4. Trilinearly interpolates the 8 normalized normals (Eq. 12) based on position within the dual cell

Result: C0-continuous normals across voxel boundaries. Cost: 27 texel fetches + 8 gradient evaluations (~1-7% overhead per paper benchmarks). The 2-voxel overlap border in the brick storage is sufficient for all dual voxel accesses.

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Removed `+ 0.5f` from `world_pos` computation (line 130). SDF values now sampled at voxel corners instead of cell centers |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Replaced `computeNormalCompact` with `computeNormalDualVoxel`: 27-fetch 3x3x3 neighborhood, 8 analytic gradients, trilinear normal interpolation for C0 continuity across voxel boundaries |

---

### Smooth Auto-Coarsening for SDF Voxel Resolution

**Problem:** When SDF objects are moved apart, the scene bounds grow. Once the grid exceeds `SDF_MAX_GRID_RES=128` bricks on any axis, the old code doubled `voxel_size` iteratively (×2, ×4, ×8…). This created jarring quality snaps — e.g., from voxel_size 0.125 to 0.25 (halving resolution) the instant the scene crossed the threshold.

**Fix:** Replaced the doubling loop with a single exact computation. If the grid exceeds 128 bricks on any axis, `voxel_size` is scaled by `max_axis / 128.0` — the exact minimum factor needed. As the scene grows from 128 to 256 BU, voxel_size increases linearly (0.125 → 0.25) instead of snapping to 0.25 immediately.

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Replaced auto-coarsening doubling loop in `end_sync()` with smooth linear scaling: `voxel_size_ *= float(max_axis) / float(SDF_MAX_GRID_RES)`. Recomputes chunk_size, grid_min/max, grid_res once with the new voxel_size |

---

### FXAA Post-Processing Pass

**Problem:** SDF ray-marching renders directly to the viewport framebuffer with no anti-aliasing. At low voxel resolutions or on brick boundaries, the output shows jagged edges.

**Fix:** Added an FXAA post-processing pass using Blender's bundled NVIDIA FXAA 3.11 library. The march pass now renders to an offscreen RGBA16F texture instead of the default framebuffer. A fullscreen FXAA fragment shader then reads the offscreen texture and composites the anti-aliased result to the default framebuffer with alpha blending (so background/overlays show through). Uses max quality preset 39, hardcoded parameters. Toggled on/off via `View3DShading.sdf_fxaa` (default on). Timing integrated into the SDF performance debugger.

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_fxaa_frag.glsl` | **New.** Fragment shader wrapping `draw_fxaa_lib.glsl` with preset 39 (max quality). Reads offscreen march color texture, applies FXAA, outputs anti-aliased color |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `GPU_SHADER_CREATE_INFO(sdf_fxaa)` with `DEFINE_VALUE("FXAA_QUALITY__PRESET", "39")` — sampler, push constant for rcpFrame, fullscreen triangle |
| `draw/engines/sdf/sdf_engine.cc` | Added FXAA shader, offscreen texture/framebuffer, `ensure_fxaa_target()`, `draw_fxaa()`. March renders to offscreen texture when FXAA enabled, direct to default FB when disabled. FXAA composites to default FB with alpha blending. Added `PERF_PASS_FXAA` timing to performance overlay. Reads `sdf_fxaa` setting in `sync_sdf_settings()` |
| `draw/CMakeLists.txt` | Added `sdf_fxaa_frag.glsl` to shader sources |
| `makesdna/DNA_view3d_types.h` | Added `char sdf_fxaa` to `View3DShading` (reused padding from `short→char` change of `sdf_debug_grid`) |
| `makesdna/DNA_view3d_defaults.h` | Default `.sdf_fxaa = 1` (on) |
| `makesrna/intern/rna_space.cc` | Added `sdf_fxaa` boolean RNA property on `View3DShading` |
| `blenloader/intern/versioning_500.cc` | Set `sdf_fxaa = 1` for existing .blend files |
| `scripts/startup/bl_ui/properties_render.py` | Added FXAA checkbox to SDF Ray Marcher panel |

---

## SDF Pipeline Performance Optimizations

### Shared-Memory BVH Traversal in Bake Shader

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | **Shared-memory candidate list.** BVH traversal and candidate sorting now done once by thread 0 per workgroup, stored in `shared int shared_candidates[]`. All 144 threads read from shared memory instead of each traversing the BVH independently. Eliminates 143 redundant BVH traversals per brick |

### Dead Code Removal: Unused `coarse_threshold` Push Constant

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Removed computation and upload of `coarse_threshold` push constant (was set but never read by any shader) |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Removed `PUSH_CONSTANT(float, coarse_threshold)` from `sdf_classify` shader info |

### Per-Brick AABB Culling in Cycles CPU Bake

| File | Change |
|------|--------|
| `intern/cycles/blender/sdf.cpp` | **Phase 1 (classify):** Added per-brick AABB culling — each brick's expanded AABB (half-diagonal + max_blend) is tested against object AABBs before evaluating SDF distance. **Phase 2 (bake):** Added per-brick candidate list — computes brick storage AABB (including border + blend expansion), filters objects by AABB overlap once per brick, then only evaluates candidates at each voxel. Reduces per-voxel work from O(all_objects) to O(overlapping_objects) |

### Pre-Computed Slot Origin in Cycles DDA Kernel

| File | Change |
|------|--------|
| `intern/cycles/kernel/geom/sdf.h` | **Slot→atlas lookup optimization.** Added `sdf_slot_origin()` helper that computes the atlas base coordinate (3 integer divides + 3 modulos) once per brick entry. Updated `sdf_fetch_corners()`, `sdf_compute_normal()`, and `sdf_grid_to_compact()` to accept pre-computed `int3 slot_org` instead of `(brick_slot, bpa)`. Eliminates redundant div/mod per voxel step in all 4 DDA paths (primary, shadow, brick-only, and shader setup) |

### Single-Voxel Analytical Normal in Cycles DDA

| File | Change |
|------|--------|
| `intern/cycles/kernel/geom/sdf.h` | **Replaced dual-voxel normal computation (27 fetches + 8 gradients + trilinear blend) with single-voxel analytical gradient (8 fetches + 1 gradient).** Uses the 8 corners of the hit voxel and `sdf_trilinear_gradient()` for an exact gradient within the voxel. Trade-off: C0 continuity at voxel boundaries (vs C1 dual-voxel), visually negligible at typical resolutions. ~3.4x fewer texture fetches per primary ray hit |

### Instanced Atlas: float4→float Packing + Helper Dedup

| File | Change |
|------|--------|
| `intern/cycles/kernel/data_arrays.h` | Changed `sdf_shape_atlas` from `float4` to `float` — instanced atlas stores distance only, no color. **4x memory reduction and cache efficiency** |
| `intern/cycles/scene/devicescene.h` | Added missing `sdf_shape_*` device_vector declarations for instanced mode |
| `intern/cycles/scene/devicescene.cpp` | Added initializers for instanced device vectors |
| `intern/cycles/scene/sdf.h` | Changed `shape_atlas_data` from `vector<float4>` to `vector<float>` |
| `intern/cycles/blender/sdf.cpp` | Changed per-shape bake from `make_float4(d,0,0,0)` to scalar `d` |
| `intern/cycles/scene/geometry.cpp` | Changed upload from `sizeof(float4)` to `sizeof(float)` |
| `intern/cycles/kernel/geom/sdf.h` | Added `sdf_fetch_corners_shape()` helper with pre-computed plane offsets. Replaced 3x8=24 inline fetches with 3 helper calls. Removed `.x` member access (now scalar) |
| `intern/cycles/kernel/device/optix/bvh.h` | Fixed const-correctness for instanced SDF in shadow/closesthit/visibility handlers |

---

## Object-Space Atlas Baking (Instanced Mode)

Per-shape local-space atlas baking with instance-aware ray marching. When no objects use
smooth blending (blend=0), the engine switches from world-space to instanced mode:
each unique shape is baked once in normalized local space and shared by all instances.
Atlas memory scales with O(unique_shapes) instead of O(instances).

Based on "Ray Tracing of Signed Distance Function Grids"
(Hansson-Soderlund, Evans, Akenine-Moller, JCGT 2022).

### New Files

| File | Purpose |
|------|---------|
| `draw/engines/sdf/shaders/sdf_shape_bake_comp.glsl` | Per-shape local-space bake compute shader. Evaluates single sdBox+bevel per voxel, no BVH traversal. One workgroup per active brick, 12x12 threads cover XY, loops Z. Color not stored (comes from per-instance data at march time) |

### Modified Files

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_private.hh` | Added `SDF_MAX_SHAPE_GRID_RES = 32` constant for per-shape grid resolution cap |
| `draw/engines/sdf/sdf_shader_shared.hh` | Expanded `SDFShapeGPU` from 32→80 bytes: added `grid_params` (grid_res + indirection offset), `local_params` (origin + voxel_size), `atlas_params` (bricks_per_axis + active_brick_count). Renamed `atlas_index` to `slot_offset` |
| `draw/engines/sdf/shaders/sdf_lib.glsl` | Added `ray_aabb_intersect()` helper for BVH traversal in march shader |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `sdf_shape_bake` shader info (12x12x1 workgroup, per-shape push constants). Modified `sdf_march` to include 4 SSBOs (shapes[], instances[], bvh_nodes[], shape_indir[]) and 3 push constants (use_instanced, instance_count, bvh_node_count) |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | **Major rewrite.** Factored inner DDA into `dda_march()` function with mode parameter (indir_offset: -1=texture, ≥0=SSBO). Added `march_instanced()` with BVH traversal over instances (ordered child processing for early termination). `InstanceHit` struct tracks closest hit across instances. Normal transform: `normalize(mat3(transpose(world_to_local)) * local_normal)`. Main dispatches based on `use_instanced` flag |
| `draw/engines/sdf/sdf_engine.cc` | **Per-shape atlas pipeline.** Added `use_instanced_` mode selection (no blend, no grid objects). New functions: `compute_shape_atlas_params()` (per-shape grid_res/voxel_size), `shape_classify_cpu()` (CPU-side brick classification via sdBox), `upload_shape_indirection()` (flat int SSBO), `dispatch_shape_bake_all()` (per-shape bake dispatch). Modified `end_sync()` for mode selection, `draw()` for dual pipeline, `draw_march()` for SSBO binding and push constants. `ShapeInfo` expanded with grid_res, local_origin, local_voxel_size, indir_offset, slot_offset, active_brick_count |
| `draw/CMakeLists.txt` | Added `sdf_shape_bake_comp.glsl` to shader list |

---

## Two-Level BVH (TLAS/BLAS) for Cycles OptiX (Step 4)

Per-shape instanced BVH for Cycles OptiX rendering. When no objects use smooth blending,
each unique SDF shape gets its own BLAS (local-space brick AABBs), and each instance becomes
a TLAS entry with its world transform. Reduces BVH memory from O(total_bricks) to
O(unique_shape_bricks) + O(instances).

Dual pipeline: instanced mode (no blend) vs world-space mode (blend > 0), mutually exclusive.
CPU-side per-shape atlas baking with multithreaded `parallel_for`.

### New Kernel Structs

| Struct | Size | Fields |
|--------|------|--------|
| `KernelSDFShape` | 48 bytes | indirection_offset, atlas_offset, brick_map_offset, active_bricks, grid_res_xyz, bricks_per_axis, voxel_size, origin |
| `KernelSDFInstance` | 16 bytes | shape_id, shader_id, object_id, pad |

### New Kernel Data Arrays

| Array | Type | Purpose |
|-------|------|---------|
| `sdf_shape_objects` | `KernelSDFShape` | Per-shape atlas parameters |
| `sdf_shape_instances` | `KernelSDFInstance` | Per-instance shape/shader/object mapping |
| `sdf_shape_indirection` | `int` | Concatenated per-shape brick indirection grids |
| `sdf_shape_atlas` | `float4` | Concatenated per-shape SDF atlas data |
| `sdf_shape_brick_map` | `int4` | Concatenated per-shape brick map entries |

### Modified Files

| File | Change |
|------|--------|
| `intern/cycles/kernel/types.h` | Added `KernelSDFShape` (48B) and `KernelSDFInstance` (16B) structs. Replaced `pad3`/`pad4` in `KernelData` with `num_sdf_shapes`/`num_sdf_instances` |
| `intern/cycles/kernel/data_arrays.h` | Added 5 new kernel data arrays for per-shape instanced data |
| `intern/cycles/scene/sdf.h` | Extended `ShapeInfo` with per-shape atlas fields (grid_res, voxel_size, origin, bricks_per_axis, active_bricks, offsets). Added `use_instanced` flag and concatenated per-shape data vectors to `SDFGeometry` |
| `intern/cycles/blender/sdf.cpp` | Added per-shape local atlas baking in instanced mode: CPU-side classify (sdBox at brick centers), multithreaded bake via `parallel_for`, `local_to_world`/`world_to_local` transform computation per instance. Early return skips world-space bake path |
| `intern/cycles/scene/geometry.cpp` | Added instanced mode upload path: builds `KernelSDFShape`/`KernelSDFInstance` arrays, uploads concatenated per-shape indirection/atlas/brick_map data. Clears cross-mode arrays to avoid stale data |
| `intern/cycles/device/optix/device_impl.cpp` | Per-shape BLAS building (one BLAS per unique shape with local-space brick AABBs via `optixAccelBuild`). Per-instance TLAS entries with `local_to_world` transform. Preserved world-space BLAS path as fallback |
| `intern/cycles/device/optix/device_impl.h` | Added `sdf_shape_blas_handles` and `sdf_shape_blas_data` member vectors |
| `intern/cycles/kernel/device/optix/bvh.h` | `__intersection__sdf()`: branches on `num_sdf_shapes` for instanced vs world-space. Instanced uses `optixGetInstanceId()` → `KernelSDFInstance` → `KernelSDFShape` → per-shape brick_map. Updated anyhit (shadow_all_hit) and closest-hit to resolve instance_id/object_id in instanced mode |
| `intern/cycles/kernel/geom/sdf.h` | Added `sdf_intersect_brick_shape()` and `sdf_intersect_brick_shape_shadow()` for per-shape DDA. Updated `sdf_shader_setup()` to branch on instanced mode for normal computation and shader resolution |

---

## Incremental Baking Pipeline (Step 5)

Avoid full re-bake when only some objects move or change. Three optimizations:

### Draw Engine: World-Space Partial Bake
When `assign_persistent_slots()` succeeds (atlas not resized, no structural change):
1. `compute_dirty_objects()` identifies which objects changed since last frame
2. `compute_dirty_brick_set()` computes brick ranges affected by dirty objects (old + new AABB)
3. `build_dirty_bricks_ssbo()` filters active bricks to dirty subset
4. `dispatch_bake_dirty()` rebakes only dirty bricks (~5% of atlas typical)

Falls back to full bake when: atlas resized, grid objects present, all objects dirty, or `force_full_rebake_`.

### Draw Engine: Instanced Shape Skipping
Tracks `prev_shape_fingerprints_` between frames. When a shape's fingerprint matches the
previous frame (same type + normalized size + bevel), its atlas data is reused — only new
or changed shapes get rebaked. Moving instances don't trigger rebake since atlas is in local space.

### Cycles: Scene Hash Early Exit
FNV-1a hash over all SDF objects (position, transform, size, bevel, blend, color, type).
When `prev_scene_hash` matches, `sync_sdf()` returns immediately, skipping the entire CPU bake.

### Modified Files

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Wire up incremental bake in world-space pipeline: `assign_persistent_slots()` → `compute_dirty_objects()` → `compute_dirty_brick_set()` → `dispatch_bake_dirty()`. Instanced pipeline: `prev_shape_fingerprints_` tracking, `dispatch_shape_bake_all()` accepts optional dirty set to skip clean shapes |
| `intern/cycles/blender/sdf.cpp` | Scene hash computation after object collection, early return when hash matches `prev_scene_hash` |
| `intern/cycles/scene/sdf.h` | Added `prev_scene_hash` member to `SDFGeometry` for cross-sync change detection |

---

## Fix: CPU/CUDA Instanced SDF Rendering (Cycles)

Instanced SDF objects (the default mode when no blend > 0) were invisible on all backends
(CPU, CUDA, OptiX). The instanced intersection path only existed in OptiX (via hardware TLAS);
CPU/CUDA had no codepath for it.

### Root Causes

1. **Missing intersection dispatch**: `scene_intersect` in `bvh/bvh.h` only checked `num_sdfs > 0`.
   In instanced mode `num_sdfs = 0` (no world-space atlas), so SDF intersection was skipped entirely.
2. **Missing transforms in kernel struct**: `KernelSDFInstance` had no world-to-local/local-to-world
   transforms — OptiX gets these from hardware TLAS, but CPU/CUDA need them explicitly.
3. **Wrong local_hit in shader_setup**: `sdf_shader_setup` used `sd->P` (world-space) as the
   local-space hit point for normal computation, producing garbage normals.
4. **Empty bounds**: `compute_bounds()` returned empty for instanced mode since `grid_res == 0`.

### Modified Files

| File | Change |
|------|--------|
| `intern/cycles/kernel/types.h` | Added `Transform world_to_local` and `Transform local_to_world` fields to `KernelSDFInstance` |
| `intern/cycles/scene/geometry.cpp` | Upload instance transforms from `InstanceInfo` to `KernelSDFInstance` |
| `intern/cycles/kernel/geom/sdf.h` | Added `sdf_fetch_shape_indirection` helper, `sdf_intersect_instanced`, `sdf_intersect_all_instanced`, shadow variants; fixed `sdf_shader_setup` to transform `sd->P` to local space |
| `intern/cycles/kernel/bvh/bvh.h` | Added `else if (num_sdf_shapes > 0)` branches in `scene_intersect` and `scene_intersect_shadow` for instanced path |
| `intern/cycles/scene/sdf.cpp` | Added instanced mode branch in `compute_bounds()` using `scene_min`/`scene_max` |

---

## Unified Cross-Backend SDF Performance Timers

Replaced dual-path GL queries / SSBO fence timer system with a single unified approach using `GPU_finish()` + `BLI_time_now_seconds()`. Fixes macOS Metal backend where `GPU_storagebuf_read()` doesn't reliably drain the pipeline.

### Modified Files

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Removed `GPU_context.hh` and `<epoxy/gl.h>` includes. Removed GL double-buffer infrastructure (`perf_use_gl_queries_`, `perf_queries_[2][N]`, `perf_queries_valid_[2]`, `perf_frame_idx_`, `perf_fence_ssbo_`, `perf_pass_start_time_`). Flattened `perf_pass_active_[2][N]` → `perf_pass_active_[N]`, `perf_slot_baked_[2]` → `perf_currently_baking_`. Added `perf_pass_start_[N]` per-pass timestamps. Rewrote `perf_begin_pass()`/`perf_end_pass()` to use `GPU_finish()` + wall-clock. Simplified `perf_ensure_queries()` → `perf_init()`, `perf_end_frame()`, `perf_cleanup()`. Eliminated all raw GL calls |

---

## Dual Voxel Normal Interpolation

Replaced normal computation with the dual voxel method from Section 3.2 of
"Ray Tracing of Signed Distance Function Grids" (Hansson-Soderlund, Evans,
Akenine-Moller, JCGT 2022). Instead of computing the trilinear gradient from
a single voxel (C0-discontinuous at boundaries), the new method:

1. Finds the dual voxel (shifted by half a voxel) containing the hit point
2. Evaluates the analytic trilinear gradient in each of the 2×2×2 overlapping voxels
3. Normalizes each gradient independently
4. Trilinearly blends using the hit point's position within the dual voxel

This yields C0-continuous normals across voxel boundaries. Cost: 27 fetches
(3×3×3 neighborhood) + 8 gradient evaluations + 8 normalizations, vs the
previous 8 fetches + 1 gradient.

The draw engine's previous B-spline gradient approach (quadratic B-spline
reconstruction) has been replaced with the paper's method for consistency
between Cycles and the draw engine.

### Modified Files

| File | Change |
|------|--------|
| `intern/cycles/kernel/geom/sdf_lib.h` | Added `sdf_dual_voxel_normal()`: takes pre-fetched 27 values + grid_pos + dual_center, evaluates 8 trilinear gradients in overlapping voxels, normalizes each, trilinearly blends. Updated `sdf_trilinear_gradient()` comment to note p may extend outside [0,1]^3 |
| `intern/cycles/kernel/geom/sdf.h` | Rewrote `sdf_compute_normal()`: now fetches 3×3×3 neighborhood via `sdf_fetch_distance()` loop, delegates to `sdf_dual_voxel_normal()`. Rewrote instanced mode normal computation: same 3×3×3 fetch from shape atlas (uint16 decode), same `sdf_dual_voxel_normal()` call, then world-space transform |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Replaced B-spline `computeDualVoxelNormal()` with paper's dual voxel method: fetches 27 values into array, evaluates 8 analytic gradients via `trilinearGradient()`, normalizes each, trilinearly blends. Same 27 texture fetches, different math (8 gradients + 8 sqrts vs separable tensor-product) |

---

## Fix: SDF Shader Resolution and Material Change Detection

**Problem:** SDF objects render black with default material, and material changes don't update the render (must duplicate the SDF to force update).

**Root causes:**
1. **Stale shader IDs:** `sync_sdf()` caches shader IDs during the Blender sync phase, BEFORE `shader_manager->device_update_pre()` reassigns `shader->id` values. The uploaded shader IDs point to the wrong shader (e.g. background shader → black).
2. **Missing shading recalc trigger:** When a material is changed on an SDF object, `is_updated_shading()` is true but the code only triggers `geometry_map.set_recalc` for geometry or transform changes — not shading. So `sync_sdf` is never re-called for material-only changes.

**Fix:**
1. Store `Shader*` pointers alongside cached shader IDs, re-resolve IDs at device upload time (when `shader->id` values are correct).
2. Add `is_updated_shading() && is_sdf_object` as a trigger for `geometry_map.set_recalc`.

### Modified Files

| File | Change |
|------|--------|
| `intern/cycles/scene/sdf.h` | Added `Shader *shader` to `InstanceInfo`, added `vector<Shader *> object_shaders` for per-object shader pointers |
| `intern/cycles/scene/sdf.cpp` | Clear `object_shaders` in `clear()` |
| `intern/cycles/blender/sdf.cpp` | Store `Shader*` in instances and `object_shaders` in all code paths (instanced bake, world-space bake, early return). Fixed instanced mode `object_shader_ids` from hardcoded 0 to actual resolved IDs |
| `intern/cycles/scene/geometry.cpp` | Re-resolve shader IDs from `Shader*` at device upload time via `get_shader_id()` for both instanced and world-space paths |
| `intern/cycles/blender/sync.cpp` | Added `(b_update.is_updated_shading() && is_sdf_object)` to geometry re-sync trigger |

---

## Code Audit Fixes (2026-03-09)

Systematic audit of SDF integration across all Blender layers. Found and fixed missing registrations, dead code, strict aliasing UB, and enum duplication.

### Critical Fixes

| File | Fix |
|------|-----|
| `depsgraph/intern/depsgraph_tag.cc` | Added `case OB_SDF:` and `case ID_SF:` to `geometry_tag_to_component()`. Without these, `DEG_id_tag_update(..., ID_RECALC_GEOMETRY)` from `rna_SDF_update()` mapped to `NodeType::UNDEFINED`, potentially causing missed viewport redraws via the depsgraph path. |
| `blenkernel/intern/object_update.cc` | Added `case OB_SDF:` in geometry eval dispatch (calls `BKE_sdf_data_update`) and batch cache dirty tag (no-op, SDF has no batch cache). Added `#include "BKE_sdf.hh"`. |

### Warning Fixes

| File | Fix |
|------|-----|
| `draw/engines/sdf/sdf_engine.cc` | Replaced 24 `*reinterpret_cast<const uint32_t *>(&float)` strict aliasing violations with `memcpy`-based `float_as_uint()` helper in hash computation. |
| `makesrna/intern/rna_sdf.cc` + `editors/object/object_sdf.cc` | Consolidated duplicated `rna_enum_sdf_type_items` into single definition in `rna_sdf.cc`, declared via `RNA_enum_items.hh`. Unified label from "Box" (RNA) / "Cube" (editor) to "Cube" everywhere. |

### Info Fixes

| File | Fix |
|------|-----|
| `makesdna/DNA_sdf_types.h` | Removed dead `SDF_DS_EXPAND` flag enum (referenced `#SDF.flag` but struct has no `flag` field). Added comment for enum value gap (2, 3 reserved from removed Cylinder/Cone types). |
| `blenkernel/intern/sdf.cc` | Removed unused `#include <optional>`. |
| `makesrna/RNA_enum_items.hh` | Added `DEF_ENUM(rna_enum_sdf_type_items)` for shared enum access. |

---

## Performance Optimizations (2026-03-09)

Performance audit of SDF draw engine. Targeted GPU stalls, shader ALU waste, unnecessary allocations, and per-frame overhead.

### Shader Optimizations

| File | Change | Impact |
|------|--------|--------|
| `shaders/sdf_fxaa_lib.glsl` | Replaced `pow(x, 1/2.4)` sRGB luma with `dot(rgb, vec3(0.299, 0.587, 0.114))` — eliminates 33+ `pow()` calls per pixel. Reduced preset from 29 (12 search steps) to 25 (8 steps) with threshold 0.063→0.125. | ~2-5% frame time |
| `shaders/sdf_lib.glsl` | Removed dead code: `sdf_cbrt()` and `solveCubicFirstRoot()` (Marmitt+NR solver is used instead). Reduces shader binary size and compile time. | Shader compile |
| `shaders/sdf_grid_blend_comp.glsl` | Removed dead bounds check (workgroup size matches `BRICK_STORAGE`). Added explicit `textureLod(..., 0.0)` for defined behavior in compute shaders. | Minor |

### Engine Optimizations

| File | Change | Impact |
|------|--------|--------|
| `sdf_engine.cc` — `clear_indirection()` | Replaced CPU-side `Vector<int32_t>` (up to 8MB at 128^3) with `GPU_texture_clear()` — zero CPU allocation. | ~0.1-0.5ms |
| `sdf_engine.cc` — `dispatch_shape_bake_all()` | Eliminated per-shape SSBO alloc/free cycle by uploading all active bricks once and using `brick_offset` push constant. Consolidated N per-shape memory barriers into single barrier after all shapes. | ~0.5-3ms with many shapes |
| `sdf_engine.cc` — `upload_shapes_instances()` | Added SSBO reuse: track count, only recreate when size changes, otherwise `GPU_storagebuf_update()`. Applied same pattern to shape/instance/shape_indirection SSBOs. | ~0.05-0.2ms per dirty frame |
| `sdf_engine.cc` — `dispatch_classify()` | Moved `max_blend_` computation from draw-time O(N) scan to incremental update during `object_sync()`. | Minor |

### Overlay Optimizations

| File | Change | Impact |
|------|--------|--------|
| `overlay_sdf.hh` — `end_sync()` | SSBO reuse for select_id_map (track count, update in-place). Cached 11 shader binding/uniform slots after shader creation (eliminates per-draw string hashing). | ~10-25us per selection draw |
| `overlay_sdf.hh` — `draw()` | Use cached slots + `GPU_shader_uniform_*_ex()` with integer locations instead of string lookups. Cache texture pointers to avoid double cross-TU fetch for unbind. | ~10-25us per selection draw |

### Shader Info Changes

| File | Change |
|------|--------|
| `shaders/infos/sdf_shader_infos.hh` | Added `brick_offset` push constant to `sdf_shape_bake` for per-shape SSBO offset. |
| `shaders/sdf_shape_bake_comp.glsl` | Use `active_bricks[brick_idx + brick_offset]` to index into shared SSBO. |

---

## Full CSG Boolean & Blend Operations (Phase 12 — All MathOPS blend/CSG in native engine)

Implemented all SDF boolean and blending operations from the MathOPS addon in the native
Blender draw engine. Previously only smooth union was hard-coded; now the engine supports
all 4 blend types × 4 CSG operations with per-object dispatch.

### Operations Supported

| CSG Operation | Blend: Linear | Blend: Smooth | Blend: Chamfer | Blend: Round |
|---------------|---------------|---------------|----------------|--------------|
| Union         | `min(d1,d2)` | `opSmoothUnion` | `opChamferUnion` | `opRoundUnion` |
| Subtract      | `max(d1,-d2)` | `opSmoothSubtraction` | `opChamferSubtraction` | `opRoundSubtraction` |
| Intersect     | `max(d1,d2)` | `opSmoothIntersection` | `opChamferIntersection` | `opRoundIntersection` |
| Shell         | `opOnion` | (planned) | (planned) | (planned) |

### DNA Changes

| File | Change |
|------|--------|
| `makesdna/DNA_sdf_types.h` | Added `SDF_CSG_SHELL = 3` to `eSDFCSGOperation`. Replaced `_pad1[4]` with `float shell_distance` |
| `makesdna/DNA_sdf_defaults.h` | Added `.shell_distance = 0.0f` default |

### RNA Changes

| File | Change |
|------|--------|
| `makesrna/intern/rna_sdf.cc` | Added Shell to CSG enum items. Added SVG icons to all blend type and CSG enum items. Added `shell_distance` float property (range -5.0 to 5.0) |

### Icon Registration (8 new icons)

| File | Change |
|------|--------|
| `editors/include/UI_icons.hh` | 8 new DEF_ICON entries: `SDF_BLEND_LINEAR`, `SDF_BLEND_SMOOTH`, `SDF_BLEND_CHAMFER`, `SDF_BLEND_ROUND`, `SDF_CSG_UNION`, `SDF_CSG_SUBTRACT`, `SDF_CSG_INTERSECT`, `SDF_CSG_EXTRUDE` |
| `editors/datafiles/CMakeLists.txt` | 8 SVG filenames added to `SVG_FILENAMES_NOEXT` |

### GPU Struct Changes

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_shader_shared.hh` | Added `blend_type`, `csg_operation`, and `shell_distance` to `SDFObjectGPU` (replaces padding). Added `blend_type`, `csg_operation` to `SDFInstanceGPU` |

### GLSL Shader Changes

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_lib.glsl` | Ported all blend functions from MathOPS `sdf_ops.glsl`: smooth (union/sub/intersect), chamfer (union/sub/intersect + smooth variants), round/spherical (union/sub/intersect + smooth variants), onion shell. Added `combineCSG()` dispatch function with `shell_dist` parameter. **Bug fix:** Shell now applies `opOnion(d2, shell_dist)` to the new object then unions with scene (was incorrectly applying to accumulator) |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | **Bug fix:** Replaced hard-coded union with `combineCSG()` dispatch. Subtraction/intersection skip empty accumulators. Shell uses `obj.shell_distance` for onion thickness |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Added `blend_type`, `csg_operation`, and `shell_distance` to `SharedObj`. Per-object CSG dispatch. Subtraction/intersection skip empty accumulators. Shell uses `shell_distance` for onion |
| `draw/engines/sdf/shaders/sdf_grid_blend_comp.glsl` | CSG dispatch with `grid_shell_distance` push constant. Subtraction/intersection skip empty atlas voxels |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `grid_blend_type`, `grid_csg_operation`, and `grid_shell_distance` push constants to `sdf_grid_blend` |

### Engine Changes

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Pass `blend_type`/`csg_operation`/`shell_distance` from SDF DNA to GPU objects, instances, and grid objects. Intersection objects get infinite AABB to be always evaluated. Shell objects get AABB expanded by `shell_distance`. Scene hash includes `shell_distance`. Grid blend dispatch passes `grid_shell_distance` push constant |

---

## Session: CSG Two-Pass Evaluation + Shell Fix + UI Redesign (2026-03-09)

### Critical Bug Fixes

**Two-pass evaluation in classify and bake shaders:**
Object ordering caused subtraction/intersection objects to be processed before any union object existed, resulting in those objects rendering as solid blobs or disappearing entirely.

**Fix:** Both `sdf_classify_comp.glsl` and `sdf_bake_comp.glsl` now use two-pass evaluation:
- Pass 1: Accumulate only union objects to build the base SDF
- Pass 2: Apply subtraction/intersection/shell modifiers to the built base

**Shell algorithm rewrite:**
Shell was using `opOnion(d2, shell_dist)` which applies a hollow shell to the new object. The correct MathOPS algorithm is: union the shape with base, then intersect with a limit surface (`base - thickness`), producing an expanded intersection region.

### Shader Changes

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Two-pass evaluation: pass 1 unions only, pass 2 modifiers. 6-param `combineCSG` call |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Two-pass evaluation matching classify. `SharedObj` struct gets `shell_distance` field |
| `draw/engines/sdf/shaders/sdf_lib.glsl` | `combineCSG` rewritten to 6 params (`d1, d2, op, bt, k, shell_dist`). Shell case implements MathOPS algorithm: union + intersect with limit surface `(d1 - abs(shell_dist))` |
| `draw/engines/sdf/shaders/sdf_grid_blend_comp.glsl` | Shell init uses `grid_dist` (needs base). Empty accumulator skips subtract/intersect |

### UI Redesign

| File | Change |
|------|--------|
| `scripts/startup/bl_ui/properties_data_sdf.py` | Panel renamed "Blending" → "Operation". Icon-only grids for CSG operation and blend type (`prop_enum` with `text=""`). Title labels "Boolean" and "Blend Type". Blend amount always shown; shell_distance shown only when CSG operation is SHELL |

### SVG Icon Redesign (all 8 icons)

All icons redesigned with bordered/outlined style using `fill-rule="evenodd"` (outer path minus inner path = outlined shape with transparent background).

| File | Style |
|------|-------|
| `sdf_csg_union.svg` | Two overlapping outlined squares, overlap region open |
| `sdf_csg_subtract.svg` | A outline (with overlap cut out) + B cutter outline |
| `sdf_csg_intersect.svg` | A and B outlined, overlap region filled solid |
| `sdf_csg_extrude.svg` | A and B outlined, expanded overlap region outlined |
| `sdf_blend_linear.svg` | L-profile outline with sharp 90-degree corner |
| `sdf_blend_smooth.svg` | L-profile outline with curved (Bezier) transition |
| `sdf_blend_chamfer.svg` | L-profile outline with 45-degree diagonal cut |
| `sdf_blend_round.svg` | L-profile outline with circular arc corner |

---

## UI Improvements, Blend Fixes, and Shell Voxel Fix

### UI: Full-width icon buttons, pie menus, defaults

| File | Change |
|------|--------|
| `source/blender/editors/interface/interface_layout.cc` | Exempt `ButType::Row` (expanded enum buttons) from `fixed_size_set(true)`, allowing icon-only expanded enums to fill available width |
| `source/blender/makesrna/intern/rna_sdf.cc` | Added icons to shape type enum items (ICON_MESH_CUBE, ICON_MESH_UVSPHERE, ICON_MESH_CAPSULE, ICON_MESH_TORUS) |
| `scripts/startup/bl_ui/properties_data_sdf.py` | Full-width icon-only enum rows with `scale_y=1.6`, pie menu operators (Tab=CSG, Shift+Tab=Blend), blend slider disabled for Linear, shell thickness only shown for Shell |
| `scripts/presets/keyconfig/keymap_data/blender_default.py` | Added SDF pie menu keymaps (Tab and Shift+Tab) in `km_object_non_modal()` before mode-switch entries |
| `source/blender/makesdna/DNA_sdf_defaults.h` | Changed defaults: `blend=0.1f`, `blend_type=1` (Smooth), `shell_distance=0.2f` |
| `release/datafiles/icons_svg/sdf_*.svg` (8 files) | Redesigned with thin stroke-based rendering (`stroke-width="90"`) for pixel-sharp display at 20×20px |

### Blend math: round operations and shell blend clamp

| File | Change |
|------|--------|
| `source/blender/draw/engines/sdf/shaders/sdf_lib.glsl` | Replaced round blend ops with MathOPS `opUnionIRound` (outward convex fillet using mirror2D). Added `opDifferenceIRound`, `opIntersectIRound`, `opSmoothRoundUnionInverted`. Shell blend clamped: `k = clamp(k, 0.01, abs(shell_dist)*2)`. Full shell CSG implementation (union + limit surface intersection). `combineCSG` now takes `shell_dist` parameter |

### Shell voxel classification fix

| File | Change |
|------|--------|
| `source/blender/draw/engines/sdf/sdf_engine.cc` | Track `max_shell_distance_` across all shell objects (like `max_blend_`). Pass as uniform to classify and bake shaders |
| `source/blender/draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `PUSH_CONSTANT(float, max_shell_distance)` to classify and bake shader infos |
| `source/blender/draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Brick AABB expansion includes `max_shell_distance`. Surface test threshold: `brick_half_diag + max_shell_distance` ensures bricks near shell boundaries are activated |
| `source/blender/draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Brick AABB expansion includes `max_shell_distance` for candidate object collection |

---

### Text SDF primitive (`SDF_TYPE_TEXT = 7`)

Adds a Text SDF primitive that rasterizes font glyphs to a 2D SDF atlas via BLF, then extrudes along Z on the GPU (text lies in XY plane, facing up). Follows the existing `grid_blend` pattern for blending external texture data into the brick atlas.

**New Files:**

| File | Purpose |
|------|---------|
| `source/blender/blenkernel/intern/sdf_text.cc` | CPU-side 2D SDF atlas generation: BLF text rasterization + 8SSEDT distance transform |
| `source/blender/draw/engines/sdf/shaders/sdf_text_blend_comp.glsl` | GPU compute shader: samples 2D text SDF atlas at XY, extrudes Z, blends into brick atlas via CSG |

**Modified Files:**

| File | Change |
|------|--------|
| `source/blender/makesdna/DNA_sdf_types.h` | Added `SDF_TYPE_TEXT = 7`, `eSDFTextAlign` enum, text fields to `SDF` struct (`text`, `text_font_path`, `text_depth`, `text_size`, `text_spacing`, `text_line_spacing`, `text_align`, `text_sdf_resolution`) |
| `source/blender/makesdna/DNA_sdf_defaults.h` | Added defaults for all text fields |
| `source/blender/blenkernel/BKE_sdf.hh` | Added `BKE_sdf_text_generate_atlas()` and `BKE_sdf_text_update_bounds()` declarations |
| `source/blender/blenkernel/intern/sdf.cc` | Text string lifecycle: deep copy, free, blend_write/read |
| `source/blender/blenkernel/CMakeLists.txt` | Added `intern/sdf_text.cc` to sources |
| `source/blender/makesrna/intern/rna_sdf.cc` | Added TEXT enum item, RNA properties for all text fields, type update handler; auto-calls `BKE_sdf_text_update_bounds()` in `rna_SDF_update` and `rna_SDF_type_update` |
| `source/blender/editors/object/object_sdf.cc` | Added TEXT to add operator name table; calls `BKE_sdf_text_update_bounds()` for correct initial size |
| `source/blender/draw/engines/sdf/sdf_engine.cc` | `TextBlendObject` struct, `process_text_object()`, `dispatch_text_blends()`, text hash/cache in `end_sync()` |
| `source/blender/draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `sdf_text_blend` shader create info |
| `source/blender/draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Added `SDF_TYPE_TEXT` — evaluates as box with `max(dist, 0.0)` to force all interior bricks to be allocated (text has internal glyph detail) |
| `source/blender/draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Added `SDF_TYPE_TEXT` — skipped in both CSG passes (text_blend fills actual values) |
| `source/blender/draw/engines/sdf/shaders/sdf_shape_bake_comp.glsl` | Added `SDF_TYPE_TEXT` — returns 1e10 (skip) |
| `source/blender/draw/engines/sdf/shaders/sdf_outline_march_frag.glsl` | Added `SDF_TYPE_TEXT` define for consistency |
| `source/blender/draw/CMakeLists.txt` | Added `sdf_text_blend_comp.glsl` to shader list |
| `scripts/startup/bl_ui/properties_data_sdf.py` | Added TEXT to property panel poll, `draw_text()` method with searchable system font dropdown, `SDF_OT_select_font` operator |

**Key design decisions:**

1. **Auto-sizing**: `BKE_sdf_text_update_bounds()` auto-computes `sdf->size` from BLF text metrics whenever text properties change. This keeps the AABB accurate without manual user intervention.
2. **Interior brick allocation**: Classify shader uses `max(sdBox(...), 0.0)` for text — forces ALL interior bricks to be allocated. Regular SDF primitives only need surface bricks, but text has internal glyph detail the box SDF can't predict.
3. **Bake skip**: Text objects are skipped in both CSG passes of the bake shader. Without this, text's 1e10 return value would erase geometry via `max(acc, 1e10)` in intersect operations.
4. **Font selection**: Searchable dropdown (`invoke_search_popup`) enumerates system fonts from OS font directories (Windows/macOS/Linux). Cached for 60 seconds to avoid repeated filesystem scans.

### Text SDF fixes (orientation, real-time updates, quality, performance, Add menu)

| File | Change |
|------|--------|
| `source/blender/draw/engines/sdf/shaders/sdf_text_blend_comp.glsl` | Text lies in XY plane facing up (like a Blender plane), Z is extrusion. UV samples `local_pos.xy`, depth along Z |
| `source/blender/blenkernel/intern/sdf_text.cc` | Bounds: `size[0]=half_w, size[1]=half_h, size[2]=depth`. Added sub-pixel edge refinement using BLF's anti-aliased alpha — replaces staircase binary threshold with smooth alpha-derived distances near glyph edges, blended with 8SSEDT via smoothstep for far-field accuracy |
| `source/blender/draw/engines/sdf/sdf_engine.cc` | Fixed real-time updates: text_hash now included in scene_hash so text parameter changes trigger `needs_bake_`. Added missing hash params. **Performance fix**: added `augment_indirection_for_text()` — CPU-side smart brick allocation that samples the 2D text SDF at each brick center, only activating bricks near actual glyph surfaces. TextBlendObject now stores CPU copy of 2D SDF data for this augmentation. Atlas always cleared before bake to initialize text bricks. |
| `source/blender/draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Text objects now skipped in both CSG passes — brick allocation moved to CPU-side augmentation for ~10x fewer active bricks |
| `scripts/startup/bl_ui/space_view3d.py` | Added "SDF Text" entry to `VIEW3D_MT_sdf_add` menu under separator |

**Performance notes:**
- Previously: classify used `max(sdBox, 0.0)` for text, allocating ALL interior bricks (100% of AABB). A single text object could create hundreds of active bricks.
- Now: CPU-side `augment_indirection_for_text()` uses bilinear-sampled 2D SDF at each brick center. Only bricks within `brick_half_diagonal` of actual glyph surfaces are activated (~5-15% of AABB). March time scales with visible text detail, not bounding box volume.

### Text SDF brick counter sync fix

**Bug**: Text objects were invisible (0 active bricks) because `dispatch_bake()` reads back the GPU `brick_counter` SSBO — which only reflects the classify shader's atomic count (excludes text bricks added by CPU-side augmentation) — and overwrites `active_brick_count_` with it. This caused `dispatch_text_blends()` to early-exit since `active_brick_count_ <= 0`.

| File | Change |
|------|--------|
| `source/blender/draw/engines/sdf/sdf_engine.cc` | After `augment_indirection_for_text()`, update GPU `brick_counter_` SSBO with the total count (classify + text bricks). Creates the SSBO if it doesn't exist yet (text-only scene without analytic objects). |

### Text SDF quality improvement — internal 2× supersampling

**Problem**: The 8SSEDT distance transform creates staircase artifacts at pixel boundaries because it operates on a binary threshold of the rasterized text. At the output resolution, these integer-pixel-precision distances create visible stepping on curved glyph surfaces when baked into the voxel atlas.

**Fix**: Rasterize and distance-transform at 2× the output resolution internally, then bilinear-downsample the SDF to the target atlas size. The 2× hi-res computation halves the staircase step size; bilinear downsampling averages out the remaining artifacts. The output atlas stays the same size — zero impact on GPU texture memory, text_blend performance, or march speed.

| File | Change |
|------|--------|
| `source/blender/blenkernel/intern/sdf_text.cc` | Removed 2× CPU supersampling (GPU bicubic + RGSS handles quality now). Direct rasterize → 8SSEDT → SDF at native resolution. 4× faster, 4× less memory. |
| `source/blender/makesdna/DNA_sdf_defaults.h` | Default `text_sdf_resolution` bumped from 256 to 512 (now safe — rendering performance is independent of atlas resolution after the classify fix) |
| `source/blender/makesrna/intern/rna_sdf.cc` | Max resolution raised from 1024 to 2048 |

### Rotated Text Normal Artifacts + Jagged Text Edges

**Problem**: Text SDF surfaces show a crosshatch/leather pattern when rotated and jagged staircase edges when blended with other primitives. Root cause: the 8SSEDT binary threshold creates pixel-level discontinuities in the 2D SDF that the DDA trilinear solver can't smooth over.

**Fix**: Two-layer approach:
1. **CPU Gaussian blur** (sigma=2px) applied to the 2D SDF after 8SSEDT — removes pixel-level staircases at the source, producing smooth distance gradients that the voxel DDA solver handles correctly.
2. **GPU bicubic (Catmull-Rom) sampling** — C1-continuous interpolation between the pre-filtered texels.
3. Removed GPU-side 4× RGSS (redundant now that CPU blur handles quality).

| File | Change |
|------|--------|
| `source/blender/blenkernel/intern/sdf_text.cc` | Added `gaussian_blur_separable()` — separable 2-pass Gaussian blur applied to 2D SDF before GPU upload |
| `source/blender/draw/engines/sdf/shaders/sdf_text_blend_comp.glsl` | Simplified to single bicubic sample (CPU blur handles anti-aliasing); kept `sampleTextSDF_bicubic()` for C1 smooth GPU interpolation |

### Real-Time Text Editing

**Problem**: Text SDF only updated when the text input field was confirmed (Enter/click away), not during typing.

**Fix**: Added `PROP_TEXTEDIT_UPDATE` flag to the `text` RNA string property. This makes Blender fire the `rna_SDF_update` callback on every keystroke, regenerating the 2D SDF atlas in real time as the user types.

| File | Change |
|------|--------|
| `source/blender/makesrna/intern/rna_sdf.cc` | Added `RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE)` to the `text` property |

---

## SDF Groups — Ordered Evaluation Containers

Added `SDFGroup` as a new ID type (`ID_SG`) that acts as an ordered container for SDF objects.
Each group evaluates its members sequentially (top-to-bottom), producing a single combined SDF.
Groups then combine with each other via group-level CSG operations.

### New Files (5)

| File | Purpose |
|------|---------|
| `makesdna/DNA_sdf_group_types.h` | `SDFGroup` + `SDFGroupMember` DNA structs |
| `makesdna/DNA_sdf_group_defaults.h` | Default values (union, smooth blend 0.1, white color) |
| `blenkernel/BKE_sdf_group.hh` | Public API: add, member_add/remove/move, reindex |
| `blenkernel/intern/sdf_group.cc` | IDTypeInfo (`IDType_ID_SG`) + API implementation |
| `makesrna/intern/rna_sdf_group.cc` | RNA property definitions for SDFGroup + SDFGroupMember |
| `scripts/startup/bl_ui/properties_data_sdf_group.py` | Properties panels for SDFGroup |

### Modified Files — DNA

| File | Change |
|------|--------|
| `DNA_ID_enums.h` | Added `ID_SG = MAKE_ID2('S', 'G')` |
| `DNA_ID.h` | Added `FILTER_ID_SG`, `INDEX_ID_SG`, updated `FILTER_ID_ALL` |
| `DNA_sdf_types.h` | Added `sdf_group` pointer and `group_order` to SDF struct |
| `source/blender/CMakeLists.txt` | Added `DNA_sdf_group_types.h` to `SRC_DNA_INC` |

### Modified Files — BKE

| File | Change |
|------|--------|
| `intern/idtype.cc` | `INIT_TYPE(ID_SG)` + both `CASE_IDINDEX(SG)` switches |
| `intern/main.cc` | `case ID_SG` in `which_libbase()` + `BKE_main_lists_get()` |
| `BKE_main.hh` | Added `ListBase sdf_groups = {}` to Main |
| `intern/sdf.cc` | Walk `sdf->sdf_group` in `sdf_foreach_id()` |
| `CMakeLists.txt` | Added `sdf_group.cc` and `BKE_sdf_group.hh` |

### Modified Files — RNA

| File | Change |
|------|--------|
| `rna_ID.cc` | `case ID_SG: return &RNA_SDFGroup;` |
| `rna_main.cc` | `bpy.data.sdf_groups` collection |
| `rna_main_api.cc` | `.new()` / `.remove()` / `.tag()` API |
| `makesrna.cc` | Registered `rna_sdf_group.cc` |
| `rna_sdf.cc` | Added `sdf_group` pointer + `group_order` on SDF struct |
| `rna_internal.hh` | Declarations for `RNA_def_sdf_group` and `RNA_def_main_sdf_groups` |
| `CMakeLists.txt` | Added `rna_sdf_group.cc` |

### Modified Files — Depsgraph

| File | Change |
|------|--------|
| `deg_builder_nodes.cc` | `case ID_SG:` in generic ID section |
| `deg_builder_relations.cc` | `case ID_SG:` in generic ID section |
| `depsgraph_tag.cc` | `case ID_SG: return NodeType::PARAMETERS;` |

### Modified Files — Editors

| File | Change |
|------|--------|
| `object/object_sdf.cc` | Auto-grouping on SDF add + 4 new operators: group_add, group_assign, group_remove_member, group_reorder |
| `object/object_intern.hh` | Declarations for 4 new operators |
| `object/object_ops.cc` | Registered 4 new operators |
| `space_buttons/buttons_context.cc` | `context.sdf_group` exposure |
| `space_outliner/outliner_draw.cc` | `case ID_SG:` icon |
| `space_outliner/outliner_select.cc` | `case ID_SG:` context panel |
| `space_outliner/outliner_tools.cc` | `case ID_SG:` standard ID |
| `space_outliner/tree/tree_element_id.cc` | `case ID_SG:` tree element |
| `interface/interface_icons.cc` | `case ID_SG:` in `UI_icon_from_id()` |

### Modified Files — Draw (Shader Pipeline)

| File | Change |
|------|--------|
| `sdf_shader_shared.hh` | Added `SDFGroupGPU` struct; added `group_id`, `group_first` to `SDFObjectGPU`; added `group_count` to params structs |
| `sdf_engine.h` | Added `sdf_groups_ssbo_get()` and `sdf_group_count_get()` public API |
| `sdf_engine.cc` | Group SSBO upload, binding in classify/bake dispatch, group building from `bmain->sdf_groups`, destructor cleanup, static getters |
| `shaders/infos/sdf_shader_infos.hh` | Added `groups[]` SSBO (slot 5) + `group_count` push constant to classify, bake, select_march, outline_march shaders |
| `shaders/sdf_bake_comp.glsl` | Replaced two-pass evaluation with sequential group-aware evaluation + ungrouped fallback |
| `shaders/sdf_classify_comp.glsl` | Same group-aware evaluation change |
| `shaders/sdf_select_march_frag.glsl` | Group-aware hit verification for CSG-modified surfaces |
| `shaders/sdf_outline_march_frag.glsl` | Same group-aware hit verification |

### Modified Files — Overlay

| File | Change |
|------|--------|
| `overlay_sdf.hh` | Bind group SSBO + set `group_count` uniform for selection shader |
| `overlay_outline.hh` | Bind group SSBO + set `group_count` uniform for outline shader |

### Modified Files — Python UI

| File | Change |
|------|--------|
| `bl_ui/__init__.py` | Registered `properties_data_sdf_group` module |
| `bl_ui/properties_data_sdf.py` | Removed `DATA_PT_sdf_group` panel — group settings only shown when selecting group root in outliner |
| `bl_ui/space_view3d.py` | Added "SDF Group" to Add > SDF menu |

### Modified Files — Outliner (Selection & Display)

| File | Change |
|------|--------|
| `outliner_select.cc` | Added `ID_SG` selection handling: clicking SDFGroup selects all member objects and activates first member |
| `outliner_draw.cc` | SDF group order number prefix in outliner names (e.g. "1. Cube"), `ICON_SDF_GROUP` for group entries |
| `outliner_intern.hh` | Added `ID_SG` to `TREESTORE_ID_TYPE` macro (required for group visibility in outliner) |
| `tree/tree_display_view_layer.cc` | Added `add_sdf_groups()` method to build SDF group tree entries with member children |
| `tree/tree_display.hh` | Declared `add_sdf_groups()` private method |

### New Files — Icons

| File | Change |
|------|--------|
| `release/datafiles/icons_svg/sdf_group.svg` | Custom SVG icon for SDF groups (overlapping rounded squares) |

### Bug Fixes — Group Lifecycle & Cleanup

| File | Change |
|------|--------|
| `blenkernel/intern/sdf.cc` | Changed `sdf_group` pointer walk from `IDWALK_CB_NOP` to `IDWALK_CB_USER` — fixes groups being orphaned during undo/user-count recalculation |
| `blenkernel/intern/sdf_group.cc` | Added `id_us_plus`/`id_us_min` for group reference when setting/clearing SDF back-pointer; `sdf_group_foreach_id` now removes NULL members (deleted SDFs) and reindexes survivors |
| `editors/space_buttons/buttons_context.cc` | Added SDFGroup to context path via SDF back-pointer, making group panels visible in Properties editor |

### Group Transform Feature

| File | Change |
|------|--------|
| `makesdna/DNA_sdf_group_types.h` | Added `loc[3]`, `rot[3]`, `scale[3]` fields to SDFGroup struct |
| `makesdna/DNA_sdf_group_defaults.h` | Added defaults: loc=0, rot=0, scale=1 |
| `makesrna/intern/rna_sdf_group.cc` | Added `location`, `rotation`, `scale` RNA properties |
| `draw/engines/sdf/sdf_engine.cc` | Group transform composed with member object matrices via `loc_eul_size_to_mat4` in `object_sync`; added `BLI_math_matrix.h` include |
| `bl_ui/properties_data_sdf_group.py` | Added `DATA_PT_sdf_group_transform` panel with location/rotation/scale fields |

### Outliner Cleanup — SDF Objects Hidden from Scene Collection

| File | Change |
|------|--------|
| `tree/tree_display_view_layer.cc` | SDF objects (`OB_SDF`) filtered from `add_layer_collection_objects` and flat view — they only appear under SDF Groups; NULL members cleaned up in `add_sdf_groups` before tree build |

### Properties Editor — Exclusive Group/SDF Panel Display

| File | Change |
|------|--------|
| `editors/space_buttons/buttons_context.cc` | Added `RNA_SDFGroup` to data path checks (with `type == -1` guard to prevent bone tab showing); removed SDFGroup-alongside-SDF path addition; added `"sdf_group"` to context directory |
| `editors/space_outliner/outliner_select.cc` | Added `outliner_sdf_group_update_pin()` helper — pins SDFGroup in Properties editors when group root is clicked, clears pin when anything else is clicked. SDF Group click → only group panels; SDF member click → only SDF panels |

### Bug Fixes — Duplicate, Delete, Reorder, Bone Tab

| File | Change |
|------|--------|
| `editors/object/object_add.cc` | Added `object_add_sync_sdf_group()` — duplicated SDF objects auto-added to same group as original; includes for `DNA_sdf_group_types.h`, `DNA_sdf_types.h`, `BKE_sdf_group.hh` |
| `editors/object/object_sdf.cc` | Reorder and remove operators now use `member_index` parameter instead of active object; find group from pinned SDFGroup or active object; removed `ED_operator_objectmode` poll to work from Properties panel; added `DNA_space_types.h` include |
| `scripts/startup/bl_ui/properties_data_sdf_group.py` | Reorder and remove buttons now pass `member_index = idx` to operators |
| `blenkernel/BKE_sdf_group.hh` | Added `BKE_sdf_group_cleanup_null_members()` declaration |
| `blenkernel/intern/sdf_group.cc` | Added `BKE_sdf_group_cleanup_null_members()` — removes members with NULL object pointers and reindexes |
| `editors/space_outliner/tree/tree_display_view_layer.cc` | Replaced inline NULL member cleanup with `BKE_sdf_group_cleanup_null_members()` call |

### Outliner Order Display Fix & Reorder from Outliner

| File | Change |
|------|--------|
| `editors/space_outliner/outliner_draw.cc` | Order numbers now derived from tree position (sibling index) instead of stale `sdf->group_order` field — checks parent is ID_SG to show "N. Name" prefix |
| `scripts/startup/bl_ui/space_outliner.py` | Added "Move Up in Group" / "Move Down in Group" to OUTLINER_MT_object right-click menu for SDF objects in groups |

### Outliner Reorder Arrow Buttons

| File | Change |
|------|--------|
| `editors/space_outliner/outliner_draw.cc` | Added `outliner_draw_sdf_reorder_buts()` — draws up/down arrow buttons for SDF members under SDF Groups in the outliner; called from `draw_outliner()` after restrict buttons |

### SDF Group Origin Viewport Overlay

| File | Change |
|------|--------|
| `draw/engines/overlay/overlay_sdf_group_origins.hh` | **New.** `SdfGroupOrigins` overlay class — draws plain axes (cross) markers at each SDFGroup's `loc` position using the `extra_shape` shader. Iterates `bmain->sdf_groups` in `end_sync()` (groups are not Objects, so no `object_sync`). Uses group's `color` field for marker color |
| `draw/engines/overlay/overlay_instance.hh` | Added `#include "overlay_sdf_group_origins.hh"`, `SdfGroupOrigins sdf_group_origins` member at Instance level (alongside Origins) |
| `draw/engines/overlay/overlay_instance.cc` | Added `sdf_group_origins` calls in `begin_sync()`, `end_sync()`, and `draw_v3d()` (after layer draw_line calls) |

### Bug Fixes — Arrow Overlap, Deleted SDF Display, Pin Clearing

| File | Change |
|------|--------|
| `editors/space_outliner/outliner_draw.cc` | Arrow button positions now offset by `outliner_right_columns_width()` to avoid overlapping restrict column icons |
| `editors/space_outliner/tree/tree_display_view_layer.cc` | Skip creating tree elements for group members whose object has no Base in the view layer (prevents grayed-out display of deleted SDFs) |
| `editors/space_view3d/view3d_select.cc` | Clear SDFGroup pin from Properties editors on viewport click — ensures Properties shows SDF data panels after viewport selection instead of stale group panels |

### Removed Group Transform & Visual Overlay

Group transform (loc/rot/scale) and viewport origin overlay removed — groups are purely organizational containers; members are moved individually.

| File | Change |
|------|--------|
| `makesdna/DNA_sdf_group_types.h` | Removed `loc[3]`, `rot[3]`, `scale[3]` and padding fields from SDFGroup struct |
| `makesdna/DNA_sdf_group_defaults.h` | Removed `.loc`, `.rot`, `.scale` defaults |
| `makesrna/intern/rna_sdf_group.cc` | Removed `location`, `rotation`, `scale` RNA property definitions |
| `draw/engines/sdf/sdf_engine.cc` | Removed group transform composition (`loc_eul_size_to_mat4` block in `object_sync`); removed `BLI_math_matrix.h` include |
| `bl_ui/properties_data_sdf_group.py` | Removed `DATA_PT_sdf_group_transform` panel |
| `draw/engines/overlay/overlay_sdf_group_origins.hh` | **Deleted.** Viewport overlay for group origin markers no longer needed |
| `draw/engines/overlay/overlay_instance.hh` | Removed `SdfGroupOrigins` include and member |
| `draw/engines/overlay/overlay_instance.cc` | Removed `sdf_group_origins` calls from `begin_sync()`, `end_sync()`, `draw_v3d()` |

### Fixed Arrow Buttons — Group Name Passing

Outliner reorder/remove buttons now work by passing `group_name` directly to operators instead of relying on Properties editor context (which is unavailable in outliner).

| File | Change |
|------|--------|
| `editors/object/object_sdf.cc` | Added `sdf_group_from_operator()` helper that resolves SDFGroup from: (1) `group_name` operator property, (2) pinned ID, (3) active object; added `group_name` string property to both reorder and remove operators; added `BKE_lib_id.hh` include |
| `editors/space_outliner/outliner_draw.cc` | Arrow buttons now set `group_name` property on operator via `RNA_string_set()` |
| `bl_ui/properties_data_sdf_group.py` | Python panel buttons also pass `group_name` for consistency |

### Improved SDF Group Icon

| File | Change |
|------|--------|
| `release/datafiles/icons_svg/sdf_group.svg` | Redesigned icon: overlapping cube, sphere ring, and cone silhouettes (matches reference image of grouped 3D objects) |

### Outliner Sort Fix — Preserve SDF Group Member Order

| File | Change |
|------|--------|
| `editors/space_outliner/outliner_tree.cc` | Both `outliner_sort()` and `outliner_collections_children_sort()` now early-return when `lb` is a subtree whose parent is an `ID_SG` element — checked via `first_te->parent`. Previous approach (guarding only the recursive descent) was insufficient because the sort fires on the list **before** recursing into children. The early-return prevents any qsort from touching SDF group member order while still recursing into members' own subtrees |

### CSG/Blend Controls Moved to Outliner

Removed per-SDF Operation panel from Properties. CSG operation and blend type are now controlled via icon buttons in the outliner for each SDF group member.

| File | Change |
|------|--------|
| `bl_ui/properties_data_sdf.py` | Removed `DATA_PT_sdf_operation` panel class and registration |
| `editors/object/object_sdf.cc` | Added `OBJECT_OT_sdf_set_csg` and `OBJECT_OT_sdf_set_blend` operators for setting CSG/blend on SDF objects by name |
| `editors/object/object_intern.hh` | Declared new operators |
| `editors/object/object_ops.cc` | Registered new operators |
| `editors/space_outliner/outliner_draw.cc` | Added `sdf_csg_icon()`, `sdf_blend_icon()`, `sdf_csg_next()`, `sdf_blend_next()` helpers; CSG and blend icon buttons drawn for each SDF group member (skipped for first member which acts as base); reorder arrows moved to columns 3-4 to make room |

### Reorder Button Fix — ExecRegionWin + Notifiers

| File | Change |
|------|--------|
| `editors/space_outliner/outliner_draw.cc` | Changed reorder button `OpCallContext` from `ExecDefault` to `ExecRegionWin` |
| `editors/object/object_sdf.cc` | Added `NC_SCENE \| ND_OB_ACTIVE` notifier to reorder and remove operators for outliner rebuild |

---

## Async Shader Precompilation

### Problem

SDF shaders were compiled lazily on first draw via `ensure_shaders()` — 6 synchronous calls to `GPU_shader_create_from_info_name()`. Despite `DO_STATIC_COMPILATION()` flags, Blender only uses those for a debug validation tool (`--debug-gpu-compile-shaders`), not for startup warmup. First frame with SDF objects would stall while all shaders compiled sequentially.

### Solution

Shader compilation now starts asynchronously in `init()` using `GPU_shader_batch_create_from_infos()`, which dispatches compilation to a background subprocess. By the time `draw()` calls `ensure_shaders()`, binaries are typically ready. On subsequent launches, the OpenGL binary cache (`gl-shader-cache/`) provides near-instant loads.

### Architecture

- **Table-driven**: Shader names and pointers managed via `ShaderIndex` enum + `shader_info_names_[]` array. Adding a shader = one enum value + one string.
- **Reference aliases**: `classify_sh_`, `bake_sh_`, etc. are C++ references into `shaders_[SH_COUNT]`, so all existing code works unchanged.
- **Lifecycle**: Destructor cancels in-flight batches and frees all shaders (fixes pre-existing leak).

### Modified Files (Draw)

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Replaced 6 raw `gpu::Shader*` members with `ShaderIndex` enum, `shader_info_names_[]` table, `shaders_[SH_COUNT]` array, and reference aliases. Added `BatchHandle shader_compile_batch_` and `bool shaders_compiled_` members. `init()` kicks off `GPU_shader_batch_create_from_infos()` on first call. `ensure_shaders()` finalizes the batch (blocking only if subprocess not done) with synchronous fallback. Destructor cancels in-flight batch and frees all shaders via loop |

---

## SDF Outline — Instanced AABB Rasterization (Performance)

Replaced the fullscreen ray-march outline shader with **instanced AABB box rasterization**. The old approach drew a single fullscreen triangle where each fragment looped over ALL SDF objects (O(pixels × objects × march_steps)). With 1000 selected objects at 1080p, this was ~96 billion SDF evaluations per frame.

The new approach draws one instanced box per selected SDF object. Each fragment evaluates exactly **one** object's SDF, and the GPU depth test resolves overlapping surfaces automatically. This is O(covered_pixels × march_steps) — independent of total object count.

### New Files

| File | Purpose |
|------|---------|
| `draw/engines/sdf/shaders/sdf_outline_march_vert.glsl` | Vertex shader: procedural unit cube (36 verts) transformed to each object's world-space AABB via `gl_InstanceID` → `selected_indices[]` → SSBO lookup. Passes flat `obj_index` to fragment shader |

### Modified Files

| File | Change |
|------|---------|
| `draw/engines/sdf/shaders/sdf_outline_march_frag.glsl` | Removed per-pixel object loop. Fragment shader now evaluates a single object identified by flat `obj_index` input. Uses `gl_FragCoord` + `viewport_size_inv` for ray reconstruction instead of `screen_uv` |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `sdf_outline_inst_iface` (flat int obj_index). Shader info now uses own vertex/fragment sources instead of `gpu_fullscreen`. Added `selected_indices[]` SSBO (slot 2) and `viewport_size_inv` push constant. Removed `object_count` push constant |
| `draw/engines/overlay/overlay_outline.hh` | Builds `sdf_selected_indices_` compact list during `sdf_object_sync()`. `draw_sdf_outline_()` uploads selected indices SSBO, sets front-face culling (renders back faces for camera-inside AABB correctness), and draws instanced boxes via `GPU_batch_draw_advanced()` instead of fullscreen triangle |
| `draw/CMakeLists.txt` | Registered `sdf_outline_march_vert.glsl` |

---

## SDF Engine Performance Optimization Pass

Comprehensive performance optimization targeting GPU→CPU sync stalls, ray march efficiency, bake pipeline, and CPU-side overhead.

### New Files

| File | Purpose |
|------|---------|
| `draw/engines/sdf/shaders/sdf_augment_grids_comp.glsl` | GPU compute shader replacing CPU-side `augment_indirection_for_grids()`. One thread per brick in grid AABB, atomically allocates slots for inactive bricks, eliminates massive indirection texture + active_bricks SSBO readbacks |

### Modified Files

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | **A2**: Removed `GPU_flush()` + `GPU_finish()` from `perf_end_pass()` — wall-clock timing only, no GPU pipeline drain per pass |
| `draw/engines/sdf/sdf_engine.cc` | **B1**: Re-enabled scissor rect in `draw_march()` — projects atlas AABB to screen space via view-projection matrix, applies GPU_scissor with 16px padding. Skips when any AABB corner is behind camera. Reduces fragment invocations by up to 90%+ for small SDF scenes |
| `draw/engines/sdf/sdf_engine.cc` | **B4**: Dynamic `normal_quality` — uses fast normals (8 texelFetch) during bake frames (interaction), smooth dual-voxel normals (27 texelFetch) when idle |
| `draw/engines/sdf/sdf_engine.cc` | **A1**: Replaced CPU `augment_indirection_for_grids()` with GPU compute dispatch. Old code read entire 3D indirection texture (up to 8MB at 128³) + full active_bricks SSBO to CPU, modified, re-uploaded. New code runs compute shader on GPU, only reads back 16-byte BrickCounter for active_brick_count |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `sdf_augment_grids` shader create info (4x4x4 workgroup, indirection image, brick_counter + active_bricks SSBOs, grid AABB push constants) |
| `draw/CMakeLists.txt` | Registered `sdf_augment_grids_comp.glsl` |
| `draw/engines/sdf/sdf_engine.cc` | **C1**: Optimized `dispatch_grid_blends()` — hoisted invariant push constants, SSBO/image binds, and binding slot lookups outside per-grid loop. Only per-grid-varying uniforms (transform, color, blend params) set inside loop |
| `draw/engines/sdf/sdf_engine.cc` | **C3**: BVH caching — added `spatial_hash_` (positions + sizes only) and `needs_bvh_rebuild_` flag. SAH BVH construction skipped when only non-spatial properties change (color, blend, bevel). Added `update_bvh_aabbs()` for bottom-up AABB refresh without topology rebuild |
| `draw/engines/sdf/sdf_engine.cc` | **D1**: Scene hash optimization — replaced O(N×25) redundant per-object hash with reuse of existing `compute_object_hash()` results + `group_struct_hash`. Cut scene hash from ~70 multiplies/object to 1 combine/object |
| `draw/engines/sdf/sdf_engine.cc` | Added `SH_AUGMENT_GRIDS` to shader enum, `augment_grids_sh_` reference alias |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | **B8**: Reordered brick-level DDA to check indirection slot BEFORE computing `t_brick_exit` — avoids unnecessary min3 computation for empty bricks |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | **B2**: Added interior brick early termination — `slot == -2` bricks (fully inside surface) return immediate hit at entry point instead of being skipped. Handled in main() with default color and camera-facing normal |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | **Per-voxel AABB culling**: Before evaluating SDF (matrix multiply + primitive + modifiers), checks if the voxel world position is inside the object's expanded AABB (read from SSBO, coalesced). Skips the entire SDF eval when outside. For 100+ blended objects, reduces per-voxel evals from ~100 to ~5-20 (only geometrically nearby objects). Group-first objects set grp_dist=1e10 instead of evaluating |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | **Per-center AABB culling**: Before evaluating SDF at brick center, checks if brick_center is inside the object's expanded AABB. Tighter than the brick-level BVH culling (point vs box). Same group-first handling as bake |

---

## Per-Brick Dirty Tracking + Shared Memory AABB Cache

Eliminates redundant bake work during incremental updates. Previously, moving one large SDF object with 100+ small neighbors caused either a full rebake (dirty region > 50% of grid) or rebaked every brick in the dirty AABB even if only one object affected it. Now bricks are skipped entirely when none of their candidate objects changed.

### Changes

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_engine.cc` | Added `dirty_flags_` vector + `dirty_flags_ssbo_` SSBO (per-object 0/1). Computed alongside dirty_indices in incremental detection. Uploaded and bound to classify/bake shaders at slot 6. Removed 50% dirty-volume threshold — per-brick dirty checks make large dirty regions efficient. Dirty objects' AABBs expanded to cover old+new positions so BVH routes bricks at old position to the moved object. Non-expanded AABBs saved for next-frame comparison |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `STORAGE_BUF(6, read, int, dirty_flags[])` and `PUSH_CONSTANT(int, has_dirty_flags)` to both `sdf_classify` and `sdf_bake` shader infos |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | After BVH candidate collection, checks if any candidate has `dirty_flags != 0`. If none dirty and in incremental mode, returns early — keeps existing indirection slot unchanged, brick not added to active_bricks list |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Thread 0 checks dirty flags after candidate collection, stores result in `shared_any_dirty`. All 144 threads skip the brick if no candidate is dirty. Added `shared_bb_min/shared_bb_max[MAX_CANDIDATES]` arrays — per-candidate bounding boxes loaded cooperatively into shared memory during candidate load phase, replacing per-voxel global SSBO reads (saves 2 × candidates × 1728 global memory reads per brick) |

---

## Lipschitz Pruning — Per-Brick Candidate Pre-Cull

Implements the core pruning algorithm from "Lipschitz Pruning: Hierarchical Simplification of Primitive-Based SDFs" (Barbier et al., Eurographics 2025). For a brick (spatial region with radius R), evaluates all candidates at the brick center and prunes those provably outside the blend influence radius using the 1-Lipschitz property of SDFs: if |f₁(p) - f₂(p)| ≥ k + 2R, the operator reduces to one operand for all points in the region.

### Pruning Rules (per CSG operation)

| CSG Op | Prune candidate when |
|--------|---------------------|
| UNION | `d - acc > k + 2R` (candidate too far from accumulated surface) |
| SUBTRACT | `acc + d > k + 2R` (subtraction target too far) |
| INTERSECT | `acc - d > k + 2R` (intersection constraint too far) |

### Changes

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | After BVH candidate collection and dirty check, thread 0 evaluates ALL candidates' SDF at the brick center (matrix multiply + primitive eval per candidate). Then simulates the sequential CSG evaluation at center, applying Lipschitz pruning: for each candidate, checks if \|accumulated - distance\| exceeds the blend radius k plus twice the brick circumradius 2R. Pruned candidates are removed from the candidate list before the cooperative shared memory load. Handles groups (per-group accumulator, then inter-group combination) and ungrouped objects. Group-first objects are never pruned. This is especially effective for high-smin scenes where AABB-based culling fails (AABBs expanded by k become huge) but Lipschitz pruning uses actual evaluated distances |

### Expected Impact

For a scene with 100+ small objects smooth-unioned (k=2.0) with a large SDF:
- Brick circumradius R ≈ 0.054 (at voxel_size=1/128), so threshold = k + 2R ≈ 2.108
- Objects with center distance > 2.108 from the accumulated surface are pruned
- Typically reduces ~100 candidates to ~5-15 active per brick near the surface
- Each pruned candidate saves 1728 SDF evaluations (12³ voxels per brick)
- Cost: ~100 center evaluations by thread 0 (cheap vs 1728×N_pruned saved across 144 threads)

---

## Streaming Bake & Grid Stability Overhaul

Replaced the hard-limited bake pipeline (MAX_CANDIDATES=128) with a streaming
architecture that has no hard limits on candidate count, and fixed several grid
management issues that caused freezes and permanent quality degradation.

### Problems Solved

1. **Blending artifacts at brick boundaries** — With 100+ SDFs blending, the
   MAX_CANDIDATES=128 hard cap silently dropped objects. Different bricks dropped
   different objects, creating rectangular block-shaped discontinuities.
2. **Lipschitz pruning overhead** — The pruning step itself (cooperative center eval +
   thread-0 sequential pruning) was expensive and only needed because of the hard cap.
3. **Grid resize freezes** — Moving objects outside hysteresis padding caused full
   rebakes (200ms+ frames).
4. **Quality permanently degrades** — After moving objects far away and back, voxel
   resolution stayed coarse due to symmetric deadband blocking recovery.

### Bake Shader — Streaming Evaluation (`sdf_bake_comp.glsl`)

Complete rewrite. Instead of caching all candidates in shared memory (requiring a
hard limit), candidates are streamed through shared memory in batches of 64:

- **Z-outer loop**: Each voxel layer is fully evaluated before the next. Per-voxel
  accumulators (acc_dist, grp_dist, etc.) are registers that persist across batches.
- **Batch-inner loop**: Each batch cooperatively loads 64 candidates into shared
  memory, all threads evaluate against them, then the next batch overwrites.
- **Single-pass group evaluation**: Candidates are already sorted by (group_id,
  group_order). Group transitions are tracked per-voxel via `prev_group` register.
  No separate per-group loop — O(candidates) instead of O(groups × candidates).
- **Overflow fallback**: BVH candidate indices are collected into a 2048-slot shared
  buffer. If exceeded, the shader falls back to iterating all objects in sorted order
  (no BVH filtering). Per-voxel AABB culling still skips non-overlapping objects.
- **No Lipschitz pruning**: Removed entirely. With no candidate limit, there's nothing
  to prune down to.

| Old | New |
|-----|-----|
| MAX_CANDIDATES = 128 (hard cap, causes artifacts) | No limit (streaming) |
| MAX_COLLECT = 1024 (silent drop) | CANDIDATE_BUF_SIZE = 2048 + overflow fallback |
| Lipschitz pruning (2 barriers + thread-0 sequential) | Removed |
| Candidate-outer, Z-inner (needs all candidates cached) | Z-outer, batch-inner (only BATCH_SIZE cached) |
| Two-pass group evaluation: per-group + ungrouped | Single-pass ordered evaluation |

### Classify Shader — Linear Scan (`sdf_classify_comp.glsl`)

Rewritten to iterate all objects in sorted order with AABB culling. No candidate
buffer, no BVH dependency, zero hard limits.

- Single-pass group-aware evaluation (same pattern as bake shader)
- Group transitions tracked before AABB cull so group boundaries are correct
  even when the base shape is culled for a brick
- Dirty check as a separate lightweight pass (AABB test + dirty flag only)

### Grid Stability (`sdf_engine.cc`)

- **Asymmetric deadband**: Refinement passes through at > 10% improvement (ratio < 0.9),
  coarsening suppressed up to 25% (ratio < 1.25). Previously symmetric ±20% blocked
  quality recovery after temporary scene expansion.
- **Proportional hysteresis padding**: `max(chunk_size * 8, scene_extent * 0.25, 1.0)`
  instead of `max(chunk_size * 8, 1.0)`. Larger scenes get proportionally more movement
  room before triggering grid expansion.
- **Aggressive contraction**: Grid contracts when ideal size < 50% of current per axis
  (was 33%). Reclaims quality faster when the scene shrinks.

### Changes

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Complete rewrite: streaming batch evaluation (BATCH_SIZE=64), overflow fallback, single-pass group tracking, removed MAX_CANDIDATES/Lipschitz |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Rewrite: linear scan through sorted objects, no candidate buffer, single-pass group evaluation |
| `draw/engines/sdf/sdf_engine.cc` | Asymmetric voxel deadband (0.9–1.25), proportional hysteresis padding, 50% contraction threshold |

---

## Chunk-Hashed World Brickmap

Made `sdf_resolution` a fixed world-space voxel density instead of a scene-bounded
atlas resolution. The renderer now builds a sparse chunk directory on the CPU,
stores dense brick-slot tables per chunk on the GPU, and marches that structure
through a chunk hash instead of a single scene-sized indirection texture.

### Why

1. **Scene-size independence** — `128` now means `1 / 128` Blender units per voxel everywhere.
2. **Sparse world coverage** — distant objects no longer force one dense atlas AABB over the empty gap.
3. **Remove dead instanced path** — the unused per-object brickmap march path was removed from the shader interfaces.

### Changes

| File | Change |
|------|--------|
| `draw/engines/sdf/sdf_private.hh` | Replaced the old global-grid constants with chunked world-brick constants: `SDF_CHUNK_BRICK_RES` and `SDF_CHUNK_BRICK_COUNT` |
| `draw/engines/sdf/sdf_shader_shared.hh` | Added `ChunkPageGPU` and `ChunkHashEntryGPU` for sparse world chunk lookup. Removed the unused per-object instancing structs from the march path |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Rewired shader bindings away from the scene `indirection_tx` texture and onto chunk page/hash/brick SSBOs. Removed unused instanced march bindings and dirty-flag bindings from the world path |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Rewritten to classify dense local bricks inside sparse world chunks, output global brick coordinates, and populate chunk-local brick-slot tables |
| `draw/engines/sdf/shaders/sdf_augment_grids_comp.glsl` | Rewritten to mark chunk-local bricks overlapping dense grid-object bounds active without a 3D indirection texture |
| `draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Switched bake positions and brick AABBs from atlas-relative coordinates to global world brick coordinates. Intersection objects now bypass BVH/AABB culling instead of using infinite bounds |
| `draw/engines/sdf/shaders/sdf_grid_blend_comp.glsl` | Switched grid blending to the same world brick coordinates as the new chunked bake path |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Removed the per-object instanced path and replaced world marching with chunk-hash lookup + chunk DDA + local brick DDA + voxel DDA |
| `draw/engines/sdf/sdf_engine.cc` | Replaced the scene-bounded indirection texture pipeline with CPU-built sparse chunk pages, uploaded chunk hash/brick-slot buffers, active-brick SSBO readback for debug grid, fixed-density voxel sizing, and grid/object chunk coverage building |
| `makesrna/intern/rna_space.cc` | Updated `sdf_resolution` UI text to describe fixed world-space voxel density instead of total atlas resolution |

### Follow-up Fixes

| File | Change |
|------|--------|
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Restored `sdf_modifiers` SSBO bindings for `sdf_grid_blend` and `sdf_march` because `sdf_lib.glsl` still references the global modifier buffer |
| `draw/engines/sdf/sdf_engine.cc` | Rebound `modifier_ssbo_` in the grid-blend and march dispatch paths to match the restored shader interfaces |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Renamed local `flat` index variable to `flat_index` to avoid GLSL keyword conflicts on OpenGL drivers |
| `draw/engines/sdf/shaders/sdf_augment_grids_comp.glsl` | Renamed local `flat` index variable to `flat_index` to avoid GLSL keyword conflicts on OpenGL drivers |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Renamed local `flat` index variable to `flat_index` to avoid GLSL keyword conflicts on OpenGL drivers |
| `draw/engines/sdf/sdf_engine.cc` | Fixed the CPU chunk hash to match the GLSL chunk hash used by `sdf_march_frag.glsl`. The previous mismatch caused march lookups to miss baked chunks, which manifested as partially rendered SDFs and disappearing pieces when objects moved |
| `draw/engines/sdf/sdf_engine.cc` | Fixed the 3D voxel debug overlay to read back the full `active_bricks_` SSBO allocation instead of a smaller `active_brick_count_` vector. The old code could overwrite CPU memory whenever capacity exceeded active count, causing freezes/crashes in `tbbmalloc.dll` while moving or deleting SDFs with debug view enabled. Also added a safety cap so the CPU wireframe overlay is skipped for very large brick sets |
| `blenkernel/intern/sdf.cc` | Fixed SDF group lifetime handling: duplicated SDF datablocks now clear copied `sdf_group` back-pointers and `group_order`, and `sdf_free_data()` no longer manually decrements the group ID user count. This avoids stale copied group links and removes a nonstandard free-time decrement that could desync group users |
| `blenkernel/intern/sdf_group.cc` | Hardened group membership updates: `BKE_sdf_group_member_add()` now avoids duplicate entries and removes the object from its previous group before re-adding, while `BKE_sdf_group_member_remove()` only decrements group users when the count is actually positive. This fixes `ID user decrement error: SGSDF Group ... 0 <= 0` during delete/remap paths |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added chunk page + chunk BVH SSBO bindings to the march shader so traversal can work directly on occupied chunk nodes instead of only a hash table |
| `draw/engines/sdf/sdf_engine.cc` | Added CPU-side chunk BVH construction/upload alongside the sparse chunk directory, then refined it to rebuild from the actual active surface-brick list after classify. The march pass no longer traverses every candidate chunk in object AABBs; it now traverses only chunks that contain active surface bricks, making baked rendering respond correctly to voxel density changes while reducing empty-space cost |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Replaced the top-level chunk DDA over the whole world AABB with exact per-chunk local DDA driven by an occupied-chunk BVH. This keeps the exact brick/voxel hit logic but reduces baked-render dependence on voxel density by traversing only active occupied chunks instead of every possible chunk cell along the ray |
| `draw/engines/sdf/sdf_engine.cc` | Fixed bake-time slowness and incorrect voxel-density response by moving `build_chunk_pages()` until after the final blend-expanded object AABBs are computed, and by generating candidate chunk pages as a surface shell instead of the full object AABB volume. Adding an SDF no longer classifies entire solid interiors just to discover that only the boundary contains active surface bricks |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Added a compact-scene fallback path that uses direct chunk DDA through the chunk hash when the atlas chunk span is small, while keeping active-chunk BVH traversal for large sparse spans. This avoids the "single SDF freezes Blender" case where BVH traversal overhead dominated tiny scenes, while preserving the sparse-scene acceleration path |
| `draw/engines/sdf/sdf_engine.cc` | Added console diagnostics for each bake frame: resolution, voxel size, object/grid counts, candidate chunk pages, active bricks, chosen march mode (hash vs BVH), chunk grid size, BVH node count, atlas dimensions, and total bake time. Also removed the extra global blend/shell padding from chunk-page generation, since chunk pages are now built after final blend-expanded AABBs and no longer need to be re-expanded by scene-global maxima |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Added and then reverted a conservative hybrid world-march prepass. Even with safety margins, using sampled baked distances for pre-advancement still introduced visible artifacts, so the shader now uses the exact hash/BVH DDA path again for correctness while a non-approximate acceleration is designed |
| `draw/engines/sdf/sdf_engine.cc` | Fixed another SSBO readback overflow: `rebuild_render_chunk_bvh()` now allocates CPU storage for the full `active_bricks_` allocation before `GPU_storagebuf_read()`. The previous code only allocated `active_brick_count_` entries even though Blender reads the full SSBO allocation, which could corrupt CPU memory and crash in `tbbmalloc.dll` when interacting with SDFs |
| `draw/engines/sdf/sdf_engine.cc` | Reverted chunk-page shell culling back to conservative full candidate coverage. The shell optimization was not conservative for rotated and blended shapes and could drop chunks that still contained real surface, causing obvious rendering artifacts. Candidate chunk generation now favors correctness again |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Increased exact DDA budgets (`MAX_BRICK_STEPS`, `MAX_VOXEL_STEPS`) to reduce residual misses on long diagonal traversals through chunks/bricks. This trades a bit of worst-case march cost for fewer visible artifacts along exact-path edge cases |
| `draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Fixed interior-brick classification to be conservative: chunks are now marked `-2` (fully interior) only when the center distance is more negative than the brick half-diagonal. Previously any negative center sample marked the brick interior, which could cause premature hits and severe artifacts on rotated surfaces or near blended boundaries |
| `draw/engines/sdf/sdf_engine.cc` | Expanded the SDF perf overlay text with chunk count, chunk BVH node count, current march mode, chunk grid resolution, compact atlas dimension/estimated MB, and SSBO memory estimates. This makes it easier to see whether slow frames come from candidate chunk explosion, atlas growth, or march-path choice |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Disabled the compact-scene top-level hash traversal for correctness and now always prefers the occupied-chunk BVH path when chunk BVH data exists. The previous hash path reconstructed chunk coordinates from floating-point atlas bounds for small scenes, which could misaddress chunks and contribute to rendering artifacts |
| `draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | Added `chunk_grid_resolution` and `atlas_chunk_min` push constants for the SDF march shader so compact-scene traversal can use exact integer chunk coordinates from the CPU |
| `draw/engines/sdf/sdf_engine.cc` | Stored `atlas_chunk_min_` from chunk-page generation and passed both exact chunk-grid size and chunk-grid origin to the march shader |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Restored the compact-scene exact hash traversal, but now using integer `atlas_chunk_min` and `chunk_grid_resolution` from the CPU instead of reconstructing chunk origin/grid size from floats. This keeps the fast path for small scenes while avoiding the float-rounding chunk misaddressing that contributed to artifacts |
| `draw/engines/sdf/shaders/sdf_march_frag.glsl` | Fixed occupied-chunk BVH traversal to keep searching for the nearest hit instead of returning on the first leaf hit. The old behavior caused rendering artifacts with multiple SDFs and rotated SDFs that spanned several chunks because an earlier-tested leaf could hide a closer actual hit in another intersected chunk |
| `draw/engines/overlay/overlay_sdf.hh` | Remapped select IDs from depsgraph order into the SDF engine's sorted `sdf_objects[]` order before uploading the picking SSBO. This fixes broken SDF picking when the engine reorders objects for grouped CSG evaluation |
| `editors/space_outliner/tree/tree_display_view_layer.cc` | Restored SDF objects to the normal view-layer and collection hierarchy in the native Outliner instead of filtering them out. SDFs now appear in the standard Blender object hierarchy while the separate SDF Group section remains available |
