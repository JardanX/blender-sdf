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

---

## Bugs Found & Fixed

1. **Startup crash** — `BKE_main_lists_get()` didn't map `INDEX_ID_SF` to `bmain.sdfs`. The ListBase array had a NULL slot, crashing during `Main::~Main`.
2. **Add SDF crash** — `BKE_idtype_idcode_to_index()` was missing `CASE_IDINDEX(SF)`. ID_SF lookups returned -1, so `BKE_libblock_alloc_notest` returned NULL.
3. **Build error** — `DNA_sdf_types.h` wasn't in `SRC_DNA_INC`, so `makesdna` didn't generate `_SDNA_TYPE_SDF`.
4. **Build error** — `rna_main_api.cc` used `SDF` type without `#include "DNA_sdf_types.h"`.

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
| `blenkernel/intern/object.cc` | Removed 7 `OB_MBALL` cases + 2 `ID_MB` edit mode cases. Kept `ID_MB` texspace case (file compat). |
| `blenkernel/intern/object_update.cc` | Removed `BKE_mball_data_update` dispatch and batch cache case |
| `blenkernel/intern/object_dupli.cc` | Removed metaball instance guard. Kept `ID_MB` no-draw guard (file compat). |
| `blenkernel/intern/material.cc` | Kept 6 `ID_MB` cases (file compat — material array/count access for old .blend files) |
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

### Modified Files — RNA (14 files)

Removed `rna_meta.cc`/`rna_meta_api.cc` from build. Cleaned `ID_MB`/`OB_MBALL`/metaball
references from `rna_object.cc`, `rna_main_api.cc`, `rna_main.cc`, `rna_ID.cc`, `rna_scene.cc`,
`rna_space.cc`, `rna_userdef.cc`, `rna_space_api.cc`, `rna_object_api.cc`, `rna_action.cc`.

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
