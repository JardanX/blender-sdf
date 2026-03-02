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
