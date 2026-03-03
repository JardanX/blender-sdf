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

**In-place incremental baking:** When objects move (without add/remove), only bricks overlapping moved objects' old+new AABBs are rebaked **in-place** in the existing atlas. No classify, no copy — clean brick data stays at its correct atlas positions. New slots are allocated on-the-fly for dirty bricks that are currently void. Falls back to full bake on first frame, resolution change, object add/remove, grid object presence, or atlas capacity exceeded.

Combined: moving 1 of 10 spread-out objects → ~20% bricks rebaked, each evaluating ~1-3 objects.

| File | Change |
|------|--------|
| `source/blender/draw/engines/sdf/shaders/sdf_classify_comp.glsl` | Added per-brick AABB culling in object loop (expand by `brick_half_diag`) |
| `source/blender/draw/engines/sdf/shaders/sdf_bake_comp.glsl` | Added incremental early-out (skip clean bricks), per-brick AABB culling in object loop (expand by 2-voxel overlap border) |
| `source/blender/draw/engines/sdf/shaders/infos/sdf_shader_infos.hh` | `dirty_bricks_tx` sampler + `incremental` push constant in `sdf_bake` (copy_bricks info removed) |
| `source/blender/draw/engines/sdf/sdf_engine.cc` | In-place incremental baking: `prepare_incremental_bake()` reads indirection, computes dirty mask, allocates new slots for void bricks, re-uploads indirection. Removed copy-based infrastructure (`prev_indirection_tx_`, `prev_compact_atlas_tx_`, `copy_bricks_sh_`, `dispatch_copy_bricks()`, `compute_dirty_bricks()`). Simplified `ensure_compact_atlas()` and destructor. Removed debug printfs |
| `source/blender/draw/engines/sdf/shaders/sdf_copy_bricks_comp.glsl` | **DELETED** — no longer needed; clean bricks stay in-place |
| `source/blender/draw/CMakeLists.txt` | Removed `sdf_copy_bricks_comp.glsl` from shader file list |

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
