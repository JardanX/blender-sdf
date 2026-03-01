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
