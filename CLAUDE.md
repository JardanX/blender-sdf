# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Blender 5.0.1 fork** (base: `v5.0-release`) that adds **SDF (Signed Distance Field)** as a native object type. The fork is part of the **MathOPS** project — a GPU compute-shader SDF renderer that runs as a Blender addon. The fork itself provides the data foundation; rendering is handled externally by the addon.

Key modifications from upstream Blender:
- **SDF object type** (`OB_SDF = 31`, `ID_SF = 'SF'`) — 6 new files, 26 modified
- **Metaball removal** — SDF replaces metaballs; runtime code deleted, DNA kept as tombstones for .blend compat
- **EEVEE removal** — entire render engine deleted (~297 files); MathOPS uses its own renderer
- **Feature culling** — grease pencil, sound editor, VSE, movie clip UI removed (libraries kept compiled for deps)

All changes are tracked in `SDF_CHANGES.md` at the repo root — consult it before modifying any SDF-related code.

## Building

### Windows (primary platform)

```batch
.\make.bat ninja
```

This auto-detects MSVC (2019/2022/2026), checks SVN library dependencies, runs CMake with the Ninja generator, and builds. Output goes to `build/bin/Release/`. The build directory already exists with a compiled binary.

To rebuild after C++ changes, run `make.bat ninja` again — it's incremental. Ninja is faster than MSBuild for incremental builds.

### Linux/macOS

```bash
make full ninja ccache
```

Or manual CMake:
```bash
mkdir build_cmake && cd build_cmake
cmake .. && make -j$(nproc)
```

### Running the built Blender

```
build/bin/Release/blender.exe
```

## Running Tests

```batch
.\make.bat test
```

This runs CTest which executes both C++ GTests (`tests/gtests/`) and Python integration tests (`tests/python/`). Tests run against the installed Blender binary with `--background --factory-startup --debug-memory --debug-exit-on-error`.

From the build directory:
```bash
cd build && ctest --output-on-failure          # All tests
cd build && ctest -R test_name --output-on-failure  # Single test
```

## Architecture: How Blender's Type System Works

Understanding this is essential for any SDF work. Adding a new data type in Blender requires touching **6 layers**:

### 1. DNA (Data Structure) — `makesdna/`
Binary-serializable C structs. The `makesdna` code generator reads these headers at build time.
- `DNA_sdf_types.h` — The `SDF` struct definition
- `DNA_sdf_defaults.h` — Default values via `_DNA_DEFAULT_SDF`
- `DNA_object_types.h` — `OB_SDF = 31` enum value
- `DNA_ID_enums.h` — `ID_SF`, `FILTER_ID_SF`, `INDEX_ID_SF`

### 2. BKE (Kernel) — `blenkernel/`
Runtime management. Every ID type needs an `IDTypeInfo` struct with callbacks.
- `intern/sdf.cc` — `IDType_ID_SF`: init, copy, free, blend_write, blend_read
- `BKE_main.hh` — `ListBase sdfs` in `Main` struct
- `intern/main.cc` — Maps `INDEX_ID_SF` to `&bmain.sdfs` (crash without this)
- `intern/idtype.cc` — Three registration points: `INIT_TYPE`, two `CASE_IDINDEX` switches
- `intern/object.cc` — `case OB_SDF` in naming, data creation, ID mapping, duplication

### 3. RNA (Python API) — `makesrna/`
Exposes DNA to Python/UI. Generated at build time by `makesrna.cc`.
- `intern/rna_sdf.cc` — Property definitions (type, size, bevel, color, blend, etc.)
- `intern/rna_ID.cc` — `case ID_SF: return &RNA_SDF;` in `ID_code_to_RNA_type()` (without this, `ob.data` is generic `ID`, not `SDF`, and `context.sdf` is always None)
- `intern/rna_main.cc` — `bpy.data.sdfs` collection
- `intern/rna_main_api.cc` — `.new()` / `.remove()` / `.tag()` API

### 4. Depsgraph — `depsgraph/`
Dependency graph for evaluation ordering.
- `deg_builder_nodes.cc` — Builds evaluation nodes for `ID_SF`/`OB_SDF`
- `deg_builder_relations.cc` — Builds dependency relations

### 5. Editors — `editors/`
UI operators and interaction.
- `object/object_sdf.cc` — `OBJECT_OT_sdf_add` operator
- `object/object_ops.cc` — Operator registration

### 6. Draw — `draw/`
Rendering pipeline. SDF objects currently pass through as no-ops in the draw system (the MathOPS addon handles rendering).
- `draw_cache.cc` — `case OB_SDF: break;`

### Registration Checklist (for adding new ID types)

These are the exact registration points that caused crashes or broken functionality when missed:
1. `idtype.cc`: `INIT_TYPE()` + both `CASE_IDINDEX()` switches
2. `main.cc`: `lb[INDEX_ID_*] = &bmain->listbase` in `BKE_main_lists_get()`
3. `CMakeLists.txt` (root `source/blender/`): DNA headers in `SRC_DNA_INC`
4. `makesrna.cc`: Register in RNA code generator array
5. `rna_main_api.cc`: Include the DNA header
6. `rna_ID.cc`: `case ID_SF: return &RNA_SDF;` in `ID_code_to_RNA_type()` — without this, `ob.data` resolves to generic `ID` instead of the proper RNA type, breaking `context.sdf` and all Properties panels

## Key Source Directories

| Directory | Purpose |
|-----------|---------|
| `source/blender/makesdna/` | C struct definitions (binary serialized) |
| `source/blender/makesrna/intern/` | Python API property definitions |
| `source/blender/blenkernel/` | Core kernel (ID management, object ops) |
| `source/blender/blenloader/` | .blend file versioning and I/O |
| `source/blender/depsgraph/` | Dependency graph |
| `source/blender/editors/` | UI operators, all editor spaces |
| `source/blender/draw/` | Draw engines (Workbench, Overlay — no EEVEE) |
| `source/blender/gpu/` | GPU abstraction |
| `source/blender/nodes/` | Node systems (shader, geometry, composite) |
| `source/blender/windowmanager/` | Event loop, keymaps, window management |
| `scripts/startup/bl_ui/` | Python UI panels and menus |

## SDF-Specific Identifiers

- **Object type**: `OB_SDF = 31` (in `DNA_object_types.h`)
- **ID code**: `ID_SF = MAKE_ID2('S', 'F')` (in `DNA_ID_enums.h`)
- **Filter bit**: `FILTER_ID_SF` (bit 42)
- **Struct**: `SDF` (in `DNA_sdf_types.h`)
- **Python**: `bpy.data.sdfs`, object type string `"SDF"`
- **ListBase**: `Main::sdfs`

## Conventions

- **Removed features are commented, not deleted**, with `/* MATHOPS: Removed */` markers for easy grep
- **Metaball DNA kept as tombstones** — `OB_MBALL=5` and `ID_MB` remain defined but deprecated for .blend compat
- **`SDF_CHANGES.md` must be updated** after every significant change to SDF-related code
- Blender uses `.cc`/`.hh` for C++ (not `.cpp`/`.hpp`)
- DNA structs require proper alignment padding (`char _pad[N]`)
- Code style: `.clang-format` in repo root (K&R-ish, 100 col)

## File Compatibility

Old .blend files with metaballs must still load. The following are intentionally kept:
- `DNA_meta_types.h` (struct definition)
- `mball.cc` (IDTypeInfo stub with blend_write/blend_read)
- `ID_MB`/`OB_MBALL` enum values
- Material array accessors for `ID_MB` in `material.cc`
- Animation system iteration for `ID_MB`
