# NURB Body Todo

Persistent tracker for NURB Body bevel, boolean, selection, and viewport issues.

## Current Bevel/Boolean Issues

- [x] Viewport lag after fillet bevel and boolean operations: edge polyline cache key no longer hashes whole object/body memory and avoids invalidating on unrelated memory churn.
- [x] Ctrl+B bevel increments feel too large: modal bevel uses radius-scaled mouse steps and throttles preview rebuilds.
- [x] Bottom boolean rim cannot be selected or beveled: cutter rim edges are included in selectable cut-edge references.
- [x] Beveling a second edge snaps to the previous edge value: fresh edges now start from zero, and explicit per-edge radius zero no longer falls back to the previous global bevel radius.
- [x] Bevel guide line is pulled from a corner: Ctrl+B captures the active edge screen anchor before changing bevel state.
- [x] Changing one edge to chamfer/fillet changes previous edges: chamfer state is stored per edge with `chamfer_edges`.
- [x] Pressing `C` only enables chamfer: pressing `C` again toggles the active edge set back to fillet; `F` still forces fillet.

## Needs Manual Rebuild Verification

- [ ] Rebuild Blender and verify Ctrl+B on a fresh edge starts at zero and grows gradually.
- [ ] Verify a previously beveled edge keeps its radius while beveling a different edge.
- [ ] Verify mixed chamfer and fillet edges can coexist on the same NURB Body and on boolean cut rims.
- [ ] Verify the lower boolean rim shown in the screenshot can be selected, hovered, and beveled.
- [ ] Stress test several boolean cutters plus fillets for remaining viewport lag.
