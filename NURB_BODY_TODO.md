# NURB Body Todo

Persistent tracker for NURB Body bevel, boolean, selection, and viewport issues.

## Current Bevel/Boolean Issues

- [x] Viewport lag after fillet bevel and boolean operations: edge polyline cache key no longer hashes whole object/body memory and avoids invalidating on unrelated memory churn.
- [x] Viewport pan/orbit still heavy with line overlay visible: overlay now reuses cached OCCT edge polylines, builds retained GPU line batches, and applies object transforms on the GPU instead of rebuilding world-space immediate-mode line vertices every redraw.
- [x] Fast retained line batches looked pixelated compared to the previous overlay: retained batches now use Blender's screen-space polyline shader with line smoothing instead of raw backend line rasterization.
- [x] Selected/hovered NURB edge lines visually conflicted with the normal line and Blender object/wire line: normal batches now exclude highlighted edges, and hovered/selected batches draw on top without depth testing.
- [x] Selecting an edge left the owning object selected: successful edge selection now force-syncs the object/base selected flag off, while the NURB Body outline pass still draws the dark silhouette instead of the selected-object color.
- [x] Edge selection only worked on the active NURB Body: edge hit-testing now scans visible NURB Body objects in edge mode and activates the clicked body as the edge-tool target without selecting its object base.
- [x] Clicking empty space after selecting a NURB Body edge did not reliably clear edge selection once the object base was deselected: the edge select click operator now runs from object mode and can retarget the visible NURB Body that owns the active edge selection.
- [x] Hover/select edge queries copied cached polylines every mouse move: editor hit-testing now reads the cached polyline span directly.
- [x] Blended/chamfered edges could lose their selectable hit reference after the topology changed: selectable edge references are kept for blended edges with a radius-expanded matching threshold.
- [x] Surface outline edges without an explicit boolean/body reference were visible but not selectable: visible surface lines now get a selectable final-edge fallback when they fit the 64-edge mask.
- [x] Chamfer/fillet-created sharp edges reselected the previous bevel instead of becoming independent targets: already-blended body/boolean/final refs are no longer reused as hit targets, so generated sharp edges fall through to final-edge selection.
- [x] Final sharp edges after body chamfers or boolean cuts could not receive a separate chamfer/fillet pass: final surface edges now have independent selection, hover, radius, chamfer, and bevel-order state.
- [x] Object/face selection modes still allowed edge picking: visible NURB Body edge hit-testing now requires the active NURB Body to be in Edge select mode.
- [x] Ctrl+B bevel increments feel too large: modal bevel uses radius-scaled mouse steps and throttles preview rebuilds.
- [x] Bottom boolean rim cannot be selected or beveled: cutter rim edges are included in selectable cut-edge references.
- [x] Beveling a second edge snaps to the previous edge value: fresh edges now start from zero, and explicit per-edge radius zero no longer falls back to the previous global bevel radius.
- [x] Bevel guide line is pulled from a corner: Ctrl+B captures the active edge screen anchor before changing bevel state.
- [x] Changing one edge to chamfer/fillet changes previous edges: chamfer state is stored per edge with `chamfer_edges`.
- [x] Pressing `C` only enables chamfer: pressing `C` again toggles the active edge set back to fillet; `F` still forces fillet.
- [x] Confirmed bevels/chamfers are affected by later bevels: per-edge `bevel_order` now replays confirmed operations in confirmation order instead of grouping all chamfers before all fillets.

## Needs Manual Rebuild Verification

- [ ] Rebuild Blender after the line-overlay span fix and verify `bf_blenkernel`, `bf_draw`, and `bf_editor_object` compile.
- [ ] With line overlay enabled, pan/orbit a scene with several booleans and fillets; confirm redraw no longer spikes from CPU-side line rebuilding.
- [ ] Confirm retained GPU lines match the previous smooth visual quality while staying responsive during pan/orbit.
- [ ] Select and hover an edge that overlaps Blender's object/wire outline; confirm only the NURB highlight owns the visible line.
- [ ] Select an edge and confirm the object is no longer object-selected, while Ctrl+B and edge hover still work on the NURB Body that owns the selected edge.
- [ ] With one NURB Body active in Edge mode, hover/select edges on another visible NURB Body without first selecting that object.
- [ ] Select an edge, then click viewport background/empty space; confirm the edge selection and hover clear without reselecting the NURB Body object.
- [ ] Verify all visible NURB Body line edges can be hovered and selected in Edge select mode.
- [ ] Chamfer a base edge, select one of the newly created sharp edges, then Ctrl+B it; confirm this adds a new fillet/chamfer and does not readjust the original chamfer.
- [ ] After a boolean cut, select both concave and convex cut/rim edges and confirm Ctrl+B can add fillet/chamfer operations to each.
- [ ] Switch to Face/Object select mode with the `2`/`3` modes and confirm clicking NURB Body lines does not select edges; switch back to Edge mode with `1` and confirm edge picking returns.
- [ ] Verify sharp edges, chamfer-created edges, and boolean-created cut/rim edges remain selectable and can receive Ctrl+B fillet/chamfer operations.
- [ ] Rebuild Blender and verify Ctrl+B on a fresh edge starts at zero and grows gradually.
- [ ] Verify a previously beveled edge keeps its radius while beveling a different edge.
- [ ] Verify mixed chamfer and fillet edges can coexist on the same NURB Body and on boolean cut rims.
- [ ] Verify fillet first, then chamfer another edge, leaves the first confirmed fillet unchanged.
- [ ] Verify the lower boolean rim shown in the screenshot can be selected, hovered, and beveled.
- [ ] Stress test several boolean cutters plus fillets for remaining viewport lag.
