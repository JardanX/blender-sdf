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
- [x] Beveling an edge with a radius past OCCT's stable limit could make the evaluated mesh disappear: single-edge blends now validate the result and binary-search down to the largest preview-valid radius before falling back to the previous shape.
- [x] Boolean creation could appear to bevel an edge even when the user did nothing: the legacy "radius with no explicit edge means edge 0" fallback is disabled.
- [x] Final-edge chamfers/fillets could be forgotten when adding later booleans because surface edge indices came from unstable OCCT map order: final surface edge slots now derive from quantized edge geometry instead.
- [x] Clicking one of several close bevel-ring outlines could assign the line to the first nearby selectable edge and bevel a different edge: line-to-edge reference matching now chooses the nearest candidate inside tolerance.
- [x] Replaying earlier bevels could make a later edge reference miss and then fall back to the same numeric OCCT edge index, causing a random edge to bevel: failed remaps now skip instead of applying to a different edge.
- [x] Beveling a sharp edge created by a previous final chamfer/fillet could show the correct outline but not modify the mesh: final-edge replay now recomputes the stable surface-edge map after each final operation, so generated edges can be targeted by later operations.
- [x] Fallback generated-edge overlay IDs used the raw stable hash slot while geometry replay used collision probing: fallback final-edge polylines now reserve and probe final slots the same way as the evaluator.
- [x] Edge hover/selection worked but white/orange highlights could be hidden by depth fighting with the mesh/dark outline: highlighted NURB Body edge batches now draw without depth testing after the normal dark batch.
- [x] During modal bevel, the outline could show a new radius while the mesh stayed sharp because only the overlay saw sub-threshold radius updates: every modal radius update now tags geometry so mesh and line overlay stay synchronized.
- [x] Ctrl+B bevel increments feel too large: modal bevel uses radius-scaled mouse steps and throttles preview rebuilds.
- [x] Bottom boolean rim cannot be selected or beveled: cutter rim edges are included in selectable cut-edge references.
- [x] Beveling a second edge snaps to the previous edge value: fresh edges now start from zero, and explicit per-edge radius zero no longer falls back to the previous global bevel radius.
- [x] Bevel guide line is pulled from a corner: Ctrl+B captures the active edge screen anchor before changing bevel state.
- [x] Changing one edge to chamfer/fillet changes previous edges: chamfer state is stored per edge with `chamfer_edges`.
- [x] Pressing `C` only enables chamfer: pressing `C` again toggles the active edge set back to fillet; `F` still forces fillet.
- [x] Confirmed bevels/chamfers are affected by later bevels: per-edge `bevel_order` now replays confirmed operations in confirmation order instead of grouping all chamfers before all fillets.
- [x] Alt+Z/xray made selected NURB silhouettes draw hidden back outlines at full opacity: NURB custom outline IDs now keep true scene-depth occlusion instead of using xray's unoccluded outline alpha.
- [x] Cylinder seam/closure edges could be emitted as real edge lines: edge-face filtering now de-duplicates OCCT faces with `IsSame()` and rejects single-face seam edges.
- [x] Alt+Z made NURB edge lines draw every hidden/back-side edge through the object: the NURB edge overlay now writes a cheap NURB surface depth prepass in xray before drawing its retained line batches.
- [x] Grid/axis lines could remain visible through NURB bodies in xray: the NURB xray depth prepass now runs before the grid, while NURB edge lines still draw after the grid.
- [x] Hover/selection could stop updating after edge selection deselected the object base: edge picking now scans visible NURB bodies that are themselves in Edge mode instead of requiring the active object to be in Edge mode.
- [x] Edge hover/select bindings were losing priority to Blender's normal 3D View select path: NURB Body hover/click bindings now live in the 3D View keymap before `view3d.select`, and miss/clear cases pass through to normal selection.
- [x] Hover/selected edge lines still drew black after selection: overlay line and silhouette passes now read NURB hover/selection state from the original object data instead of the evaluated draw object.
- [x] Unmatched pre-blend selectable refs could draw a requested/rounded outline even when OCCT left the evaluated mesh sharp: fallback ref-only lines are no longer emitted, and final refs are hidden only for blends that actually applied to the result shape.
- [x] Final/generated edge selection and bevel replay depended on unstable numeric slots, hash probing, and nearest-line fallback: selected final edges now store an evaluated OCCT edge geometry key, confirmed bevels replay only when that exact key exists, and ambiguous/unmatched generated edges remain unselectable instead of beveling the wrong edge.
- [x] Previous confirmed final-edge fillets/chamfers could disappear when adding another bevel because circular/arc edge keys used sampled bounds that shifted after OCCT rebuilt topology: edge identity now uses duplicate-safe center, length, and covariance invariants from fixed curve samples.

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
- [ ] On a cylinder with one chamfer plus two fillets, bevel the remaining cap/body edge past its stable limit; confirm the mesh stays visible and the bevel clamps instead of disappearing.
- [ ] Add a boolean after confirming a chamfer/fillet on an existing sharp final edge; confirm the earlier operation remains on the same edge.
- [ ] Add a boolean to a NURB Body with no active edge bevel command; confirm no new boolean/cap edge is pre-beveled.
- [ ] On close parallel bevel-ring edges, select each ring one at a time and confirm Ctrl+B affects the highlighted ring rather than a neighboring/random edge.
- [ ] With several existing bevels, add another bevel and confirm no unrelated edge changes if the saved edge reference cannot be remapped.
- [ ] Chamfer a cylinder cap, fillet one generated sharp ring, then bevel another generated sharp ring; confirm the mesh changes on the highlighted edge, not just the outline.
- [ ] On a chamfer with several generated rings, select rings that hash near each other and confirm each highlighted ring bevels the same mesh edge.
- [ ] Hover and select body, boolean, and final generated edges; confirm white hover and orange selection draw visibly on top of the dark NURB Body lines.
- [ ] Confirm hover turns edges white before object selection, left-click selects the edge orange, shift-click multi-selects, and clicking a non-edge still performs normal Blender object selection.
- [ ] Confirm selected/hovered edge colors still update when the viewport is drawing evaluated NURB body meshes.
- [ ] Drag Ctrl+B slowly on a final/generated edge and confirm the mesh surface updates at the same time as the line outline, with no sharp-edge/rounded-outline mismatch.
- [ ] Repeat the cylinder chamfer plus two fillets case and bevel the remaining rings; confirm no rounded/requested outline appears unless the evaluated mesh also changes.
- [ ] Select each generated ring after a confirmed chamfer/fillet sequence, add another bevel, and confirm unmatched/missing stored edges are skipped rather than remapped to another visible ring.
- [ ] Confirm a fillet/chamfer on a boolean rim, select another nearby generated edge, bevel it, and verify the previous confirmed rim bevel remains.
- [ ] Switch to Face/Object select mode with the `2`/`3` modes and confirm clicking NURB Body lines does not select edges; switch back to Edge mode with `1` and confirm edge picking returns.
- [ ] Verify sharp edges, chamfer-created edges, and boolean-created cut/rim edges remain selectable and can receive Ctrl+B fillet/chamfer operations.
- [ ] Rebuild Blender and verify Ctrl+B on a fresh edge starts at zero and grows gradually.
- [ ] Verify a previously beveled edge keeps its radius while beveling a different edge.
- [ ] Verify mixed chamfer and fillet edges can coexist on the same NURB Body and on boolean cut rims.
- [ ] Verify fillet first, then chamfer another edge, leaves the first confirmed fillet unchanged.
- [ ] Verify the lower boolean rim shown in the screenshot can be selected, hovered, and beveled.
- [ ] Stress test several boolean cutters plus fillets for remaining viewport lag.
