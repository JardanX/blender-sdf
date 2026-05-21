# NURB Body TODO

## Bevel Readjust

- [x] New surface bevel commits store generated bevel face provenance from whole-face/source-edge proximity so face-mode `Ctrl+B` can resize the selected bevel, not the latest bevel.
- [x] Face-mode resize refuses unknown bevel faces instead of beveling their boundary edges; unprovenanced stages can lazily bind when source edge keys are still visible.
- [x] Face-mode `Ctrl+B` checks selected-face provenance directly instead of requiring face-boundary edge conversion first.
- [x] Removed single unprovenanced-stage fallback because it resized the same bevel for every selected face.
- [x] Removed unprovenanced bevel-face lazy matching so face-mode readjust cannot pick a wrong stage.
- [x] Existing bevel-face resize refuses when later NURB Body stages depend on the selected bevel.
- [x] Face-mode `Ctrl+B` now uses Plasticity-style selected-face refillet stage instead of mutating old edge bevel history.
- [x] Refillet face modal supports `C` chamfer and `F` fillet while dragging.
- [x] Face refillet modal clears runtime edge/face caches and updates selected result face during live topology changes.
- [x] Existing bevel-face resize restores original stage if no valid solved preview exists at confirm time.
- [x] Face-mode `Ctrl+B` accepts hovered bevel face provenance as well as explicitly selected face provenance.
- [x] Surface bevel provenance assignment excludes pre-existing faces so original side faces are not stored as bevel-face provenance.
- [x] Surface bevel provenance assignment stores generated face keys even when source edge samples are unavailable.
- [x] Surface bevel provenance stores all generated face keys for a stage so selecting any generated bevel/chamfer face can readjust that stage.
- [x] Disable NURB Body silhouette overlay to avoid Vulkan indirect draw crash from mesh surface batches.
- [x] Edge overlay falls back to analytic NURB Body polylines when live refillet topology has no matching mesh edges.
- [x] Edge overlay uses cached analytic NURB edge batches during recalculation, avoiding stale mesh edge GPU batches.
- [x] Removed fast bevel timeout, solved-preview confirmation gate, failure clamp, stale last-good reuse, and reduced-radius fallback.
- [ ] Verify in Blender: select confirmed bevel face, press `Ctrl+B`, adjust radius, toggle `C` chamfer and `F` fillet.
