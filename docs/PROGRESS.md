# PROGRESS (updated 2026-11-23)

## Done
- Phase A: tools/build_assets.py → assets/hand.glb (2.8 MB, meshopt) + manifest.json; 294 structures, 334,641 triangles; 30/30 bones swapped to BodyParts3D (mean 0.07–0.33 mm, max ≤2.5 mm). Rebuild ~25 s.
- Phase B: WebXR viewer (index.html + js/). Desktop load 2.4 s, 341 draw calls, 19 shader programs. Passed Meta iwer Quest 3 emulator test (VR, ray select, wrist menu, grab, 2-hand zoom, exit).

## Decisions
- Selection: right side, Z-Anatomy upper-limb collections, bottom below mid-humerus z=1.246 m; whole humerus kept.
- Units metres, Y up, origin = capitate centroid (Z-Anatomy world -0.2647, -0.0182, 0.8427); viewer re-centres on bbox.
- Curves: resolution_u ≤3, bevel_resolution ≤2. Repair: merge 0.01 mm, smooth <300 verts, clamp 0.3 mm.
- Names: ".r" removed; footprints "X (origin)" / "X (insertion 2)", manifest kind field.
- three.js 0.170.0 (jsDelivr). VR placement 55 cm ahead, 30 cm below eyes, palm to user. Zoom 0.5–20×; ghost = 25%; focus enlarges to ~12 cm, 35 cm ahead.
- Look values in assets/look.json; attachments off at start; sheaths 35%, vessels 85% opacity.

## Known problems
- Not yet tested on a real Quest 3 (fps, passthrough, wrist-menu comfort).
- GitHub Pages must be switched on by user (Settings → Pages → main, root) → hakak01.github.io/Han-vr/.
- Fibre direction = one main axis per mesh (poor on arches/branches). Cut faces = flat colour, no true caps.
- Gaps (docs/GAPS.md): 3 missing (extensor mechanism, recurrent motor branch, subcutaneous fat), 8 partial.

## Next step
- Phase C: tools/build_gaps.py → assets/gaps.glb (approx:true); then Phase D headset corrections.

## Key files
- tools/build_assets.py, zsel.py, boneswap.py · js/main.js, anatomy.js, looks.js, panel.js, menu.js, xr.js
- assets/hand.glb, manifest.json, look.json · docs/SPEC.md, GAPS.md · data/raw/build_log.txt (local only)
