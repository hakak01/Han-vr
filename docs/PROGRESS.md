# PROGRESS

## Phase A — asset pipeline: DONE (2026-11-23)
- Rebuild: `pip install bpy scipy`, put sources in data/raw/ (see below), then `python3 -I tools/build_assets.py` (~25 s).
  - data/raw/zblend/Z-Anatomy/Startup.blend (from Z-Anatomy.zip)
  - data/raw/bp3d/ (sparse clone of BodyParts3D; only the bone STLs checked out)
- Code: tools/build_assets.py (main), tools/zsel.py (selection, layers, names), tools/boneswap.py (fit + ICP).
- Output: assets/hand.glb (2.8 MB, meshopt) + assets/manifest.json. Log: data/raw/build_log.txt.
- 294 structures, 334,641 triangles. Units metres, Y up; origin = capitate centroid.
- Bones: all 30 (27 hand + radius, ulna, humerus) swapped to BodyParts3D; mean error 0.07–0.33 mm, max ≤2.5 mm.
- Names: ".r" removed; footprints named "Muscle (origin)" / "Muscle (insertion 2)", manifest field kind = origin/insertion.
- Material names kept from Z-Anatomy ("Bone…", "Cartilage", "Tendon", …) for the viewer.
- Selection: right side, Z-Anatomy upper-limb collections/parents, bottom below mid-humerus (z 1.246 m).
- Gaps: docs/GAPS.md (3 missing, 8 partial).

## Phase B — WebXR viewer: DONE (2026-11-23)
- index.html + js/ (main, anatomy, looks, panel, menu, xr). three.js 0.170.0 from jsDelivr. No build step.
- Colours/roughness/patterns/default layers: assets/look.json (edit, then reload page).
- Tested headless: desktop load 2.4 s, 294 structures, 341 draw calls; clip/isolate/ghost/focus/search OK.
- Tested with Meta's Quest 3 emulator (iwer): Enter VR, controllers, ray select, wrist menu, grab, 2-hand zoom, Exit OK.
- Not testable here: real fps on Quest 3, passthrough look, wrist-menu comfort.
- Hosting: GitHub Pages from main, root folder (must be switched on once in repo Settings → Pages).

## Next: Phase C — build missing structures (docs/GAPS.md)
