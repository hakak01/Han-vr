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

## Next: Phase B — WebXR viewer
