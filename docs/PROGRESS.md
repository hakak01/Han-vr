# PROGRESS (updated 2026-11-24)

## Done
- Phase A: assets/hand.glb (2.7 MB) + manifest.json; 294 structures, 334,641 tris; 30/30 bones BodyParts3D.
- Phase B: WebXR viewer (index.html + js/). Desktop load 2.4 s. Passed Quest 3 emulator test.
- Phase C: gaps.glb (0.66 MB, 75 structures, approx:true) + visual-check fixes.
- Realistic view: js/tissues.js (MeshPhysicalMaterial per tissue, clearcoat, triplanar KTX2 detail, HDRI, OT spotlight), js/fibres.js, assets/tissues.json.
- This session: HDRI installed (assets/env/hdri_1k.hdr, 1.6 MB, loads, no warning). New Cadaver palette from 12 user photos (embalmed, latex-injected). Menu View: Present / Realistic / Cadaver; key V cycles all three. Headless test: no errors, 413 draw calls in all 3 views.

## Decisions
- Palettes: tissues.json "palettes".cadaver overrides only colour/roughness/clearcoat/bump/cavity of "tissues" (living stays default). Same materials, no new shaders. startView may be present/realistic/cadaver.
- Photo medians: muscle #966350, tendon #f0e2d0, ligament #e8d8cc, nerve #c89c7c (range bd997d–d49d5b), latex artery #be6258, vein #b07f80 (1 photo), skin #cc9e86, fat #d8a070.
- Cadaver albedo after 2 render-vs-photo rounds: muscle #8a5a4a, tendon #f4e8d8, ligament #e2d4c2, nerve #f0c49c, artery #d86a5e (bump 0.4), vein #a87c84, bone #e2d3b0, fat #d6a460, skin #d2a690. Rendered within ~3% (muscle, tendon, ligament, skin, bone, artery); nerve ~8% and vein ~12% darker than photo.
- Photos are copyrighted atlas images: used only to measure colour, not stored in the repo.

## Known problems
- cdn.jsdelivr.net blocked in this cloud session: headless test serves three@0.170.0 from a local npm copy (Playwright route). App still uses the CDN.
- Fat hidden by default, so fat colour not render-checked. Vein colour from 1 photo only.
- Not tested on a real Quest 3 (clearcoat + 3 texture reads per pixel). Fibre direction wrong on U-shaped structures and pulleys (annular fibres).
- From Phase C: volar plate/tendon overlap, thin median nerve, fat thin in the hand, cut faces flat colour.

## Next step
- Phase D: Quest 3 headset test (fps in Tools tab in all 3 views), user gives numbered corrections. Optional: living intra-op photos to check the Realistic (living) palette the same way.

## Key files
- js/tissues.js (applyPalette, setView), menu.js (View row), main.js (startView) · assets/tissues.json (tissues, palettes), assets/env/hdri_1k.hdr, assets/tex/*.ktx2
- tools/make_tissue_tex.py, tools/encode_ktx2.mjs · js/looks.js (present view) · assets/look.json
