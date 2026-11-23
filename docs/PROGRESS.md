# PROGRESS (updated 2026-11-23)

## Done
- Phase A: tools/build_assets.py → assets/hand.glb (2.8 MB) + manifest.json; 294 structures, 334,641 tris; 30/30 bones BodyParts3D.
- Phase B: WebXR viewer (index.html + js/). Desktop load 2.4 s, 341 draw calls. Passed Quest 3 emulator test.
- Phase C: tools/build_gaps.py (+ gapgeom, gaps_finger, gaps_wrist, gaps_fat, glbio) → assets/gaps.glb (0.68 MB) + gaps.json; 75 structures, 97,664 tris, approx:true, build 9 s. Viewer loads both (369 structures, no console errors).
- Phase C visual check (screenshots, user allowed PNG viewing): fixed fat holes (inner skin = 'Skin-in' + inward-facing faces, per-face winding), sagittal bands (wrap intrinsic tendons only), Guyon roof rounded, LRL/SRL fanned + separated, RMB thinner, thumb oblique pulley flap.

## Decisions
- Units m, Y up, origin capitate centroid. three.js 0.170.0. Look values in assets/look.json (new "fat" look + layer, off at start, kept off by "Show all").
- Finger frame: centreline through joint centres; s = mm from MCP; palmar = towards own flexor tendons. Middle finger MCP–PIP 44.7, PIP–DIP 29.7 mm.
- Pulleys (Doyle): A1 −4.5…+3.5 mm about MCP, A2 5→22 mm (17, ≤60% PP), A3 ±1.5 at PIP, A4 7 mm centred mid-MP, A5 ±1.5 at DIP; scaled by PP length / middle PP. Thickness 0.7, C 1.4 mm wide ×0.4. Thumb A1 7.5, oblique 7 mm wide over 32–62% of PP (proximal-ulnar → distal-radial), A2 9 mm.
- MCP volar plates: −8 to +3 mm, 2.0 mm thick distally → 0.8 mm. Extensor: lateral bands 2.2 mm, 80°→4° from dorsal midline; terminal tendon to DP base +2.5 mm.
- Radiolunate: LRL 4 mm, SRL 5 mm wide (fanning to ×1.2 at ends), 1.3 mm, on bone; insertions 6 mm apart on lunate. RMB 0.9→0.76 mm diameter, 3 twigs (APB, FPB, OP). Guyon roof 0.8 mm, 19 mm long.
- Fat = gap from skin to deep layers − 0.5 mm (0.3–10 mm); hand median 0.6–0.9 mm, arm 3–7.5 mm.

## Known problems
- Not yet tested on a real Quest 3. GitHub Pages must be on (Settings → Pages → main, root).
- Source flexor tendons touch MCP heads → MCP volar plates overlap tendons 17–37%; little-finger A1 overlaps lumbrical/ADM 32%. 0% of new parts inside bone (tools/check_gaps.py).
- Source carpus sits ulnar to radius → LRL/SRL both run oblique. Source median nerve is thin (~1 mm) at the wrist. Z-Anatomy FR already spans over the ulnar nerve, so Guyon roof lies under it.
- Fat thin in the hand (model gap is small). Draw calls ~413 with defaults (+72).
- Fibre direction = one axis per mesh. Cut faces flat colour.

## Next step
- Phase D: user tests in headset (fat holes, sagittal bands, LRL/SRL, Guyon roof, RMB, thumb oblique pulley, pulleys/extensor), sends numbered corrections; fix only those.

## Key files
- tools/build_gaps.py, gaps_finger.py, gaps_wrist.py, gaps_fat.py, gapgeom.py, glbio.py, check_gaps.py
- js/main.js (loads gaps), looks.js (lobule shader), menu.js · assets/gaps.glb, gaps.json, look.json · docs/GAPS.md
