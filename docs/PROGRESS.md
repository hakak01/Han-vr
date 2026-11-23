# PROGRESS (updated 2026-11-23)

## Done
- Phase A: tools/build_assets.py → assets/hand.glb (2.8 MB) + manifest.json; 294 structures, 334,641 tris; 30/30 bones BodyParts3D.
- Phase B: WebXR viewer (index.html + js/). Desktop load 2.4 s, 341 draw calls. Passed Quest 3 emulator test.
- Phase C: gaps.glb (0.68 MB, 75 structures, approx:true) + visual-check fixes (fat holes, sagittal bands, Guyon roof, LRL/SRL, RMB, thumb oblique pulley).
- Realistic view: js/tissues.js (MeshPhysicalMaterial per tissue, clearcoat, triplanar KTX2 detail, HDRI, overhead OT spotlight), js/fibres.js (fibre frame follows each mesh's centreline), assets/tissues.json (all values + name→tissue rules). Menu Tools → View: Present / Realistic; key V on desktop. Headless test: no errors, 413 draw calls (unchanged), 23–32 shader programs.

## Decisions
- 12 tissue types: muscle, tendon, ligament, nerve, artery, vein, bone, cartilage, fat, skin + nail, sheath. Mapping: glTF slot (Cartilage/Tendon/Nail) → exact-name overrides → name regex rules → layer default. Attachments keep present look.
- Detail maps: 9 × 1024² KTX2 UASTC+Zstd, RDO 2.0, mipmaps, linear (RG normal, B cavity) = 7.7 MB in assets/tex/. RMS normal tilt 4° (cartilage) – 18° (muscle, fat). Tile mm: tendon 6, skin 6, ligament 8, nerve 8, vessel 10, bone 10, muscle 12, fat 20, cartilage 20.
- Fibre frame: centreline bins every 4 mm (2–40 bins) along the PCA axis, smoothed, parallel-transported; checked: stripes run along FDP (cross-gradient 4.2 vs 1.9).
- Realistic loads after the main model (present view first), then switches (startView "realistic"). Extra download 7.7 MB textures (+~1.5 MB HDRI).
- OT light: SpotLight 0.9 m above model, 0.25 m toward viewer, 28°, penumbra 0.7, 2.6 cd, #fff1e0, no shadows. Fill: hemi 0.2, dir 0.35. No SSS, no sheen, no transmission.

## Known problems
- HDRI not yet in repo (Poly Haven blocked here). Until assets/env/hdri_1k.hdr exists, realistic view uses the built-in room light (console warning).
- Not tested on a real Quest 3: clearcoat + 3 texture reads per pixel may cost fps; check fps in Tools tab.
- Fibre direction wrong where a structure folds back on itself (U-shape) or for flat sheets whose fibres are not along the longest side (e.g. pulleys: annular fibres).
- From Phase C: volar plate/tendon overlap, thin source median nerve, fat thin in the hand, cut faces flat colour.

## Next step
- User uploads HDRI to data/upload/ → copy to assets/env/hdri_1k.hdr. Then Phase D: headset test, numbered corrections.

## Key files
- js/tissues.js, fibres.js, anatomy.js (setView), menu.js (View buttons), main.js · assets/tissues.json, assets/tex/*.ktx2
- tools/make_tissue_tex.py, tools/encode_ktx2.mjs (needs `npm install ktx2-encoder` in data/raw/enc)
- tools/build_gaps.py etc. · js/looks.js (present view) · assets/look.json
