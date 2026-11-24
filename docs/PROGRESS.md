# PROGRESS (updated 2026-11-24)

## Done
- Phases A-C + Realistic/Cadaver views (see git history). Movement plan: docs/MOTION.md.
- Phase E1 rig: tools/build_rig.py -> assets/rig.json (60 kB), assets/joints.json, assets/carpal_model.json (Akhbari 2019 Table S1; docx git-rm'd, credited in ATTRIBUTION.md). Checks: tools/check_rig.py -> docs/MOTION_CHECK.md (24 PASS, 3 FAIL). No viewer changes.

## Decisions
- Units m, glTF frame of hand.glb. Signs: flexion +, ulnar dev +, pronation +, MCP abd + = radial, CMC1 + = flexion / palmar abduction. Paper Fig. 1: flexion +, ulnar +. Translations +distal/+ulnar/+dorsal: implied capitate centre 12 mm proximal in flexion (in capitate head).
- Translation scale = cbrt(our capitate 2731 mm3) = 13.98 mm. SP = 0. Model rest: wrist FE 0.3, RU 0.4; forearm supination 88 (vs elbow axis); finger rest flexion 6-23 deg.
- Distal row (hamate, trapezoid, trapezium) + MC2-5 rigid with capitate: Table S1 intra-row differences (up to 10 deg) made CMC joints collide; paper itself reports < 3 deg.
- Contact correction: Table S1 + smallest pushes that stop bone overlap, on a 10-deg grid (FE -70..80 x RU -20..30), bilinear at runtime. Median push 2-4 mm, max 12.5 mm at grid corners; proximal row also turns up to 15 deg.
- Forearm axis: radial head centre -> ulnar head sphere centre, distal end moved 3 mm (pronation overlap 1.1 -> 0 mm).
- Finger hinges: cylinder fits on cartilage (rms 0.3-0.7 mm); MCP centre = sphere fit. Thumb MCP/IP: joint-facing bone surface (cartilage patch fit badly). PIP2/PIP4 +5, little MCP -10/PIP +15 deg turned to aim at the tubercle.
- Thumb CMC: both Hollister axes from the trapezium saddle (convex r 6.1 mm -> FE axis in trapezium; concave r 12 mm -> AA axis in MC1, 18 mm apart). Limits FE +/-26 (Cooney arc 53), AA +36/-24 (bone contact).
- CMC4 cup axis = fallback (perpendicular to MC4); CMC5 = cylinder fit.

## Known problems
- Ulnar deviation 25-30 deg: triquetrum/pisiform 4.5 mm inside the rigid hamate (fine up to 20 deg).
- Little finger flexed points 20 deg off the scaphoid tubercle (target 15); aiming it hits the ring finger in a full fist.
- CMC1 axes are perpendicular (one quadric; MC1 base surface was not a clean saddle), Hollister found them oblique.
- Rest supination 88 > default limit 80. build_rig.py takes ~8 min (carpal grid).

## Next step
- E1 fixes if wanted (ulnar deviation, little finger), then Phase E2 (bind tissues, cables).

## Key files
- tools/build_rig.py, rigfit.py (fits), rigpose.py (pose engine = reference for js/rig.js), rigsolve.py (contact/axis refinement), rigrange.py (ranges), check_rig.py
- assets/rig.json, joints.json (edit ranges here), carpal_model.json; docs/MOTION_CHECK.md; data/raw/rig_log.txt
