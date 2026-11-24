# PROGRESS (updated 2026-11-24)

## Done
- Phases A-C, Realistic/Cadaver views, E1 rig (git history; plan in docs/MOTION.md).
- Phase E2: tools/bind_tissues.py -> assets/hand_rig.glb (4.7 MB, 369 structures, one glTF skin, 33 joints = 30 bones + 3 forearm twist) + assets/cables.json (1.6 MB, 172 cables). Reference poser tools/tisspose.py; checks tools/check_tissues.py -> docs/MOTION_CHECK.md (E2 section). Nerve/vessel centrelines: tools/zcurves.py (Z-Anatomy curves, 64 structures). js/anatomy.js, tissues.js, fibres.js: textures/fibres use bind-pose positions, fibre direction turned by the skin matrix. Viewer still loads hand.glb (switch in E3). three.js skinning of hand_rig.glb = python within 0.07 mm.

## Decisions
- Skin weights: 1/d^4 to bone segments of the nearest bone's ray, 10 smoothing passes; skin+fat 3 extra 2-mm spatial passes (seams match). Twist joints at 25/50/75 % of forearm length turn that fraction of pronation; skin/fat/fascia twist evenly (no radius/ulna split).
- Rigid: bones, footprints, pulleys (A1/A2/C1->P1, A3/A4/C2/C3->P2, A5->P3; thumb A1/oblique->P1, A2->distal), volar plates -> distal bone, nails. Ligaments/capsules: touched bones (<1.5 mm), fixed within 0.7 mm, blended between.
- Cables, 2 mm samples: 24 extrinsic tendons + 3 'common' paths (fused proximal sheets FDS/FDP/EDC) + 145 nerve/vessel splines. Carriers = skin weights, overridden by anchors: insertion, pulleys, carpal tunnel (capitate), Guyon (hamate), extensor retinaculum (radius; ECU ulna), sagittal bands (MC head). Joint arcs: samples near a joint turn by a fraction of its motion (constant moment arm); half-width MCP/CMC1 8, PIP/IP 6, DIP 4, wrist 12 mm.
- Tendon: insertion fixed, constant length -> slides; belly moves f x mean excursion along a straight belly axis, radius x 1/sqrt(axial scale). Lumbricals ride FDP; lateral bands 3 mm volar at PIP 90. Nerves/vessels: ends fixed, taut -> uniform glide, slack -> wave (8 mm wavelength, max 2 mm). Flexor sheaths follow flexor paths, no slide.
- Tendon/nerve meshes are bent along the posed centreline (vertex = arc position + offset), not rebuilt as tubes: keeps real shapes (Camper's chiasm, flat tendons).

## Known problems
- Inside-bone check fails (fist, wrist flex/ext, ulnar 30, pronation 80): 0.2-1.3 % of soft-tissue vertices >1 mm inside bone, mostly FDS/FDP bellies and palmar fat.
- Excursions for E4: fist FDS 17, FDP 21, FPL 9 mm (Wehbe 24/32/27); wrist ext->flex FCU 11 (low), ECU +19 (sign doubtful).
- EIP/EDM bellies stretch a lot (short bellies); EDM tendon mesh ends at MCP5. E1 problems (ulnar dev 25-30, little finger aim) still open.
- E3 notes: skip isBone nodes when loading hand_rig.glb (bone joints share bone names); share one Skeleton across meshes; set bone matrices = rig world matrices (IBMs hold meshopt de-quantisation).

## Next step
- Phase E3: js/rig.js, cables.js, move.js (port tools/rigpose.py + tisspose.py), "Move" tab.

## Key files
- tools/bind_tissues.py, bindskin.py, bindcable.py, cablegeom.py, glbskin.py, zcurves.py, tisspose.py, check_tissues.py; assets/hand_rig.glb, cables.json; data/raw/bind_log.txt
