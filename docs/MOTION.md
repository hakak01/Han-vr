Read CLAUDE.md and docs/PROGRESS.md. This message is a NEW plan: realistic joint movement. STEP 0: save this whole message unchanged as docs/MOTION.md and add one line to CLAUDE.md: "Movement plan: docs/MOTION.md (Phases E1-E4)". Then do ONLY Phase E1, stop, and report in 10 lines or fewer.

GOAL
Move the right forearm, wrist, thumb and fingers through their real ranges, with every tissue moving as in a living hand: tendons glide through pulleys and retinacula, muscle bellies shorten and bulge, ligaments tighten and slacken, nerves and vessels glide and fold, skin and fat follow. No physics engine or FEM (too heavy for Quest browser). Method: rigid bones on anatomical joint axes + skinning + sliding "cables". Keep 72 fps on Quest 3 in Present, Realistic and Cadaver views.

FACTS ALREADY CHECKED
- hand.glb (294 parts) and gaps.glb (75, approx) are static, anatomical position; node names = structure names; layers in assets/manifest.json.
- Some meshes hold several digits ("Flexor digitorum profundus", "Extensor digitorum", "Lumbrical muscles of hand", "Dorsal/Palmar interossei...", "Common palmar digital branches...", "Dorsal digital branches..."). Split per digit (connected components first, else nearest digit ray) before binding.
- Muscles have a muscle slot + a "Tendon" slot: use it to separate belly from tendon.
- Nerves and many vessels were Z-Anatomy CURVE objects (tools/zsel.py). Take centrelines + radii from those curves (re-download per docs/SPEC.md), not from the meshes.
- Carpal kinematics: Akhbari et al., J Orthop Res 2019;37:2661 (in-vivo CT, 116 wrists). Each carpal DOF = p10*RU + p01*FE + p20*RU^2 + p11*RU*FE + p02*FE^2, where FE, RU = capitate rotation relative to the radius (degrees). Outputs: RX = supination-pronation, RY = flexion-extension, RZ = radial-ulnar deviation (helical-axis components phi*n: apply together as one rotation vector about the bone's volume centroid); TX = distal-proximal, TY = radial-ulnar, TZ = volar-dorsal translation of that centroid, scaled by cube root of capitate volume (multiply by cbrt of OUR capitate volume). Radial coordinate system: x = distal radius shaft axis; y = centre of sigmoid notch -> radial styloid tip, projected on distal radius; z = x cross y; origin on distal radius surface. Coefficients = Supplementary Table S1, which I uploaded to data/. If Table S1 is not there, STOP and tell me; never invent coefficients. Copy them to assets/carpal_model.json with the citation, credit it in ATTRIBUTION.md, then git rm the uploaded file. Set carpal RX to 0 (model weak for SP, R^2 < 0.6). Check signs from the README/table, then confirm with the E1 checks.
- Capitate + 3rd metacarpal move as one. Hamate, capitate, trapezoid, trapezium move together within 3 degrees.
- Thumb CMC (Hollister, J Orthop Res 1992): two fixed axes; flexion-extension axis in the trapezium, abduction-adduction axis in the first metacarpal; not perpendicular, not intersecting.

PHASE E1: rig (tools/build_rig.py -> assets/rig.json + assets/joints.json; no viewer changes)
1. Forearm: ulna fixed; radius rotates about the axis from radial head centre to ulnar head fovea. Everything distal follows the radius.
2. Wrist: inputs FE and RU drive the capitate (its own Table S1 translations too). MC2 rides on trapezoid, MC3 on capitate, MC4/MC5 on hamate with extra CMC flexion for palmar cupping (defaults 10 deg ring, 20 deg little). Scaphoid, lunate, triquetrum, trapezium, trapezoid, hamate from Table S1. Pisiform rides on triquetrum.
3. Fingers: fit axes on the Cartilage-material surfaces. PIP, DIP, thumb MCP and IP: hinge = cylinder fit of the head of the proximal bone. Finger MCP: centre = sphere fit of metacarpal head; flexion axis from cylinder fit; ab/adduction axis perpendicular to it through the centre. Thumb CMC: Hollister axes from the saddle's two curvature centres.
4. assets/joints.json, typical adult normal values (I will edit): pronation 80, supination 80; wrist flexion 80, extension 70, radial 20, ulnar 30; MCP 0-90 (hyperext 20), ab/adduction +/-20 at 0 deg shrinking to +/-5 at 90 deg flexion; PIP 0-110; DIP 0-80 (hyperext 10); thumb MCP 0-50; IP 0-80 (hyperext 15); thumb CMC limited where bones would collide.
5. tools/check_rig.py on: neutral, full fist, full extension, wrist max FE/RU, full pronation/supination, opposition to little-finger tip. Must pass: bone-bone overlap < 0.5 mm (else adjust axes); in fist nails parallel, no finger crossing, fingers point toward the scaphoid tubercle; in wrist flexion scaphoid and lunate flex less than capitate; radial deviation flexes the proximal row, ulnar deviation extends it; thumb pulp faces index/middle pulp in opposition. Write results to docs/MOTION_CHECK.md.

PHASE E2: tissues follow (tools/bind_tissues.py -> assets/hand_rig.glb + assets/cables.json)
1. Rigid with one bone: cartilage, origin/insertion footprints, pulleys (A1, A2 -> proximal phalanx; A3 -> middle phalanx base; A4 -> middle phalanx; A5 -> distal phalanx base), volar plates (-> distal bone).
2. Skinned (glTF skin, max 4 influences; weights from distance to bone segments of the same digit ray, then 10 smoothing passes): skin, fat, fascia, retinacula, sheaths, intrinsic muscles, capsules. Forearm: 3 twist segments so pronation spreads along the forearm (no candy-wrapper). Ligaments: each end fixed to the bone it touches, middle blended by position between them.
3. Cables (tubes rebuilt at runtime only when the pose changes): every extrinsic tendon (FDS, FDP, FPL, EDC, EIP, EDM, EPL, EPB, APL, ECRL, ECRB, ECU, FCR, FCU, PL) and every nerve, artery and vein. Store centreline, radius profile and anchors (bone, pulley, dorsal compartment, flexor retinaculum). Flexors run through the carpal tunnel and A1-A5 with no bowstringing; FDS splits at Camper's chiasm around FDP; extensors stay in their 6 compartments and centred over MCP heads (sagittal bands). Tendon length is constant: when the path lengthens the tendon slides, and the muscle belly moves by the same excursion (fading to 0 at its origin) and bulges (constant volume: radius x 1/sqrt(axial scale)). Lumbrical origins move with FDP. Lateral bands move volarly with PIP flexion and back dorsally in extension.
4. Nerves and vessels: fixed at the proximal cut and at their distal ends; glide in between. When their path shortens, show slack as a gentle wave; when it lengthens they straighten, then glide.
5. tissues.js and fibres.js must use bind-pose (pre-skinning) positions for texture coordinates, so fibres move with the tissue and don't swim.

PHASE E3: controls (js/rig.js, js/cables.js, js/move.js, each < 300 lines; new wrist-menu tab "Move")
- Slider with degree readout for every joint; Reset.
- Grab: point at a bone, squeeze, drag to rotate the joint just proximal to it; grab a fingertip to bend the whole finger (CCD IK); always within range. Desktop/phone: same tab with touch sliders.
- Animated presets with speed slider and loop: neutral, full fist, hook fist, straight fist, intrinsic-plus, intrinsic-minus (claw), tip pinch, key pinch, Kapandji 1-10, Finkelstein, Phalen, reverse Phalen, dart-thrower's motion, full pronation/supination. Active finger flexion uses DIP = 2/3 PIP (editable).
- Tenodesis toggle: wrist extension flexes the fingers, wrist flexion opens them, through the constant tendon lengths.
- "Mirror my hand": request WebXR 'hand-tracking' as optional feature; when I put the controllers down, convert my right hand's joint poses to joint angles, clamp to range, smooth (~100 ms), apply.
- Teaching aids: glide markers (thin rings every 5 mm on tendons and median/ulnar nerves) with live excursion in mm in the info panel; ligament strain colour (green slack -> red taut, end-to-end length vs neutral); "ghost of neutral" toggle.
- Recompute skinned bounding spheres after each pose change; raycast only on trigger press.

PHASE E4: accuracy test (tools/check_motion.mjs, headless) -> table in docs/MOTION_CHECK.md: test | published | model | pass (within 25%)
- Excursion, wrist neutral, fingers extended -> full fist: FDS 24 mm, FDP 32 mm, FPL 27 mm. With full wrist motion added: FDS 49, FDP 50, FPL 35 mm (Wehbe & Hunter, J Hand Surg Am 1985).
- Median nerve excursion at the wrist: wrist 60 deg extension -> 65 deg flexion = 19.6 mm; fingers hyperextension -> full flexion = 9.7 mm (Wright, JBJS Am 1996).
- MCP collateral ligaments longer at 90 deg flexion than in extension.
- Tenodesis on: wrist 0 -> 60 deg extension flexes the fingers.
- No tissue enters bone by more than 1 mm in any preset; tendons stay inside pulleys.
- fps while animating in all 3 views (desktop estimate; I will test on the Quest).
If a test fails, fix the anchors or axes, not the published numbers.
