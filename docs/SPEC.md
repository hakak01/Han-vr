I want a realistic 3D model of the NORMAL RIGHT hand, wrist and forearm for Meta Quest 3 (browser WebXR). I am an orthopaedic/hand surgeon with no programming experience: explain anything you need from me in simple English. Keep token use minimal.

STEP 0: Save this whole message unchanged as docs/SPEC.md. Create CLAUDE.md containing only the WORKING RULES below. Create docs/PROGRESS.md (max 30 lines). Then do ONLY Phase A, stop, and report in 10 lines or fewer.

WORKING RULES
1. Never open, cat or print binary/mesh/image files (.blend .glb .stl .png .zip). Inspect them only with scripts that print 20 lines or fewer.
2. Pipe long output through `tail -20`; use -q for pip/npm/git.
3. Keep each source file under ~300 lines. Edit files; don't rewrite whole files.
4. No frameworks, bundlers or test suites. Plain HTML + ES modules; three.js from a CDN via importmap.
5. Don't web-search unless something is blocked; the anatomy rules are in docs/SPEC.md.
6. Raw downloads go in data/raw/ (git-ignored). Commit only code and final assets/. Keep assets/ under 25 MB.
7. One phase per session. At the end: update docs/PROGRESS.md, commit, push to main, and tell me exactly what to test.
8. If a download is blocked, stop and give me the exact URL; I will upload the file to data/upload/.

SOURCES (one body, already aligned)
- Z-Anatomy (CC BY-SA 4.0): github.com/Z-Anatomy/The-blend, file Z-Anatomy.zip (87 MB) → Z-Anatomy/Startup.blend (307 MB). Read it headless with Blender's Python module: `pip install bpy` (bpy 5.x wheels are for Python 3.13; if pip finds no wheel, install a matching Python with uv and retry).
- BodyParts3D v3.0 STL mirror: github.com/Kevin-Mattheus-Moerman/BodyParts3D. Meshes are in assets/BodyParts3D_data/stl/FMA*.stl (millimetres); names are in assets/BodyParts3D_data/parts_list_e.txt (e.g. "FMA24446 right capitate"). Clone with --depth 1 --filter=blob:none --no-checkout, list names with `git ls-tree -r --name-only`, then check out only the files you need. NEVER run `git ls-tree -l` (it downloads all 887 MB).
- Write ATTRIBUTION.md: Z-Anatomy (CC BY-SA 4.0); "BodyParts3D, © The Database Center for Life Science, CC BY 4.0". Licence for the repo's assets: CC BY-SA 4.0.

FACTS ALREADY CHECKED
- Startup.blend: ~7,200 objects; metres; Z up. Right-side names end in ".r". Muscle origin/insertion footprints end in ".or", ".er", ".o1r", ".e2r" etc. (collection "2: Muscular insertions").
- Collections: "1: Skeletal system", "2: Muscular insertions", "3: Joints", "4: Muscular system", "5: Cardiovascular system", "7: Nervous system & Sense organs", "9: Regions of human body" (skin regions such as "Palm.r" and "Dorsum of hand.r", with a Solidify modifier).
- Nerves and many vessels are CURVE objects. Convert them to meshes with lower curve/bevel resolution (digital vessels must not be ~30k triangles each).
- Bones have material slots "Bone…" + "Cartilage"; muscles have their own slot + "Tendon". Keep these slot names as glTF material names; the viewer uses them to draw cartilage and tendon differently.
- Selecting by bounding box alone wrongly picks thigh structures beside the hand (e.g. "Iliotibial tract"). Select by Z-Anatomy hierarchy (upper-limb groups) + right side, from mid-humerus to fingertips.
- A test export of this region: ~290 objects, ~450k triangles, ~9 MB GLB before compression.
- The Z-Anatomy capitate has 238 vertices; the BodyParts3D capitate has 644. A similarity fit on bone centroids gives scale ≈0.00103, near-identity rotation, and a mean surface distance of ~1 mm, so refine each bone with rigid ICP.

PHASE A: asset pipeline (tools/build_assets.py, headless)
1. Download the sources into data/raw/.
2. Extract the right upper limb from mid-humerus to fingertips: bones, cartilage, joints/ligaments/capsules, muscles + tendons, tendon sheaths, retinacula, fascia, nerves, arteries, veins, skin regions, and origin/insertion footprints.
3. Bones: replace each Z-Anatomy bone with the matching BodyParts3D mesh (match by nearest centroid after the global fit + similar volume). Refine with rigid ICP onto the Z-Anatomy bone, then transfer the Cartilage material to the new faces by nearest face. Log the mean/max error per bone; if the mean error is >1.5 mm, keep the Z-Anatomy bone.
4. Repair: merge duplicate vertices, recalculate normals, and lightly smooth only meshes with fewer than 300 vertices (no shape change >0.3 mm).
5. Export ONE assets/hand.glb with meshopt compression (gltf-transform via npx), with node names = structure names. Also write assets/manifest.json: {name, layer, source, approx:false, triangles}. Layers: skin, fascia_retinacula, muscle, tendon_sheath, ligament_capsule, nerve, artery, vein, bone, attachment.
6. Write docs/GAPS.md: mark each item in REQUIRED STRUCTURES as present / partial / missing. Don't build anything yet.
7. Report: counts per layer, total triangles, GLB size, bone-swap result, number of gaps.

PHASE B: WebXR viewer (index.html + js/, served by GitHub Pages)
- three.js WebXR on the Quest 3 browser: "Enter VR", "Enter MR (passthrough)", and an in-headset "Exit" button. Orbit controls on phone/desktop.
- Realistic look with procedural shaders (no texture downloads). Bone: ivory, matte, fine porous noise. Cartilage: bluish-white, glossy. Tendon: pearly white, glossy, fine lengthwise fibres. Muscle: deep red, lengthwise striations, slightly wet. Ligament/capsule/retinaculum: dull white fibrous. Nerve: cream-yellow with lengthwise fascicle bands. Artery: red. Vein: dark blue-purple. Arteries and veins slightly translucent. Synovial sheaths: thin, glossy, translucent. Skin: natural tone with a tone slider. Put all colours/roughness values in assets/look.json so I can edit them myself.
- Wrist menu on the left controller: layers on/off, a ghost slider per layer (0–100% opacity), a searchable list of structures, reset view.
- Right-controller ray + trigger selects a structure: highlight + label (name, layer, "approximate – check" if approx). Buttons: hide, ghost, isolate, show all, focus (brings the structure in front of me).
- One-hand grab moves/rotates the model; two-hand grab zooms 0.5×–20×.
- Movable clipping plane (axial / sagittal / coronal) for MRI-like cross-sections.
- Toggle origin (red) / insertion (blue) footprints.
- Steady 72 fps on Quest 3, a loading bar, and loading in under 10 s on Wi-Fi.

PHASE C: build missing structures (tools/build_gaps.py → assets/gaps.glb, approx:true)
Build each missing or partial item from docs/GAPS.md as smooth geometry fitted to the existing bones and tendons:
- Finger flexor pulleys (Doyle, J Hand Surg Am 1988): A1 over the MCP volar plate (~8 mm wide), A2 on the proximal part of the proximal phalanx (~17 mm), A3 over the PIP (~3 mm), A4 on the middle third of the middle phalanx (~7 mm), A5 over the DIP (~2–4 mm); thin cruciate C1–C3 between them. Thumb: A1 at the MCP, oblique pulley over the proximal phalanx, A2 near the IP. Pulleys wrap the flexor tendons and attach to the bone edges.
- Volar plates of the MCP, PIP and DIP joints.
- Finger extensor mechanism: sagittal bands, central slip to the dorsal base of the middle phalanx, lateral bands joining into the terminal tendon at the distal phalanx base, triangular ligament, transverse and oblique retinacular ligaments.
- Long and short radiolunate ligaments (palmar, radius to lunate; Berger 2001), if missing.
- Recurrent motor branch of the median nerve: from the radial side of the median nerve at the distal edge of the flexor retinaculum, curving back to the thenar muscles (extraligamentous, the commonest type; Lanz 1977).
- Roof of Guyon's canal (volar carpal ligament) over the ulnar nerve and artery, from the pisiform side to the hook-of-hamate side.
- Subcutaneous fat: fill the space between the skin and the deep fascia/retinacula, with a yellow lobulated look.

REQUIRED STRUCTURES (for docs/GAPS.md)
- Bones: all 27 hand bones + radius and ulna, with articular cartilage.
- Muscles and tendons: all forearm and intrinsic hand muscles with their tendons; FDS decussation (Camper's chiasm).
- Retinacula and fascia: flexor and extensor retinacula with the 6 dorsal compartments; finger flexor sheaths and pulleys; palmar aponeurosis.
- Joints and ligaments: volar plates; MCP/PIP/DIP collateral ligaments; extensor mechanism; scapholunate and lunotriquetral ligaments; palmar and dorsal extrinsic wrist ligaments; TFCC (disc, radioulnar ligaments, ulnocarpal ligaments, ECU subsheath).
- Nerves: median, ulnar and radial nerves; palmar cutaneous branch of median; recurrent motor branch; superficial radial nerve; dorsal branch of ulnar nerve; common and proper digital nerves; deep branch of ulnar nerve; AIN; PIN.
- Arteries: radial and ulnar arteries; superficial and deep palmar arches; common and proper digital arteries.
- Veins: cephalic, basilic, dorsal venous network.
- Other: Guyon's canal; skin; subcutaneous fat.

PHASE D: I will test in the headset and send a numbered list of corrections. Fix only those.
