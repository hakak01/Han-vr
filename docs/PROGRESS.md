# PROGRESS (updated 2026-11-25)

## Done
- Phases A-C, views, E1 rig, E2/E2b tissue binding + ARMS mechanics (git history; plan in docs/MOTION.md).
- Phase E3 (controls): viewer loads assets/hand_rig.glb (skinned, all 369 structures). New wrist-menu tab "Move": sliders with degree readout for all 25 dofs (pages Wrist, Thumb, Index-Mid, Ring-Little), Reset pose, Neutral; 23 animated presets (speed 0.25-2.5x, loop, stop) incl. Kapandji 1-10; tenodesis; grab bones (VR squeeze / desktop press-drag, CCD IK, within range); Mirror my hand (WebXR hand-tracking); glide rings + live excursion in the info panel; ligament strain colour; ghost of neutral. JS poser = python tisspose.py (docs/MOTION_CHECK.md, Phase E3).

## Decisions
- Soft tissue on the GPU skin + per-vertex offset aDelta added before skinning (ShaderChunk patch), so fibre/noise textures keep bind-pose coordinates. Offsets: cable vertices B^-1(target - skinned), bellies/lumbricals bind-space offset.
- Bones every frame; cables + offsets only when bones they depend on moved, and every 1-3 frames while animating (cost 4-11 ms per full update on a 2.1 GHz CPU).
- Skinned bounds: frustum culling off; cheap conservative spheres per pose (rest sphere carried by each skin joint + max offset); exact pick on cached CPU-skinned positions only on click/trigger (60 ms first, 15 ms after).
- Presets in assets/presets.json (tools/make_presets.py, thumb poses optimised on bone meshes, 17 min). Keys: unlisted fingers/thumb = 0, forearm/wrist kept; DIP "link" = 0.667 x PIP (Aids page slider).
- Tenodesis = constant FDP2-5 / FPL length (ARMS tables); resting cascade s = 0.25-0.40 at wrist 0.
- Strain = end-to-end length of fixed-vertex centroids vs neutral, per vertex nearest bone pair (same ray / carpus / MC-MC), colour scale +/-10 %.
- Mirror: fingers from joint positions (palm normal from wrist + index/little MCP), thumb CMC by IK on the metacarpal direction; wrist/forearm not mirrored (WebXR tracks no forearm); 100 ms smoothing.

## Known problems
- Not reachable with E1 thumb CMC ranges/axes: Kapandji 1, 2, 7-10 and thumb-in-palm (gaps 18-58 mm, presets.json "log").
- Median nerve glide wrist ext 60 -> flex 65 = 10.6 mm (Wright 19.6) - E4 test.
- Tendon/sheath vertices 0.2 mm mean (1.4 max) off: 12-bit _CABLE quantisation in hand_rig.glb (meshopt). Extensor hood / palmar structures show through thin skin (also at rest), palm skin opens in a full fist (E2 binding).
- Untested here (no headset): VR squeeze-grab, hand-tracking mirror (finger flexion signs!), Quest fps. Older E1/E2 problems still open.

## Next step
- Phase E4: tools/check_motion.mjs (headless) -> accuracy table in docs/MOTION_CHECK.md.

## Key files
- js/rig.js (pose engine), cables.js (cable posing), tissuemove.js (vertex offsets, bounds, pick), move.js (state, presets, tenodesis), grab.js (grab/IK/mirror), aids.js, movemenu.js; tools/make_presets.py; assets/presets.json.
