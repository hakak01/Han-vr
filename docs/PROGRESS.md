# PROGRESS (updated 2026-11-25)

## Done
- Phases A-C, views, E1 rig, E2/E2b tissue binding + ARMS mechanics, E3 Move tab (git history; plan docs/MOTION.md).
- Phase E4: `node tools/check_motion.mjs` opens the real viewer in headless Chromium (Playwright, local three 0.170.0) and writes "Phase E4" in docs/MOTION_CHECK.md (test | published | model | pass). ~2 min.
- Fixes: median nerve + 10 digital branches glide as one elastic tree (js/nervetree.js); tendon material proximal to the musculotendinous junction shortens with the belly instead of sliding past the origin (cables.js sigma); "Full fist" thumb moved clear of the fingers.

## Decisions
- Excursion pass = within 25 % of ARMS scaled to our hand (user decision 2026-11-24); Wehbe shown with % difference.
- Fist = MCP 90 / PIP 110 / DIP 80 (joints.json max), thumb MCP 50 / IP 80; "total" = wrist ext 70 + extended -> flex 80 + fist.
- Nerve tree: springs k = area / length; area median 10, common digital 3, proper digital 1.8 mm2 (typical sizes, editable in nervetree.js). Fixed at proximal cut and fingertips. Wave only where a piece is compressed.
- Glide marker: nerve point 20 mm proximal to its point nearest the lunate (bed rigid with the radius there).
- Full fist thumb: CMC FE 0, AA 20, MCP 50, IP 30 (was 20/0/50/40; thumb went 1-5 mm into index/middle).
- Tissue-in-bone pass = < 0.2 % of soft vertices newly > 1 mm inside bone (E2 rule); pulley growth < 1.5 mm.
- fps pass = mean JS frame <= 5 ms and 95th pct <= 13.9 ms on this CPU ("Full fist" stepped at 72 Hz).
- Tried and reverted: deep forearm tissue keeping its radius weight (linear-blend collapse made pronation worse, 1.28 -> 2.11 %).

## Results (MOTION_CHECK.md)
- PASS: all 6 excursions (vs ARMS; Wehbe +76/+45/-15/+61/+56/+24 %), median glide 22.0 (Wright 19.6) and 7.9 mm (9.7), tenodesis, 9 Kapandji presets.
- FAIL: MCP collaterals shorter in flexion (8 of 9 measurable sides; one index side has < 8 fixed vertices): Z-Anatomy origin lies on the flexion axis (0 mm dorsal), no cam effect.
- FAIL tissue in bone: 13 presets 0.22-1.54 % (fist 0.78, was 1.86): volar finger tissue at flexion creases 2-3 mm, thumb into palm fat (Finkelstein), pronation 1.28 % (twist skinning). Pulley gap FDS4 at A1 1.5-1.9 mm.
- fps: mean 3.7-4.2 ms, but soft-tissue frames 12-18 ms (realistic 95th pct 16 ms FAIL) -> may drop frames on Quest while animating.

## Next step
- Your choice: (a) move MCP collateral origins dorsal (re-bind) or accept; (b) corrective offsets at finger creases + pronation (dual-quaternion or more twist joints); (c) spread soft-tissue update over frames for Quest.

## Key files
- tools/check_motion.mjs, motion_tests.js (tests), motion_inside.js (bone depth); js/nervetree.js, cables.js, move.js; assets/presets.json.
