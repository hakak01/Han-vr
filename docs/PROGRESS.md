# PROGRESS (updated 2026-11-24)

## Done
- Phases A-C, views, E1 rig, E2 tissue binding (git history; plan in docs/MOTION.md).
- Phase E2b (accuracy fix): tendon mechanics from the ARMS hand-wrist model (McFarland 2023) via OpenSim 4.6 (tools/arms_ma.py -> tools/arms_ma.json, 24 tendons). Joint "arcs" replaced by wider zones posed in polar coordinates (tisspose.zone): no tendon fold-backs (worst bend per pose 145-178 deg -> <= 101 deg). Excursion = integral of scaled ARMS moment arms; all 32 moment-arm checks pass (docs/MOTION_CHECK.md, E2b). ECU now extensor (-4 mm FE, +20-24 mm ulnar), FCU largest flexor (12-15 mm).

## Decisions
- Mechanics separate from visuals: our mesh tendons sit ~2 mm too close to the PIP/DIP axes (thin BodyParts3D phalanges, FDP centre 4.3 mm from PIP axis, bone surface 2.8 mm), so moment arms come from ARMS, not from the drawn path.
- Scale ARMS by segment length ours/ARMS: wrist 0.911, thumb 1.024, digits 2-5 1.027/1.119/1.138/1.038. ARMS r clamped beyond its range (e.g. PIP > 100, wrist > 70).
- Zone half-width = clip(k x tendon height, Z0, Zmax); k = tan(max/2 + 10 deg): MCP 1.43 (Zmax 16 mm), PIP 2.14 (11), DIP 1.19 (7), wrist 1.19 (20), CMC1 0.84 (12).
- Tendon material map: insertion fixed; outside zones rest length kept; a zone holds rest length - E (shortfall carried proximally). Belly slide = whole-unit shortening (sum of E).
- Not used from ARMS: MCP abduction, CMC1 (sign convention vs our rig unchecked) -> visual path there; pronation effect on ECU (Horii 1993) not modelled.

## Known problems
- Fist excursions now FDS3 40.7, FDP3 45.0, FPL1 21.1 mm (ARMS 36.7/40.5/20.6) vs Wehbe in vivo 24/32/27: E4 (25 %) fails for FDS/FDP. Needs user decision: keep ARMS or calibrate to Wehbe.
- Inside-bone (fist 1.13 %, wrist flex 0.60 %, pronation 1.25 %): mostly FDS/FDP bellies, now sliding further; fist pulley gap 1.5 mm (limit < 1.5). PL end kinks in pronation (E2 skin weights).
- E1 problems (ulnar dev 30 overlap, little finger aim) open.

## Next step
- User: decide excursion target (above). Then Phase E3: js/rig.js, cables.js, move.js; port tisspose.zone + material map + mech tables (cables.json: zones, mech, mech_scale).

## Key files
- tools/arms_ma.py (needs pip opensim in a venv), arms_ma.json, bindcable.py, tisspose.py, check_tissues.py; assets/cables.json; ARMS files only in data/raw/arms (not committed).
