"""Phase E2b: moment-arm curves of the extrinsic tendons from the ARMS hand-wrist model
(McFarland et al., IEEE TBME 2023;70:1424, OpenSim). Needs the `opensim` pip package (4.6) in a separate venv:
    python3 -m venv ENV && ENV/bin/pip install opensim numpy
    ENV/bin/python tools/arms_ma.py data/raw/arms/Hand_Wrist_Model.osim
-> tools/arms_ma.json: {our tendon: {our dof: [[deg, moment arm mm], ...]}}, + = flexion / ulnar / abduction
moment (OpenSim sign: -dL/dq). Also muscle-tendon length changes for the E4 excursion poses."""
import json, os, sys
import numpy as np
import opensim as osim

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FING = {'I': 2, 'M': 3, 'R': 4, 'L': 5}
DIG_DOFS = {'mcp%d_flex': '%dmcp_flexion', 'mcp%d_abd': '%dmcp_abduction', 'pip%d': '%dpm_flexion', 'dip%d': '%dmd_flexion'}
WRIST = {'wrist_fe': 'flexion', 'wrist_ru': 'deviation'}
THUMB = {'cmc1_fe': 'cmc_flexion', 'cmc1_aa': 'cmc_abduction', 'mcp1': 'mp_flexion', 'ip1': 'ip_flexion'}


def ours(name):
    """ARMS muscle -> (our tendon cable name, digit or None)"""
    for k in ('FDS', 'FDP', 'EDC'):
        if name.startswith(k) and len(name) == 4:
            return k + str(FING[name[3]]), FING[name[3]]
    return {'EIP': ('EIP2', 2), 'EDM': ('EDM5', 5), 'FPL': ('FPL1', 1), 'EPL': ('EPL1', 1), 'EPB': ('EPB1', 1),
            'APL': ('APL', 1), 'ECRL': ('ECRL', None), 'ECRB': ('ECRB', None), 'ECU': ('ECU', None),
            'FCR': ('FCR', None), 'FCU': ('FCU', None), 'PL': ('PL', None)}.get(name, (None, None))


def dofs_for(digit):
    d = dict(WRIST)
    if digit == 1:
        d.update(THUMB)
    elif digit:
        d.update({k % digit: v % digit for k, v in DIG_DOFS.items()})
    return d


def main(path):
    osim.Logger.setLevelString('error')
    m = osim.Model(path)
    s = m.initSystem()
    C = {c.getName(): c for c in m.getCoordinateSet()}
    mus = {x.getName(): x for x in m.getMuscles()}

    def zero():
        for c in C.values():
            c.setValue(s, 0.0, False)
        m.assemble(s)

    out, lengths = {}, {}
    for an, mu in mus.items():
        on, dig = ours(an)
        if not on:
            continue
        out[on] = {}
        for od, cn in dofs_for(dig).items():
            c = C[cn]
            lo, hi = c.getRangeMin(), c.getRangeMax()
            rows = []
            for q in np.linspace(lo, hi, 9):
                zero()
                c.setValue(s, float(q), True)
                rows.append([round(float(np.degrees(q)), 1), round(1e3 * mu.computeMomentArm(s, c), 2)])
            if max(abs(r[1]) for r in rows) > 0.2:
                out[on][od] = rows
    # muscle-tendon length changes (mm, + = shortening = tendon pulled proximally) for the E4 poses
    def length(name, pose):
        zero()
        for cn, v in pose.items():
            C[cn].setValue(s, float(np.radians(v)), False)
        m.assemble(s)
        return mus[name].getLength(s)
    for an in mus:
        on, dig = ours(an)
        if not on or dig in (None,):
            continue
        if dig == 1:
            fist = {'mp_flexion': 45, 'ip_flexion': 75}
        else:
            fist = {'%dmcp_flexion' % dig: 90, '%dpm_flexion' % dig: 100, '%dmd_flexion' % dig: 80}
        L0 = length(an, {})
        lengths[on] = dict(fist=round(1e3 * (L0 - length(an, fist)), 1),
                           wrist_ext60_to_flex65=round(1e3 * (length(an, {'flexion': -60}) - length(an, {'flexion': 65})), 1))
    for an in ('ECRL', 'ECRB', 'ECU', 'FCR', 'FCU', 'PL'):
        on, _ = ours(an)
        lengths[on] = dict(wrist_ext60_to_flex65=round(1e3 * (length(an, {'flexion': -60}) - length(an, {'flexion': 65})), 1))
    # segment lengths (mm, joint centre to joint centre, neutral) for scaling to our hand
    zero()
    J = {j.getName(): j for j in m.getJointSet()}
    pos = lambda jn: np.array(J[jn].getChildFrame().getPositionInGround(s).to_numpy())
    seg = {'wrist': np.linalg.norm(pos('_3MCP') - pos('radiocarpal')), 'thumb': np.linalg.norm(pos('IP') - pos('MCP'))}
    for d in range(2, 6):
        seg['digit%d' % d] = np.linalg.norm(pos('_%dprox-midph_b' % d) - pos('_%dMCP' % d))
    res = dict(source='ARMS hand-wrist model (McFarland et al. 2023, doi:10.1109/TBME.2022.3217722), '
                      'Hand_Wrist_Model.osim, computed with OpenSim 4.6; moment arms mm, + = flexion/ulnar/abduction',
               segment_mm={k: round(1e3 * float(v), 2) for k, v in seg.items()},
               moment_arms=out, length_change_mm=lengths)
    json.dump(res, open(os.path.join(ROOT, 'tools/arms_ma.json'), 'w'), indent=0)
    print('tendons', len(out), 'written tools/arms_ma.json')


if __name__ == '__main__':
    main(sys.argv[1])
