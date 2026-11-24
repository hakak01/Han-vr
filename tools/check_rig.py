"""Phase E1 checks of assets/rig.json -> docs/MOTION_CHECK.md. Run: python3 tools/check_rig.py"""
import json, os, sys
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
sys.path.insert(0, os.path.dirname(__file__))
import glbio, rigfit as F, rigpose as P, rigrange
from build_rig import Bones, ph, mc

ROOT = P.ROOT
rig, cm = P.load()
J = json.load(open(os.path.join(ROOT, 'assets/joints.json')))
B = Bones(glbio.read(os.path.join(ROOT, 'data/raw/hand_raw.glb')))
NAMES = sorted(set(rig['fixed']) | {j['child'] for j in rig['joints']})
OV = P.Overlap({n: B.all(n) for n in NAMES}, spacing=0.5e-3, reach=8e-3)
W0 = rig['wrist']
VOL, DIS, ULN = -np.array(W0['dorsal']), np.array(W0['distal']), np.array(W0['ulnar'])
OUT = []
FAILS = []


def row(test, value, ok):
    OUT.append('| %s | %s | %s |' % (test, value, 'PASS' if ok else 'FAIL'))
    if not ok:
        FAILS.append(test)


def clamp(pose):
    return {d: float(np.clip(v, *rigrange.limits(J, d, pose))) for d, v in pose.items()}


ZERO = {d: 0.0 for d in rig['dofs'] if not d.startswith('cmc1')}
FIST = dict(ZERO, **{'mcp%d_flex' % i: 90 for i in (2, 3, 4, 5)}, **{'pip%d' % i: 110 for i in (2, 3, 4, 5)},
            **{'dip%d' % i: 80 for i in (2, 3, 4, 5)}, cmc4=10, cmc5=20)
for d in ('mcp1', 'ip1'):
    FIST.pop(d)
EXT = dict(ZERO, **{'mcp%d_flex' % i: -20 for i in (2, 3, 4, 5)}, **{'dip%d' % i: -10 for i in (2, 3, 4, 5)},
           ip1=-15, cmc1_fe=rigrange.limits(J, 'cmc1_fe')[0])
POSES = {'rest (model as built)': {}, 'neutral (all 0)': ZERO, 'full fist': FIST, 'full extension': EXT,
         'wrist flexion 80': {'wrist_fe': 80}, 'wrist extension 70': {'wrist_fe': -70},
         'wrist radial 20': {'wrist_ru': -20}, 'wrist ulnar 30': {'wrist_ru': 30},
         'pronation 80': {'pronation': 80}, 'supination 80': {'pronation': -80},
         'dart-thrower: ext 35 + radial 15 (off grid)': {'wrist_fe': -35, 'wrist_ru': -15},
         'dart-thrower: flex 45 + ulnar 25 (off grid)': {'wrist_fe': 45, 'wrist_ru': 25}}


def pulp(i):
    """Pulp point and pulp normal (rest coords) of the distal phalanx: direction the tip moves in flexion."""
    jid = 'ip1' if i == 1 else 'dip%d' % i
    ax = next(j for j in rig['joints'] if j['id'] == jid)['axes'][0]
    v = B.verts(ph('Distal', i))
    c = np.array(ax['center'])
    tip = np.array(rig['landmarks']['fingertip'][str(i)])
    n = F.unit(np.cross(ax['axis'], tip - c))
    L = F.unit(tip - c)
    s = (v - c) @ L
    far = v[s > 0.6 * s.max()]
    return far[np.argmax(far @ n)], n, ph('Distal', i)


PULP = {i: pulp(i) for i in (1, 2, 3, 4, 5)}
THUMB = [mc(1), ph('Proximal', 1), ph('Distal', 1)]
FING = {f: [mc(f), ph('Proximal', f), ph('Middle', f), ph('Distal', f)] for f in (2, 3, 4, 5)}


def posed_pulp(W, i):
    p, n, b = PULP[i]
    return P.apply(W[b], p[None])[0], W[b][:3, :3] @ n


def opposition(f):
    """Thumb pulp to finger f pulp: optimise thumb + finger dofs within range."""
    ds = ['cmc1_fe', 'cmc1_aa', 'mcp1', 'ip1', 'mcp%d_flex' % f, 'mcp%d_abd' % f, 'pip%d' % f, 'dip%d' % f]
    if f >= 4:
        ds.append('cmc%d' % f)

    def pose_of(x):
        return clamp(dict(ZERO, **dict(zip(ds, x))))

    def cost(x):
        W = P.solve(rig, cm, pose_of(x))
        pt, nt = posed_pulp(W, 1)
        pf, nf = posed_pulp(W, f)
        hit = max(OV.depth(a, W[a], b, W[b]) for a in THUMB for b in FING[f])
        return ((np.linalg.norm(pt - pf) - 8e-3) * 1e3) ** 2 + 30 * (1 + nt @ nf) ** 2 + (hit * 2e3) ** 2
    best = None
    for s in ((10, 10, 20, 20, 40, 0, 40, 20), (20, 30, 30, 30, 60, 0, 60, 30), (-10, 20, 10, 10, 30, 0, 30, 20)):
        x0 = list(s) + ([5] if f >= 4 else [])
        r = minimize(cost, x0, method='Powell', options=dict(maxiter=4000, xtol=0.1, ftol=1e-4))
        if best is None or r.fun < best.fun:
            best = r
    pose = pose_of(best.x)
    W = P.solve(rig, cm, pose)
    pt, nt = posed_pulp(W, 1)
    pf, nf = posed_pulp(W, f)
    return pose, np.linalg.norm(pt - pf) * 1e3, np.degrees(np.arccos(np.clip(nt @ nf, -1, 1)))


def rel_rotvec(W, b):
    R = (np.linalg.inv(W['Radius']) @ W[b])[:3, :3]
    return np.degrees(Rotation.from_matrix(R).as_rotvec())


def main():
    W = P.solve(rig, cm, {})
    base = OV.all_pairs(W, NAMES)
    OUT.append('## Bone-bone overlap (limit 0.5 mm beyond the rest model)\n')
    pre = {k: v for k, v in base.items() if v > 0.5e-3}
    OUT.append('Rest model already overlapping > 0.5 mm: ' + (', '.join(
        '%s/%s %.2f mm' % (a.split(' ')[0], b.split(' ')[0], d * 1e3) for (a, b), d in pre.items()) or 'none') + '\n')
    OUT.append('| pose | worst pairs (mm) | result |\n|---|---|---|')
    for k in ('opposition to little', 'tip pinch index', 'tip pinch middle'):
        f = {'opposition to little': 5, 'tip pinch index': 2, 'tip pinch middle': 3}[k]
        pose, dist, ang = opposition(f)
        POSES[k] = pose
        OPP[k] = (dist, ang, pose)
    for name, pose in POSES.items():
        Wp = P.solve(rig, cm, clamp(pose))
        ov = OV.all_pairs(Wp, NAMES)
        ex = {k: v - base.get(k, 0) for k, v in ov.items() if v - base.get(k, 0) > 0.05e-3}
        worst = sorted(ex.items(), key=lambda t: -t[1])[:3]
        txt = ', '.join('%s/%s %.1f' % (a.replace(' bone', '')[:22], b.replace(' bone', '')[:22], d * 1e3)
                        for (a, b), d in worst) or '-'
        row(name, txt, not worst or worst[0][1] < 0.5e-3)
    hand_checks()
    open(os.path.join(ROOT, 'docs/MOTION_CHECK.md'), 'w').write(
        '# MOTION_CHECK (Phase E1 rig)\n\nGenerated by tools/check_rig.py on assets/rig.json + assets/joints.json.\n\n'
        + '\n'.join(OUT) + '\n\nFailed: %s\n' % (', '.join(FAILS) or 'none'))
    print('\n'.join(OUT[-30:]))
    print('FAILED:', FAILS)


OPP = {}


def hand_checks():
    OUT.append('\n## Hand and wrist checks\n\n| test | model | result |\n|---|---|---|')
    W = P.solve(rig, cm, clamp(FIST))
    nails = [-(W[ph('Distal', i)][:3, :3] @ PULP[i][1]) for i in (2, 3, 4, 5)]
    ul = W['Radius'][:3, :3] @ ULN
    roll = [np.degrees(np.arcsin(np.clip(n @ ul, -1, 1))) for n in nails]   # nail tilt toward ulnar
    rel = [r - roll[1] for r in roll]
    row('fist: nails parallel = nail roll vs middle finger within 15 deg (index, ring, little)',
        ', '.join('%+.0f' % rel[k] for k in (0, 2, 3)), max(abs(x) for x in rel) <= 15)
    dips = [P.apply(W[ph('Middle', i)], np.array(next(j for j in rig['joints'] if j['id'] == 'dip%d' % i)
                                                  ['axes'][0]['center'])[None])[0] for i in (2, 3, 4, 5)]
    order = [d @ (W['Radius'][:3, :3] @ ULN) for d in dips]
    gaps = [(order[k + 1] - order[k]) * 1e3 for k in range(3)]
    row('fist: no finger crossing (DIP order index->little, gaps mm)', ', '.join('%.1f' % g for g in gaps),
        all(g > 0 for g in gaps))
    # fingers flexed into the palm (MCP 90, PIP 90): middle phalanx points to the scaphoid tubercle
    pose = clamp(dict(ZERO, **{'mcp%d_flex' % i: 90 for i in (2, 3, 4, 5)}, **{'pip%d' % i: 90 for i in (2, 3, 4, 5)}))
    W = P.solve(rig, cm, pose)
    sv = B.verts('Scaphoid bone')
    tub = P.apply(W['Scaphoid bone'], sv[np.argmax(sv @ F.unit(VOL + 0.5 * DIS - 0.3 * ULN))][None])[0]
    nrm = W['Radius'][:3, :3] @ VOL
    angs = []
    for i in (2, 3, 4, 5):
        cen = {j['id']: np.array(j['axes'][0]['center']) for j in rig['joints'] if j.get('axes')}
        pip = P.apply(W[ph('Middle', i)], cen['pip%d' % i][None])[0]
        dip = P.apply(W[ph('Middle', i)], cen['dip%d' % i][None])[0]
        a, b = dip - pip, tub - pip
        a, b = a - (a @ nrm) * nrm, b - (b @ nrm) * nrm
        angs.append(np.degrees(np.arccos(np.clip(F.unit(a) @ F.unit(b), -1, 1))))
    row('fingers flexed (MCP 90, PIP 90) point to scaphoid tubercle (<= 15 deg, index..little)',
        ', '.join('%.0f' % a for a in angs), max(angs) <= 15)
    # wrist: carpal rotations measured from the posed bones (relative to the radius)
    flex, uln = np.array(W0['flex_axis']), np.array(W0['ulnar_axis'])
    W = P.solve(rig, cm, {'wrist_fe': 80})
    fe = {b: rel_rotvec(W, b) @ flex for b in ('Capitate bone', 'Scaphoid bone', 'Lunate bone')}
    row('wrist flexion 80: scaphoid, lunate flex less than capitate', 'cap %.0f, sca %.0f, lun %.0f deg' % tuple(fe.values()),
        fe['Scaphoid bone'] < fe['Capitate bone'] and fe['Lunate bone'] < fe['Capitate bone'])
    for ru, sign, word in ((-20, 1, 'radial deviation 20 flexes'), (30, -1, 'ulnar deviation 30 extends')):
        W = P.solve(rig, cm, {'wrist_ru': ru})
        v = [rel_rotvec(W, b) @ flex for b in ('Scaphoid bone', 'Lunate bone', 'Triquetrum bone')]
        row(word + ' the proximal row (sca, lun, trq FE deg)', ', '.join('%.0f' % x for x in v), all(sign * x > 0 for x in v))
    dr = ['Capitate bone', 'Hamate bone', 'Trapezoid bone', 'Trapezium bone']
    worst = 0
    for fe_ in (-70, -35, 0, 40, 80):
        for ru in (-20, 0, 30):
            W = P.solve(rig, cm, {'wrist_fe': fe_, 'wrist_ru': ru})
            for a in dr[1:]:
                R = np.linalg.inv(W['Capitate bone'])[:3, :3] @ W[a][:3, :3]
                worst = max(worst, np.degrees(np.linalg.norm(Rotation.from_matrix(R).as_rotvec())))
    row('distal row moves with capitate (max relative rotation, grid of 15 wrist poses; plan: <= 3 deg)', '%.1f deg' % worst, worst <= 3)
    for d, val, axis in (('wrist_fe', 30, 'flexion'), ('wrist_ru', 20, 'ulnar dev.')):
        W = P.solve(rig, cm, {d: val})
        M = np.linalg.inv(W['Radius']) @ W['Capitate bone']
        R, t = M[:3, :3], M[:3, 3]
        n = F.unit(Rotation.from_matrix(R).as_rotvec())
        p = np.linalg.lstsq(np.eye(3) - R, t - (t @ n) * n, rcond=None)[0]
        c = np.array(W0['bones']['Capitate bone']['centroid'])
        q = p - c
        q = q - (q @ n) * n
        loc = (q @ DIS * 1e3, q @ ULN * 1e3, -(q @ VOL) * 1e3)
        row('sign check: capitate rotation centre in %s 30/20 lies proximal (distal, ulnar, dorsal mm from centroid)' % axis,
            '%.1f, %.1f, %.1f' % loc, loc[0] < 0)
    for k, (dist, ang, pose) in OPP.items():
        ok = ang >= 135 and dist < 12
        row('%s: pulp gap (mm) and angle between pulp normals (>=135 = facing)' % k, '%.1f mm, %.0f deg (CMC1 FE %.0f AA %.0f, MCP %.0f, IP %.0f)' % (
            dist, ang, pose['cmc1_fe'], pose['cmc1_aa'], pose['mcp1'], pose['ip1']), ok)


if __name__ == '__main__':
    main()
