"""Phase E2 checks: tissues follow the rig (python reference tisspose.py on data/raw/hand_rig_raw.glb).
Run: python3 tools/check_tissues.py [--glb]   -> section "Phase E2" in docs/MOTION_CHECK.md
--glb also writes posed meshes to data/raw/posed_<pose>.glb for previews."""
import json, os, sys, re
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, os.path.dirname(__file__))
import glbio, tisspose as TP, rigpose as RP, cablegeom as G, bindskin as K, bindcable as C

ROOT = RP.ROOT
OUT = []
R = TP.Rig()
man = {e['name']: e for e in json.load(open(os.path.join(ROOT, 'assets/manifest.json'))) +
       json.load(open(os.path.join(ROOT, 'assets/gaps.json')))}
ZERO = {d: 0.0 for d in R.rig['dofs'] if not d.startswith('cmc1')}
FIST = dict(ZERO, **{'mcp%d_flex' % i: 90 for i in (2, 3, 4, 5)}, **{'pip%d' % i: 110 for i in (2, 3, 4, 5)},
            **{'dip%d' % i: 80 for i in (2, 3, 4, 5)}, cmc4=10, cmc5=20, mcp1=50, ip1=80)
EXT = dict(ZERO, **{'mcp%d_flex' % i: -20 for i in (2, 3, 4, 5)}, **{'dip%d' % i: -10 for i in (2, 3, 4, 5)}, ip1=-15)
POSES = {'neutral': ZERO, 'full fist': FIST, 'full extension': EXT,
         'wrist flexion 80': dict(ZERO, wrist_fe=80), 'wrist extension 70': dict(ZERO, wrist_fe=-70),
         'wrist radial 20': dict(ZERO, wrist_ru=-20), 'wrist ulnar 30': dict(ZERO, wrist_ru=30),
         'pronation 80': dict(ZERO, pronation=80), 'supination 80': dict(ZERO, pronation=-80)}
SOFT = [n for n in R.M if man[n]['layer'] not in ('bone', 'attachment')]
SKIN = [n for n in R.M if man[n]['layer'] == 'skin' and n not in ('Nail plate',)]


def row(test, value, ok=None):
    OUT.append('| %s | %s | %s |' % (test, value, '' if ok is None else ('PASS' if ok else 'FAIL')))


class Inside:
    """Signed depth of points inside each bone (rest-frame surface samples + normals)."""
    def __init__(self):
        self.S = {}
        for b in K.BONES:
            p = R.M[b]
            v = np.vstack([q['v'] for q in p]); f = np.vstack([q['f'] + o for q, o in zip(p, np.cumsum([0] + [len(q['v']) for q in p])[:-1])])
            s = G.densify(v, f, 0.6e-3)
            n = glbio.vertex_normals(v, f)
            tree = cKDTree(v)
            sn = n[tree.query(s)[1]]
            self.S[b] = (cKDTree(s), s, sn, s.mean(0), np.linalg.norm(s - s.mean(0), axis=1).max())

    def depth(self, X, M):
        """Max depth (m) of X inside any posed bone (X in world/pose)."""
        out = np.zeros(len(X))
        for k, b in enumerate(K.BONES):
            tree, s, sn, c, r = self.S[b]
            Mi = np.linalg.inv(M[R.joints.index(b)])
            x = X @ Mi[:3, :3].T + Mi[:3, 3]
            near = np.linalg.norm(x - c, axis=1) < r
            if not near.any():
                continue
            d, i = tree.query(x[near], distance_upper_bound=6e-3)
            ok = np.isfinite(d)
            sd = np.zeros(near.sum())
            sd[ok] = np.einsum('ij,ij->i', x[near][ok] - s[i[ok]], sn[i[ok]])
            dep = np.where(ok & (sd < 0), -sd, 0)
            idx = np.where(near)[0]
            out[idx] = np.maximum(out[idx], dep)
        return out


def soft_arrays(V):
    names, X = [], []
    for n in SOFT:
        for v in V[n]:
            names += [n] * len(v); X.append(v)
    return np.array(names), np.vstack(X)


def edges_ratio(V, names, rest):
    lo, hi = [], []
    for n in names:
        for v, v0, p in zip(V[n], rest[n], R.M[n]):
            e = np.vstack([p['f'][:, [0, 1]], p['f'][:, [1, 2]], p['f'][:, [2, 0]]])
            l0 = np.linalg.norm(v0[e[:, 0]] - v0[e[:, 1]], axis=1)
            l1 = np.linalg.norm(v[e[:, 0]] - v[e[:, 1]], axis=1)
            ok = l0 > 0.3e-3
            r = l1[ok] / l0[ok]
            lo.append(np.percentile(r, 0.5)); hi.append(np.percentile(r, 99.5))
    return min(lo), max(hi)


def pulley_check(V, info, M):
    """Flexor cables: max distance (mm) from each pulley centre to the cable, posed vs neutral (increase)."""
    worst = (0, '')
    for c in R.cab['cables']:
        if c.get('group') != 'flexor' or c.get('common'):
            continue
        for pn, bone in C.pulleys_of(c['digit']):
            if pn not in V:
                continue
            ds = []
            for Vx, Mx in ((V0, M0), (V, M)):
                pc = np.vstack(Vx[pn]).mean(0)
                Pp = R.cable(Mx, c)[0]
                ds.append(np.min(np.linalg.norm(Pp - pc, axis=1)))
            if ds[1] - ds[0] > worst[0]:
                worst = (ds[1] - ds[0], '%s in %s' % (c['name'], pn.split(' of ')[0]))
    return worst


def main():
    global V0, M0
    glb = '--glb' in sys.argv
    I = Inside()
    rest = {n: [p['v'] for p in R.M[n]] for n in R.M}
    V0, info0, M0 = R.pose(ZERO)
    names, X0 = soft_arrays(V0)
    d0 = I.depth(X0, M0)
    OUT.append('\n## Phase E2: tissues follow the bones (tools/check_tissues.py, python reference tools/tisspose.py)\n')
    OUT.append('Soft tissue = all non-bone, non-footprint vertices (%d). "New depth" = depth inside a posed bone minus the '
               'depth already present in the neutral pose (rest model has some tissue touching/inside bone).\n' % len(X0))
    OUT.append('| pose | new depth > 1 mm (vertices, worst structure) | skin edge stretch 0.5-99.5 % | flexor off pulley (mm) | result |\n|---|---|---|---|---|')
    fails = []
    for pn, pose in POSES.items():
        V, info, M = R.pose(pose)
        _, X = soft_arrays(V)
        dd = I.depth(X, M) - d0
        bad = dd > 1e-3
        worst = '-'
        if bad.any():
            u, cnt = np.unique(names[bad], return_counts=True)
            worst = ', '.join('%s %d (%.1f mm)' % (a, b, dd[names == a].max() * 1e3) for a, b in sorted(zip(u, cnt), key=lambda x: -x[1])[:3])
        lo, hi = edges_ratio(V, SKIN, V0)
        pw = pulley_check(V, info, M)
        frac = bad.sum() / len(X)
        ok = frac < 0.002 and pw[0] < 1.5e-3 and lo > 0.4
        if not ok:
            fails.append(pn)
        OUT.append('| %s | %d (%.2f %%) %s | %.2f-%.2f | %.1f %s | %s |' % (pn, bad.sum(), 100 * frac, worst, lo, hi,
                                                                           pw[0] * 1e3, pw[1], 'PASS' if ok else 'FAIL'))
        if glb:
            objs = [(n, [(v, p['f'], p['mat']) for v, p in zip(V[n], R.M[n])]) for n in V]
            glbio.write(os.path.join(ROOT, 'data/raw/posed_%s.glb' % re.sub(r'\W+', '_', pn)), objs)
    OUT.append('\nPass: < 0.2 % of soft-tissue vertices newly > 1 mm inside bone, pulley-to-flexor distance grows < 1.5 mm, '
               'skin edges not compressed below 0.4 (no candy-wrapper).\n')
    # excursions (informative; E4 tests them against published values)
    OUT.append('| excursion (mm, + = tendon pulled proximally / nerve bed longer) | model |\n|---|---|')
    ex = lambda a, b, k: (R.pose(b, names=['Palm'])[1][k]['excursion'] - R.pose(a, names=['Palm'])[1][k]['excursion']) * 1e3
    wf, we = POSES['wrist flexion 80'], POSES['wrist extension 70']
    for k in ('FDS3', 'FDP3', 'FPL1', 'EDC3'):
        OUT.append('| %s: neutral -> full fist | %.1f |' % (k, ex(ZERO, FIST, k)))
    for k in ('FDS3', 'FDP3', 'FPL1', 'FCR', 'FCU', 'ECRB', 'ECU', 'EDC3'):
        OUT.append('| %s: wrist extension 70 -> flexion 80 | %.1f |' % (k, ex(we, wf, k)))
    a, b = R.pose(we, names=['Palm'])[1]['Median nerve'], R.pose(wf, names=['Palm'])[1]['Median nerve']
    OUT.append('| median nerve bed length: wrist extension 70 -> flexion 80 | %.1f (slack wave in flexion %.1f mm) |' % (
        (b['Lp'] - a['Lp']) * 1e3, b.get('wave_mm', 0)))
    OUT.append('\nE2 failed: ' + (', '.join(fails) if fails else 'none'))
    e2b_table(ex)
    path = os.path.join(ROOT, 'docs/MOTION_CHECK.md')
    txt = open(path).read()
    old = txt.split('\n## Phase E2')[0].rstrip('\n')
    tail = ('\n## Phase E3' + txt.split('\n## Phase E3', 1)[1]) if '\n## Phase E3' in txt else ''   # kept
    open(path, 'w').write(old + '\n' + '\n'.join(OUT) + '\n' + tail)
    print('\n'.join(OUT))


def e2b_table(ex):
    """E2b: tendon moment arms (mechanics) vs the ARMS model scaled to our hand; excursions vs ARMS and Wehbe."""
    A = json.load(open(os.path.join(ROOT, 'tools/arms_ma.json')))
    sc = R.cab['mech_scale']
    OUT.append('\n## Phase E2b: tendon moment arms vs the ARMS model (McFarland 2023), scaled to our hand %s\n' % sc)
    OUT.append('Model = excursion change / joint angle over each half of the range; ARMS = mean of its curve over the same '
               'interval x scale. Pass = within 10 %% or 1 mm.\n')
    OUT.append('| tendon | dof | interval (deg) | model mm | ARMS mm | result |\n|---|---|---|---|---|---|')
    tests = [('FDS3', 'mcp3_flex', (0, 45, 90)), ('FDS3', 'pip3', (0, 50, 100)), ('FDP3', 'mcp3_flex', (0, 45, 90)),
             ('FDP3', 'pip3', (0, 50, 100)), ('FDP3', 'dip3', (0, 40, 80)), ('FDP2', 'pip2', (0, 50, 100)),
             ('EDC3', 'mcp3_flex', (0, 45, 90)), ('FPL1', 'mcp1', (0, 25, 50)), ('FPL1', 'ip1', (0, 40, 80)),
             ('FCR', 'wrist_fe', (-60, 0, 60)), ('FCU', 'wrist_fe', (-60, 0, 60)), ('ECRB', 'wrist_fe', (-60, 0, 60)),
             ('ECU', 'wrist_fe', (-60, 0, 60)), ('FCU', 'wrist_ru', (-10, 0, 25)), ('ECU', 'wrist_ru', (-10, 0, 25)),
             ('FDP3', 'wrist_fe', (-60, 0, 60))]
    bad = 0
    for k, d, a in tests:
        rows = np.array(A['moment_arms'][k][d])
        s = sc['wrist'] if d.startswith('wrist') else sc['thumb'] if d in ('mcp1', 'ip1') else sc['digit' + d[3]]
        for i in range(2):
            q = np.linspace(a[i], a[i + 1], 21)
            ref = np.interp(q, rows[:, 0], rows[:, 1]).mean() * s
            mod = ex(dict(ZERO, **{d: a[i]}), dict(ZERO, **{d: a[i + 1]}), k) / np.radians(a[i + 1] - a[i])
            ok = abs(mod - ref) <= max(1.0, 0.1 * abs(ref))
            bad += not ok
            OUT.append('| %s | %s | %d to %d | %.1f | %.1f | %s |' % (k, d, a[i], a[i + 1], mod, ref, 'PASS' if ok else 'FAIL'))
    L = A['length_change_mm']
    OUT.append('\n| excursion (mm) | model | ARMS (unscaled) | Wehbe 1985 in vivo |\n|---|---|---|---|')
    F2 = dict(ZERO, mcp3_flex=90, pip3=100, dip3=80)
    for k, w in (('FDS3', 24), ('FDP3', 32)):
        OUT.append('| %s fist (MCP 90, PIP 100, DIP 80), wrist 0 | %.1f | %.1f | %d |' % (k, ex(ZERO, F2, k), L[k]['fist'], w))
    OUT.append('| FPL1 (MCP 45, IP 75), wrist 0 | %.1f | %.1f | 27 |' % (ex(ZERO, dict(ZERO, mcp1=45, ip1=75), 'FPL1'), L['FPL1']['fist']))
    for k in ('FDS3', 'FDP3', 'FCR', 'FCU', 'ECRB', 'ECU'):
        OUT.append('| %s wrist ext 60 -> flex 65 | %.1f | %.1f | - |' % (
            k, ex(dict(ZERO, wrist_fe=-60), dict(ZERO, wrist_fe=65), k), L[k]['wrist_ext60_to_flex65']))
    OUT.append('\nE2b moment-arm rows failed: %d' % bad)


if __name__ == '__main__':
    main()
