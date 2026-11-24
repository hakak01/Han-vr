"""Contact corrections for the rig (used by build_rig.py):
- carpal_corr: Table S1 rotations kept; centroid translations corrected by the smallest pushes that
  remove bone overlap (> rest + TOL), on a grid of wrist FE x RU. Stored in rig['wrist']['corr'].
- refine_*: small shifts of hinge centres / forearm axis that minimise overlap over the joint range."""
import numpy as np
import rigfit as F, rigpose as P

TOL = 0.2e-3
FE_GRID = list(range(-70, 81, 10))
RU_GRID = list(range(-20, 31, 10))
GROUPS = {'Scaphoid bone': [], 'Lunate bone': [], 'Triquetrum bone': ['Pisiform bone'],
          'Capitate bone': ['Hamate bone', 'Trapezoid bone', 'Trapezium bone', 'First metacarpal bone',
                            'Second metacarpal bone', 'Third metacarpal bone', 'Fourth metacarpal bone',
                            'Fifth metacarpal bone']}   # distal row + metacarpals move as one unit
FIXED = ['Radius', 'Ulna']


def group_meshes(B):
    out = {n: B.all(n) for n in FIXED}
    for g, extra in GROUPS.items():
        v, f = B.all(g)
        c = v.mean(0)
        vs, fs, o = [v], [f], len(v)
        for e in extra:                           # carpals whole; only the base of each metacarpal
            ev, ef = B.all(e)
            keep = (np.linalg.norm(ev[ef] - c, axis=2) < 32e-3).all(1) if 'metacarpal' in e else np.ones(len(ef), bool)
            vs.append(ev); fs.append(ef[keep] + o); o += len(ev)
        out[g] = (np.vstack(vs), np.vstack(fs))
    return out


def carpal_corr(B, rig, cm, log):
    rig['wrist']['corr'] = None
    ov = P.Overlap(group_meshes(B), spacing=0.6e-3, reach=10e-3)
    names = FIXED + list(GROUPS)

    cen = {g: np.array(rig['wrist']['bones'][g]['centroid']) for g in GROUPS}
    ROT = ('Scaphoid bone', 'Lunate bone', 'Triquetrum bone')     # proximal row may also turn

    def mats(fe, ru, corr, rot=None):
        M = {n: np.eye(4) for n in FIXED}
        for g in GROUPS:
            m = P.carpal_matrix(rig, cm, g, fe, ru)
            cp = P.apply(m, cen[g][None])[0]
            if rot is not None and g in rot:
                m = P.about(cp, F.rotvec(rot[g])) @ m
            m[:3, 3] += corr[g]
            M[g] = m
        return M
    base = {}
    M0 = mats(rig['wrist']['rest']['wrist_fe'], rig['wrist']['rest']['wrist_ru'], {g: np.zeros(3) for g in GROUPS})
    rgrid = np.zeros((len(FE_GRID), len(RU_GRID), len(ROT), 3))
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            base[(a, b)] = ov.depth(a, M0[a], b, M0[b])
    grid = np.zeros((len(FE_GRID), len(RU_GRID), len(GROUPS), 3))
    worst_after, worst_before, shift = 0.0, 0.0, 0.0
    for j, ru in enumerate(RU_GRID):
        i0 = FE_GRID.index(0)
        chains = [list(range(i0, -1, -1)), list(range(i0 + 1, len(FE_GRID)))]   # from neutral outwards
        for i in chains[0] + chains[1]:
            fe = FE_GRID[i]
            if i in (chains[0][0], chains[1][0]):
                corr = {g: grid[i0, j, k].copy() if i != i0 else np.zeros(3) for k, g in enumerate(GROUPS)}
                rot = {g: rgrid[i0, j, k].copy() if i != i0 else np.zeros(3) for k, g in enumerate(ROT)}
            for it in range(150):
                M = mats(fe, ru, corr, rot)
                delta = {g: np.zeros(3) for g in GROUPS}
                drot = {g: np.zeros(3) for g in ROT}
                worst = 0.0
                for a_i, a in enumerate(names):
                    for b in names[a_i + 1:]:
                        if a in FIXED and b in FIXED:
                            continue
                        v, d, pc = ov.push(b, M[b], a, M[a])      # moves b out of a
                        ex = d - base[(a, b)]
                        if it == 0:
                            worst_before = max(worst_before, ex)
                        if ex <= TOL:
                            continue
                        worst = max(worst, ex)
                        v = v * min(1.0, ex / max(d, 1e-9)) * 1.1
                        for g, sg in ((b, 1), (a, -1)):       # turn a proximal-row bone about its centroid
                            if g in ROT:
                                r = pc - P.apply(M[g], cen[g][None])[0]
                                drot[g] += 0.3 * sg * np.cross(r, v) / max(r @ r, 16e-6)
                        if a in FIXED:
                            delta[b] += 0.7 * v
                        else:
                            delta[b] += 0.4 * v
                            delta[a] -= 0.4 * v
                if worst == 0:
                    break
                for g in GROUPS:
                    corr[g] = corr[g] + delta[g]
                for g in ROT:
                    w = rot[g] + drot[g]
                    lim = np.radians(15)
                    rot[g] = w * min(1.0, lim / max(np.linalg.norm(w), 1e-12))
            worst_after = max(worst_after, worst)
            for k, g in enumerate(ROT):
                rgrid[i, j, k] = rot[g]
            for k, g in enumerate(GROUPS):
                grid[i, j, k] = corr[g]
                shift = max(shift, np.linalg.norm(corr[g]))
    rig['wrist']['corr'] = dict(fe=FE_GRID, ru=RU_GRID, note='translation added to each carpal centroid (m), radius frame',
                                bones={g: np.round(grid[:, :, k], 6).tolist() for k, g in enumerate(GROUPS)},
                                rot={g: np.round(rgrid[:, :, k], 5).tolist() for k, g in enumerate(ROT)})
    rmax = np.degrees(np.linalg.norm(rgrid, axis=3).max())
    log('proximal-row rotation correction: largest %.1f deg (cap 15)' % rmax)
    log('carpal contact correction: overlap beyond rest before %.1f mm, after %.1f mm; largest shift %.1f mm (%d x %d grid)' % (
        worst_before * 1e3, worst_after * 1e3, shift * 1e3, len(FE_GRID), len(RU_GRID)))


def _best_shift(evaluate, plane_u, plane_w, reach, step):
    """Grid-search a centre shift in a plane; evaluate(shift) -> worst overlap (m)."""
    best = (evaluate(np.zeros(3)), 0.0, np.zeros(3))
    r0 = best[0]
    for a in np.arange(-reach, reach + 1e-9, step):
        for b in np.arange(-reach, reach + 1e-9, step):
            s = a * plane_u + b * plane_w
            v = evaluate(s)
            key = (v + 0.05 * np.linalg.norm(s), np.linalg.norm(s))
            if key < (best[0] + 0.05 * np.linalg.norm(best[2]), best[1]):
                best = (v, np.linalg.norm(s), s)
    return r0, best[0], best[2]


def refine_hinges(B, rig, cm, J, log):
    """Shift each finger/thumb hinge centre (<= 2 mm, normal to its axis) to minimise parent-child
    overlap over the flexion range."""
    import rigrange
    for j in rig['joints']:
        if j['type'] != 'hinge' or not j.get('axes') or j['id'] in ('forearm', 'cmc1'):
            continue
        ax = j['axes'][0]
        d = ax['dof']
        lo, hi = rigrange.limits(J, d)
        angs = [lo, 0.5 * (lo + hi), hi]
        ov = P.Overlap({n: B.all(n) for n in (j['parent'], j['child'])}, spacing=0.5e-3)
        c0 = np.array(ax['center'])
        u, w = F._perp(np.array(ax['axis']))
        rest = rig['dofs'][d]['rest']
        base = ov.depth(j['parent'], np.eye(4), j['child'], np.eye(4))

        def ev(s):
            ax['center'] = (c0 + s).tolist()
            worst = 0.0
            for a in angs:
                M = P.joint_matrix(rig, cm, j, {d: a})
                worst = max(worst, ov.depth(j['parent'], np.eye(4), j['child'], M) - base)
            return worst
        before, after, s = _best_shift(ev, u, w, 2e-3, 0.5e-3)
        ax['center'] = np.round(c0 + s, 6).tolist()
        for extra in j['axes'][1:]:
            extra['center'] = ax['center']
        if np.linalg.norm(s) > 0:
            log('  %-8s centre shifted %.1f mm: overlap over range %.2f -> %.2f mm' % (
                j['id'], np.linalg.norm(s) * 1e3, before * 1e3, after * 1e3))


def refine_forearm(B, rig, cm, log):
    """Shift the distal (ulnar fovea) end of the forearm axis (<= 3 mm) to minimise overlap in
    pronation/supination."""
    j = next(j for j in rig['joints'] if j['id'] == 'forearm')
    ax = j['axes'][0]
    names = ['Humerus', 'Ulna', 'Radius', 'Lunate bone', 'Triquetrum bone', 'Scaphoid bone']
    ov = P.Overlap({n: B.all(n) for n in names}, spacing=0.6e-3)
    W0 = P.solve(rig, cm, {})
    pairs = [('Ulna', n) for n in names[2:]] + [('Humerus', 'Radius')]
    base = {p: ov.depth(p[0], W0[p[0]], p[1], W0[p[1]]) for p in pairs}
    c = np.array(ax['center'])
    a = np.array(ax['axis'])
    fov = np.array(rig['landmarks']['fovea'])
    u, w = F._perp(a)

    def ev(s):
        na = F.unit(fov + s - c)
        ax['axis'] = (na if na @ a > 0 else -na).tolist()
        worst = 0.0
        for p in (80, 40, -40, -80):
            W = P.solve(rig, cm, {'pronation': p})
            for x, y in pairs:
                worst = max(worst, ov.depth(x, W[x], y, W[y]) - base[(x, y)])
        return worst
    before, after, s = _best_shift(ev, u, w, 3e-3, 1e-3)
    na = F.unit(fov + s - c)
    ax['axis'] = np.round(na if na @ a > 0 else -na, 6).tolist()
    rig['landmarks']['fovea_used'] = np.round(fov + s, 6).tolist()
    log('forearm axis: distal end shifted %.1f mm; overlap in pro/supination %.2f -> %.2f mm' % (
        np.linalg.norm(s) * 1e3, before * 1e3, after * 1e3))


def aim_fingers(B, rig, cm, log):
    """Turn each finger's MCP axes about the palm normal and its PIP axis about the proximal phalanx's
    dorsal-volar line (each <= 15 deg) so the flexed finger (MCP 90, PIP 90) points at the scaphoid
    tubercle without hitting its neighbours in a full fist (all other dofs at anatomical 0)."""
    W0 = rig['wrist']
    vol, dis, uln = -np.array(W0['dorsal']), np.array(W0['distal']), np.array(W0['ulnar'])
    ph = lambda k, i: '%s phalanx of %s finger of hand' % (k, ['first', 'second', 'third', 'fourth', 'fifth'][i - 1])
    fing = {i: [ph(k, i) for k in ('Proximal', 'Middle', 'Distal')] for i in (2, 3, 4, 5)}
    ov = P.Overlap({b: B.all(b) for i in fing for b in fing[i]}, spacing=0.6e-3)
    sv = B.verts('Scaphoid bone')
    tub = sv[np.argmax(sv @ F.unit(vol + 0.5 * dis - 0.3 * uln))]
    jd = {j['id']: j for j in rig['joints']}
    zero = {d: 0.0 for d in rig['dofs'] if not d.startswith('cmc1')}
    fist = dict(zero, **{'mcp%d_flex' % i: 90 for i in fing}, **{'pip%d' % i: 110 for i in fing},
                **{'dip%d' % i: 80 for i in fing}, cmc4=10, cmc5=20)
    flexed = dict(zero, **{'mcp%d_flex' % i: 90 for i in fing}, **{'pip%d' % i: 90 for i in fing})

    def point_err(i):
        W = P.solve(rig, cm, flexed)
        pip = P.apply(W[fing[i][1]], np.array(jd['pip%d' % i]['axes'][0]['center'])[None])[0]
        dip = P.apply(W[fing[i][1]], np.array(jd['dip%d' % i]['axes'][0]['center'])[None])[0]
        t = P.apply(W['Scaphoid bone'], tub[None])[0]
        n = W['Radius'][:3, :3] @ vol
        a, b = dip - pip, t - pip
        a, b = a - (a @ n) * n, b - (b @ n) * n
        return np.degrees(np.arccos(np.clip(F.unit(a) @ F.unit(b), -1, 1)))
    done = []
    for i in (3, 2, 4, 5):
        fx, ab = jd['mcp%d' % i]['axes']
        px = jd['pip%d' % i]['axes'][0]
        a0, b0, p0 = np.array(fx['axis']), np.array(ab['axis']), np.array(px['axis'])
        prev, nxt = (np.array(v) for v in fx['ref'])
        pprev, pnxt = (np.array(v) for v in px['ref'])
        pn = F.unit(np.cross(p0, pprev))

        def setd(dm, dp):
            R, Q = F.axis_angle(vol, dm), F.axis_angle(pn, dp)
            fx['axis'], ab['axis'], px['axis'] = (R @ a0).tolist(), (R @ b0).tolist(), (Q @ p0).tolist()
            rig['dofs']['mcp%d_flex' % i]['rest'] = round(F.signed_angle(prev, nxt, R @ a0), 2)
            rig['dofs']['mcp%d_abd' % i]['rest'] = round(F.signed_angle(prev, nxt, R @ b0), 2)
            rig['dofs']['pip%d' % i]['rest'] = round(F.signed_angle(pprev, pnxt, Q @ p0), 2)

        def cost(dm, dp):
            setd(dm, dp)
            W = P.solve(rig, cm, fist)
            hit = max([ov.depth(a, W[a], b, W[b]) for n in done if abs(n - i) == 1
                       for a in fing[i] for b in fing[n]] + [0])
            e = point_err(i)
            return max(e - 8, 0) + 20e3 * hit + 0.1 * (abs(dm) + abs(dp)), e, hit
        c0 = cost(0.0, 0.0)
        grid = [(float(m), float(q)) for m in range(-15, 16, 5) for q in range(-15, 16, 5)]
        best = min(((cost(*g), g) for g in grid), key=lambda t: t[0][0])
        if best[0][0] >= c0[0] - 0.5:
            best = (c0, (0.0, 0.0))
        setd(*best[1])
        for d in (fx, ab, px):
            d['axis'] = np.round(d['axis'], 6).tolist()
        log('  finger %d: MCP axis turned %+d, PIP axis %+d deg: %.0f -> %.0f deg off the tubercle, '
            'fist contact with neighbour %.1f -> %.1f mm' % (i, best[1][0], best[1][1], c0[1], best[0][1],
                                                            c0[2] * 1e3, best[0][2] * 1e3))
        done.append(i)
