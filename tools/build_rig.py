"""Phase E1: fit joint axes on hand_raw.glb bones -> assets/rig.json (+ assets/joints.json if missing).
Coordinates: glTF world, metres (same as hand.glb). Run: python3 tools/build_rig.py
Needs: numpy, scipy; data/raw/hand_raw.glb; assets/carpal_model.json."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import glbio, rigfit as F, rigpose as P

ROOT = P.ROOT
ORD = ['first', 'second', 'third', 'fourth', 'fifth']
CARPAL = {'Scaphoid bone': 'Scaphoid', 'Lunate bone': 'Lunate', 'Triquetrum bone': 'Triquetrum',
          'Capitate bone': 'Capitate', 'Hamate bone': 'Hamate', 'Trapezoid bone': 'Trapezoid',
          'Trapezium bone': 'Trapezium'}
LOG = []


def log(*a):
    s = ' '.join(str(x) for x in a)
    LOG.append(s)
    print(s)


def mm(p):
    return np.round(np.asarray(p) * 1000, 1).tolist()


def r6(p):
    return np.round(np.asarray(p, float), 6).tolist()


def mc(i):
    return '%s metacarpal bone' % ORD[i - 1].capitalize()


def ph(kind, i):
    return '%s phalanx of %s finger of hand' % (kind, ORD[i - 1])


class Bones:
    def __init__(self, D):
        self.D = D

    def all(self, b):
        return F.weld(*F.merge(self.D[b]))

    def verts(self, b):
        return self.all(b)[0]

    def cart(self, b):
        return F.components(*F.merge(self.D[b], 'Cartilage'), min_faces=5)

    def head_base(self, b, prox):
        """Cartilage components farthest from / nearest to point prox."""
        cs = sorted(self.cart(b), key=lambda c: np.linalg.norm(c.mean(0) - prox))
        return cs[-1], cs[0]


def forearm(B):
    rad, uln = B.verts('Radius'), B.verts('Ulna')
    c0, a0 = F.pca_axis(rad, toward=[0, -1, 0])            # glTF -Y is distal
    s = (rad - c0) @ a0
    dend = s.max()
    cents = [rad[(s > dend - k - 3e-3) & (s < dend - k + 3e-3)].mean(0) for k in np.arange(30e-3, 81e-3, 5e-3)]
    _, d = F.pca_axis(np.array(cents), toward=a0)           # distal radius shaft axis (distal +)
    rc = sorted(B.cart('Radius'), key=lambda c: c.mean(0) @ d)
    uc = sorted(B.cart('Ulna'), key=lambda c: c.mean(0) @ d)
    uhead = uc[-1]
    fovea, ur, _ = F.fit_sphere(uhead)
    near = np.linalg.norm(rc[-1] - fovea, axis=1) < ur + 2.5e-3
    notch = rc[-1][near].mean(0)
    side = (rad - notch) @ d
    cand = rad[side > np.percentile(side, 99.5)]
    sty = cand[np.argmax(np.linalg.norm((cand - notch) - np.outer((cand - notch) @ d, d), axis=1))]
    y = F.unit((sty - notch) - ((sty - notch) @ d) * d)      # radial

    volar = F.unit(np.cross(y, d))                           # right hand: y x d = volar
    head = rc[0]
    hc, ha, hr, hrms = F.fit_cylinder(head, d)
    rim = head[np.linalg.norm(np.cross(head - hc, ha), axis=1) > 0.85 * hr]   # drop the fovea
    hc, ha, hr, hrms = F.fit_cylinder(rim, ha, hc)
    hum = sorted(B.cart('Humerus'), key=lambda c: c.mean(0) @ d)[-1]          # trochlea + capitellum
    ec, ea, er, erms = F.fit_cylinder(hum, [1.0, 0, 0])
    log('elbow axis (humerus distal cartilage cylinder) %s r %.1f mm rms %.2f' % (np.round(ea, 2), er * 1e3, erms * 1e3))
    log('radial head: r %.1f mm rms %.2f; ulnar head sphere r %.1f mm' % (hr * 1e3, hrms * 1e3, ur * 1e3))
    log('RCS distal', np.round(d, 3), 'radial', np.round(y, 3), 'volar', np.round(volar, 3))
    return dict(distal=d, radial=y, volar=volar, notch=notch, styloid=sty, fovea=fovea, elbow=ea,
                rhead=hc, carpal_face=rc[-1].mean(0))


def hinge_surface(B, prox, dist, lng, a0=None):
    """Thumb hinges: cylinder on the proximal bone's surface within 3 mm of the distal bone, axis kept
    normal to the bone; multi-start when no initial axis. (Cartilage patches of the thumb fit badly.)"""
    from scipy.spatial import cKDTree
    P_, Q = B.verts(prox), B.verts(dist)
    h = P_[cKDTree(Q).query(P_)[0] < 3e-3]
    u, w = F._perp(lng)
    starts = [a0] if a0 is not None else [np.cos(t) * u + np.sin(t) * w for t in np.radians(np.arange(0, 180, 10))]
    return min((F.fit_cylinder(h, s, perp_to=lng) for s in starts), key=lambda r: r[3])


def finger_axes(B, i, A, joints, dofs):
    """MCP/PIP/DIP (rays 2-5) or MCP/IP (thumb) hinge axes and rest angles."""
    cap = F.volume_centroid(*B.all('Capitate bone'))[1]
    M = mc(i)
    mh, mb = B.head_base(M, cap)
    chain = [ph('Proximal', i)] + ([ph('Middle', i)] if i > 1 else []) + [ph('Distal', i)]
    pts, axes = [mb.mean(0)], []
    if i > 1:
        cen, rr, rms = F.fit_sphere(mh)
        _, a, cr, crms = F.fit_cylinder(mh, A['radial'], cen)
        log('MCP%d sphere r %.1f mm rms %.2f, cyl r %.1f rms %.2f' % (i, rr * 1e3, rms * 1e3, cr * 1e3, crms * 1e3))
        pts.append(cen); axes.append(a)
    prox = pts[-1] if i > 1 else None
    heads = [(M, mh)] if i == 1 else []
    for b in chain[:-1]:
        p0 = prox if prox is not None else mh.mean(0)
        h, _ = B.head_base(b, p0)
        heads.append((b, h))
        prox = h.mean(0)
    if i == 1:                                                # thumb: IP first, MCP started from it
        l1 = F.unit(heads[1][1].mean(0) - heads[0][1].mean(0))
        ip = hinge_surface(B, chain[0], chain[1], l1)
        l0 = F.unit(heads[0][1].mean(0) - pts[0])
        mcp = hinge_surface(B, M, chain[0], l0, ip[1])
        log('thumb MCP/IP axes from joint-facing bone surface; angle between them %.0f deg' %
            np.degrees(np.arccos(abs(ip[1] @ mcp[1]))))
    for k, (b, h) in enumerate(heads):
        if i == 1:
            c, a, r, rms = (mcp, ip)[k]
            if k == 0:                                        # MCP centre = sphere fit of the MC1 head
                c = F.fit_sphere(h)[0]
        else:
            c, a, r, rms = F.fit_cylinder(h, A['radial'])
        log('%-42s hinge r %.1f mm rms %.2f' % (b[:42], r * 1e3, rms * 1e3))
        pts.append(c); axes.append(a)
    dv = B.verts(chain[-1])
    pts.append(dv[np.argmax(np.linalg.norm(dv - pts[-1], axis=1))])      # fingertip
    bones = [M] + chain
    names = (['mcp%d_flex' % i, 'pip%d' % i, 'dip%d' % i] if i > 1 else ['mcp1', 'ip1'])
    for k, d in enumerate(names):
        c, a = pts[k + 1], axes[k]
        prev, nxt = F.unit(pts[k + 1] - pts[k]), F.unit(pts[k + 2] - pts[k + 1])
        if i > 1:
            a = a if a @ A['radial'] > 0 else -a
        else:
            vel = np.cross(a, pts[-1] - c)
            a = a if vel @ (A['mc2head'] - pts[-1]) > 0 else -a
        rest = F.signed_angle(prev, nxt, a)
        dofs[d] = dict(rest=round(rest, 2))
        ax = [dict(dof=d, center=r6(c), axis=r6(a), ref=[r6(prev), r6(nxt)])]
        if k == 0 and i > 1:                                  # MCP ab/adduction, fixed in P1
            b2 = F.unit(np.cross(a, nxt))
            b2 = b2 if np.cross(b2, nxt) @ A['radial'] > 0 else -b2   # + = radial deviation
            dofs['mcp%d_abd' % i] = dict(rest=round(F.signed_angle(prev, nxt, b2), 2))
            ax.append(dict(dof='mcp%d_abd' % i, center=r6(c), axis=r6(b2)))
        joints.append(dict(id=d.replace('_flex', ''), type='hinge', parent=bones[k], child=bones[k + 1], axes=ax))
        log('  %-9s centre %s mm  axis %s  rest %.1f deg' % (d, mm(c), np.round(a, 2), rest))
    return dict(base=pts[0], head=pts[1], tip=pts[-1], cmc_base=mb)


def thumb_cmc(B, A):
    """Hollister 1992: FE axis through the centre of the trapezium saddle's convex curvature (inside the
    trapezium), AA axis through the centre of its concave curvature (inside the MC1 base)."""
    from scipy.spatial import cKDTree
    tm, m1 = 'Trapezium bone', mc(1)
    T, M = B.verts(tm), B.verts(m1)
    t = T[cKDTree(M).query(T)[0] < 3e-3]                     # trapezium surface facing MC1
    c = t.mean(0)
    n0 = np.linalg.svd(t - c)[2][2]
    n0 = n0 if n0 @ (M.mean(0) - c) > 0 else -n0
    apex, n, ks = F.fit_saddle(t, n0)
    (kv, ev), (kc, ec) = sorted(ks, key=lambda k: -k[0])
    log('CMC1 trapezium saddle: convex r %.1f mm, concave r %.1f mm (%d surface points)' % (
        1e3 / kv, -1e3 / kc, len(t)))
    if kv <= 0 or kc >= 0:
        log('  WARNING: trapezium surface is not a saddle')
    c1, a1 = apex - n / kv, ec            # rotation along the convex direction, centre in trapezium
    c2, a2 = apex - n / kc, ev            # rotation along the concave direction, centre in MC1
    head = A['mc1head']
    a1 = a1 if np.cross(a1, head - c1) @ A['thumb_pulp'] > 0 else -a1     # + flexion: toward thumb pulp
    a2 = a2 if np.cross(a2, head - c2) @ (head - A['mc2head']) > 0 else -a2   # + abduction: away from MC2
    ang = np.degrees(np.arccos(abs(a1 @ a2)))
    cr = np.cross(a1, a2)
    gap = abs(cr @ (c2 - c1)) / max(np.linalg.norm(cr), 1e-9)
    log('CMC1 FE axis (trapezium) centre %s axis %s; AA axis (MC1) centre %s axis %s' % (
        mm(c1), np.round(a1, 2), mm(c2), np.round(a2, 2)))
    log('CMC1 axes angle %.0f deg, %.1f mm apart' % (ang, gap * 1e3))
    return dict(id='cmc1', type='hinge', parent=tm, child=m1,
                axes=[dict(dof='cmc1_fe', center=r6(c1), axis=r6(a1)), dict(dof='cmc1_aa', center=r6(c2), axis=r6(a2))])


def cup_axis(B, i, A, info):
    base = info['cmc_base']
    lng = F.unit(info['head'] - info['base'])
    c, a, r, rms = F.fit_cylinder(base, A['radial'])
    ok = 3e-3 < r < 30e-3 and abs(a @ A['radial']) > 0.7
    if not ok:
        a = F.unit(A['radial'] - (A['radial'] @ lng) * lng)
        c = base.mean(0)
    a = a if np.cross(a, info['head'] - c) @ A['volar'] > 0 else -a
    log('CMC%d cup axis %s (r %.1f mm, %s)' % (i, np.round(a, 2), r * 1e3, 'cylinder fit' if ok else 'fallback'))
    return dict(id='cmc%d' % i, type='hinge', parent='Hamate bone', child=mc(i),
                axes=[dict(dof='cmc%d' % i, center=r6(c), axis=r6(a))])


def main():
    D = glbio.read(os.path.join(ROOT, 'data/raw/hand_raw.glb'))
    B = Bones(D)
    A = forearm(B)
    d, y, vol = A['distal'], A['radial'], A['volar']
    dofs, joints = {}, []
    # forearm: radius about radial head centre -> ulnar head fovea; + = pronation
    fa = F.unit(A['fovea'] - A['rhead'])
    fa = fa if np.cross(fa, A['styloid'] - A['rhead']) @ vol > 0 else -fa
    yp = F.unit(y - (y @ fa) * fa)
    lat = F.unit(A['elbow'] - (A['elbow'] @ fa) * fa)       # elbow flexion axis, pointing lateral
    lat = lat if lat[0] < 0 else -lat
    ant = F.unit(np.cross(lat, fa))
    ant = ant if ant[2] > 0 else -ant
    sup0 = np.degrees(np.arctan2(yp @ lat, yp @ ant))
    dofs['pronation'] = dict(rest=round(-sup0, 2))
    joints.append(dict(id='forearm', type='hinge', parent='Ulna', child='Radius',
                       axes=[dict(dof='pronation', center=r6(A['rhead']), axis=r6(fa))]))
    log('forearm axis: radial head %s -> fovea %s mm; rest supination %.1f deg (radial axis vs elbow axis)' % (mm(A['rhead']), mm(A['fovea']), sup0))
    # wrist (Akhbari 2019): rest angles of MC3 vs radius
    m3h, m3b = B.head_base(mc(3), A['carpal_face'])
    m3 = F.unit(F.fit_sphere(m3h)[0] - m3b.mean(0))
    fe0 = np.degrees(np.arctan2(m3 @ vol, m3 @ d))
    ru0 = np.degrees(np.arctan2(-(m3 @ y), m3 @ d))
    dofs['wrist_fe'] = dict(rest=round(fe0, 2))
    dofs['wrist_ru'] = dict(rest=round(ru0, 2))
    capV, _ = F.volume_centroid(*B.all('Capitate bone'))
    wb = {}
    for b, model in CARPAL.items():
        if model in ('Hamate', 'Trapezoid', 'Trapezium'):    # distal row: rigid with the capitate
            continue
        wb[b] = dict(model=model, centroid=r6(F.volume_centroid(*B.all(b))[1]),
                     row='proximal' if model != 'Capitate' else 'distal')
        joints.append(dict(id='wrist', type='carpal', parent='Radius', child=b))
    for b in ('Hamate bone', 'Trapezoid bone', 'Trapezium bone'):
        joints.append(dict(id='distal_row', type='hinge', parent='Capitate bone', child=b, axes=[]))
    wrist = dict(rest=dict(wrist_fe=fe0, wrist_ru=ru0), scale_m=float(np.cbrt(capV)),
                 distal=r6(d), ulnar=r6(-y), dorsal=r6(-vol),
                 flex_axis=r6(F.unit(np.cross(d, vol))), ulnar_axis=r6(F.unit(np.cross(d, -y))),
                 rcs=dict(origin=r6(A['carpal_face']), x_distal=r6(d), y_radial=r6(y), z=r6(np.cross(d, y))),
                 bones=wb)
    log('wrist rest (MC3 vs radius): FE %.1f, RU %.1f deg; capitate volume %.0f mm3, cbrt %.2f mm' % (
        fe0, ru0, capV * 1e9, np.cbrt(capV) * 1e3))
    joints.append(dict(id='pisiform', type='hinge', parent='Triquetrum bone', child='Pisiform bone', axes=[]))
    joints.append(dict(id='cmc2', type='hinge', parent='Trapezoid bone', child=mc(2), axes=[]))
    joints.append(dict(id='cmc3', type='hinge', parent='Capitate bone', child=mc(3), axes=[]))
    A['mc2head'] = F.fit_sphere(B.head_base(mc(2), A['carpal_face'])[0])[0]
    A['mc1head'] = B.head_base(mc(1), A['carpal_face'])[0].mean(0)
    info = {}
    fj = []
    for i in (2, 3, 4, 5, 1):
        info[i] = finger_axes(B, i, A, fj, dofs)
    m1 = next(j for j in fj if j['id'] == 'mcp1')['axes'][0]
    A['thumb_pulp'] = F.unit(np.cross(m1['axis'], info[1]['tip'] - np.array(m1['center'])))
    for i in (4, 5):
        joints.append(cup_axis(B, i, A, info[i]))
        dofs['cmc%d' % i] = dict(rest=0.0)
    joints.append(thumb_cmc(B, A))
    dofs['cmc1_fe'] = dict(rest=0.0)
    dofs['cmc1_aa'] = dict(rest=0.0)
    joints += fj
    rig = dict(units='m', frame='glTF world of hand.glb (Y up, origin = capitate centroid)',
               note='Pose: bone world = parent world x joint. Hinge axes: first fixed in parent, later in child. '
                    'Angles in dofs are anatomical (deg); rotation = value - rest. Carpal joints: Akhbari 2019 '
                    '(assets/carpal_model.json), SP=0, relative to wrist rest.',
               fixed=['Humerus', 'Ulna'], dofs=dofs, joints=joints, wrist=wrist,
               landmarks=dict(fingertip={str(i): r6(info[i]['tip']) for i in info},
                              styloid=r6(A['styloid']), notch=r6(A['notch']), fovea=r6(A['fovea'])))
    cm = json.load(open(os.path.join(ROOT, 'assets/carpal_model.json')))
    import rigsolve
    import rigrange
    J = json.load(open(rigrange.PATH)) if os.path.exists(rigrange.PATH) else rigrange.DEFAULT
    rigsolve.refine_forearm(B, rig, cm, log)
    rigsolve.refine_hinges(B, rig, cm, J, log)
    rigsolve.aim_fingers(B, rig, cm, log)
    rigsolve.carpal_corr(B, rig, cm, log)
    json.dump(rig, open(os.path.join(ROOT, 'assets/rig.json'), 'w'), indent=1)
    rigrange.write_joints(B, rig, log)
    open(os.path.join(ROOT, 'data/raw/rig_log.txt'), 'w').write('\n'.join(LOG) + '\n')


if __name__ == '__main__':
    main()
