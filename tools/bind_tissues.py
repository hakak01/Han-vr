"""Phase E2: bind every tissue to the E1 rig -> assets/hand_rig.glb (skinned) + assets/cables.json.
Run: python3 tools/bind_tissues.py   (needs data/raw/hand_raw.glb, gaps_raw.glb, curves.json from zcurves.py)
Per-vertex attributes in hand_rig.glb (besides JOINTS_0/WEIGHTS_0):
  _CID   cable id (-1 = none; <= -2: follow cable -_CID-2 without sliding, e.g. flexor sheaths); _CABLE = (u, ox, oy, oz): rest arc length on the cable and offset in its frame
  _SLIDE = (cable id, u, f, fb): muscle belly moves f x (mean excursion of the muscle's tendons) along the muscle's
           straight belly axis (cables.json muscles: origin -> junction), bulge weight fb;
           fb = -1: lumbrical, moves f x the FDP material displacement at u along that cable
  (fused proximal sheets of FDS/FDP/EDC ride on a 'common' cable = mean path of the muscle's tendons)
  _SHIFT lateral-band volar shift weight (0-1)."""
import json, os, sys, subprocess
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import glbio, glbskin, rigfit as F, bindskin as K, bindcable as C, cablegeom as G

SHEATHS = ['Synovial sheaths of digits of hand', 'Common flexor tendon sheath', 'Tendon sheath of flexor pollicis longus']
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = os.path.join(ROOT, 'data/raw/hand_rig_raw.glb')
LOG = []


def log(*a):
    s = ' '.join(str(x) for x in a)
    LOG.append(s)
    print(s)


def method(name, layer, D):
    if layer == 'bone':
        return 'bone'
    if layer == 'attachment':
        return 'rigid_near'
    for d in range(1, 6):
        for pn, bone in C.pulleys_of(d):
            if pn == name:
                return ('rigid', bone)
    if name.startswith('Volar plate of metacarpophalangeal joint'):
        f = name.rsplit(' of ', 1)[1]
        d = 1 if f == 'thumb' else {v: k for k, v in C.FING.items()}[f.split()[0]]
        return ('rigid', K.ph('Proximal', d))
    if name in ('Palmar interphalangeal ligaments',):
        return 'rigid_distal'
    if name in ('Nail plate',):
        return 'rigid_dp'
    if layer == 'ligament_capsule' and not any(k in name for k in ('Sagittal', 'Triangular', 'retinacular')):
        return 'liga'
    return 'ray'


def forearm_axis(rig):
    j = rig['joints'][0]['axes'][0]
    c, a = np.array(j['center']), F.unit(j['axis'])
    d = -a if a[1] > 0 else a                                # distal = toward the hand (-Y)
    return dict(c=c, d=d, len=float(-c @ d) - 25e-3)          # wrist ~25 mm proximal to the capitate centroid


def main():
    D = glbio.read(os.path.join(ROOT, 'data/raw/hand_raw.glb'))
    Gp = glbio.read(os.path.join(ROOT, 'data/raw/gaps_raw.glb'))
    man = {e['name']: e for e in json.load(open(os.path.join(ROOT, 'assets/manifest.json'))) +
           json.load(open(os.path.join(ROOT, 'assets/gaps.json')))}
    D.update(Gp)
    rig = json.load(open(os.path.join(ROOT, 'assets/rig.json')))
    curves = json.load(open(os.path.join(ROOT, 'data/raw/curves.json')))
    S = K.BoneShapes(D)
    fa = forearm_axis(rig)
    names = [n for n in D if n in man]
    log('structures', len(names), 'joints', K.NJ, 'forearm length used %.0f mm' % (fa['len'] * 1e3))

    # 1. skin weights per primitive
    Wp = {}
    ray_items, P_all, F_all, off = [], [], [], 0
    for n in names:
        m = method(n, man[n]['layer'], D)
        for k, (v, f, mt) in enumerate(D[n]):
            if m == 'bone':
                Wp[n, k] = K.rigid_weights(v, [n] * len(v))
            elif m == 'rigid_near' or m == 'rigid_dp':
                pool = [K.ph('Distal', i) for i in range(1, 6)] if m == 'rigid_dp' else K.BONES
                Wp[n, k] = K.rigid_weights(v, K.nearest_bone(S, v, pool)[0])
            elif m == 'rigid_distal':
                d = np.stack([S.surf(v, b) for b in K.BONES], 1)
                two = np.argsort(d, 1)[:, :2]
                pick = np.where(two[:, 0] > two[:, 1], two[:, 0], two[:, 1])   # BONES are ordered proximal -> distal
                Wp[n, k] = K.rigid_weights(v, np.array(K.BONES)[pick])
            elif isinstance(m, tuple):
                Wp[n, k] = K.rigid_weights(v, [m[1]] * len(v))
            elif m == 'liga':
                W, touched = K.liga_weights(S, v)
                if W is None:
                    m = 'ray'
                else:
                    node, A, has = K.weld_graph(v, [f])
                    Wp[n, k] = K.smooth(W, node, A, has, passes=3, keep=W.max(1) > 0.999)
            if m == 'ray':
                ray_items.append((n, k)); P_all.append(v); F_all.append(f + off); off += len(v)
    P = np.vstack(P_all)
    sup = np.concatenate([np.full(len(v), man[n]['layer'] in ('skin', 'fascia_retinacula') or 'fat' in n.lower())
                          for (n, k), v in zip(ray_items, P_all)])
    W = K.twist_split(K.ray_weights(S, P), P, fa, uniform=sup)
    node, A, has = K.weld_graph(P, F_all)
    W = K.smooth(W, node, A, has, passes=10)
    sk = np.concatenate([np.full(len(v), man[n]['layer'] == 'skin' or 'fat' in n.lower()) for (n, k), v in zip(ray_items, P_all)])
    W[sk] = K.spatial_smooth(P[sk], W[sk], 2e-3, 3)             # skin regions + fat: same weights across seams
    o = 0
    for (n, k), v in zip(ray_items, P_all):
        Wp[n, k] = W[o:o + len(v)]; o += len(v)
    log('ray-skinned vertices', len(P), 'in', len(ray_items), 'primitives')

    # 2. cables
    B = C.CableBuilder(D, S, rig, fa, log)
    tend = {key: B.tendons(key) for key in C.EXTRINSIC}
    offs = np.median([np.vstack([p[0] for p in D[n]]).mean(0) - np.array(c['mean']) for n, c in curves.items() if n in D], 0)
    log('curve offset (mm)', np.round(offs * 1e3, 2).tolist())
    curv = {n: B.curve_cables(n, man[n]['layer'], c['lines'], offs) for n, c in curves.items() if n in D}
    attrs = {}

    def bind_mesh(n, k, cabs, tendon_part, reach=None):
        v = D[n][k][0]
        best = np.full(len(v), np.inf); cid = np.full(len(v), -1.0); cab = np.zeros((len(v), 4))
        bbest = np.full(len(v), np.inf); beyond = np.zeros(len(v), bool)
        for (i, Pc, Nc, um) in cabs:
            u, ox, oy, oz, dist = G.bind_offsets(Pc, Nc, v)
            r = np.interp(u, G.arclen(Pc), B.cables[i]['r'])
            ok = dist < best
            if tendon_part and um is not None:               # whole tendon distal to the junction rides on its cable,
                ok &= (u > um - 5e-3) & (oz < 3e-3)          # not what lies beyond its end (palmar aponeurosis)
            else:
                ok &= dist < (reach or 3 * r + 1e-3)
            best[ok] = dist[ok]; cid[ok] = i
            cab[ok] = np.stack([u, ox, oy, oz], 1)[ok]
            nb = dist < bbest
            bbest[nb] = dist[nb]; beyond[nb] = oz[nb] >= 3e-3
        bind_mesh.beyond = beyond & (cid < 0)
        return cid, cab

    def belly(v, key):
        m = B.muscles[key]
        O, P1 = np.array(m['origin']), np.array(m['junction'])
        u = np.clip((v - O) @ F.unit(P1 - O), 0, m['L'])
        f = u / m['L']
        return np.stack([np.full(len(v), m['cables'][0]), u, f, 4 * f * (1 - f)], 1)

    def slide(v, cabs, fade):
        best = np.full(len(v), np.inf); sl = np.zeros((len(v), 4))
        for (i, Pc, Nc, um) in cabs:
            u, dist, _, _ = G.project(Pc, v)
            ok = dist < best
            best[ok] = dist[ok]
            f = fade(u, um, i)
            sl[ok] = np.stack([np.full(len(v), i), u, f, 4 * f * (1 - f)], 1)[ok]
        return sl

    for key, (bellies, tmeshes, *_r) in C.EXTRINSIC.items():
        cabs = tend[key]
        for n in set(bellies + tmeshes):
            for k, (v, f, mt) in enumerate(D[n]):
                a = {}
                if mt == 'Tendon' and n in tmeshes:
                    cid, cab = bind_mesh(n, k, cabs, True)
                    a['_CID'], a['_CABLE'] = cid, cab
                    free = cid < 0
                    still = bind_mesh.beyond                 # beyond the tendon's end (palmar aponeurosis): no slide
                    if key in B.common:                      # fused part of a multi-tendon sheet: common path
                        d = np.sort(np.stack([G.project(Pc, v)[1] for (_, Pc, _, _) in cabs], 1), 1)
                        fused = (d[:, 1] - d[:, 0]) < 2e-3
                        ci, Pc, Nc = B.common[key]
                        u, ox, oy, oz, _ = G.bind_offsets(Pc, Nc, v[fused])
                        cid[fused] = ci
                        cab[fused] = np.stack([u, ox, oy, oz], 1)
                else:
                    free, still = np.ones(len(v), bool), np.zeros(len(v), bool)
                sl = belly(v, key)
                sl[~free | still] = 0; sl[~free | still, 0] = -1
                a['_SLIDE'] = sl
                attrs[n, k] = a
                log('  %-55s %-22s verts %5d on cable %5d sliding %5d' % (n[:55], mt[:22], len(v), (~free).sum(), free.sum()))
    fdp = {c['digit']: x for x, c in zip(tend['FDP'], [B.cables[t[0]] for t in tend['FDP']])}
    lum = D['Lumbrical muscles of hand']
    sls = [slide(v, list(fdp.values()), lambda u, um, i: u) for v, f, mt in lum]
    allsl = np.vstack(sls)
    for i in set(allsl[:, 0].astype(int)):                  # f: 1 at the origin on FDP (proximal) -> 0 at the insertion
        lo, hi = allsl[allsl[:, 0] == i, 1].min(), allsl[allsl[:, 0] == i, 1].max()
        for sl in sls:
            m = sl[:, 0] == i
            sl[m, 2] = (hi - sl[m, 1]) / max(hi - lo, 1e-6)
    for k, sl in enumerate(sls):
        sl[:, 3] = -1                                         # -1 = move with the local tendon material, no bulge
        attrs['Lumbrical muscles of hand', k] = {'_SLIDE': sl}
    for n, cabs in curv.items():
        for k in range(len(D[n])):
            cid, cab = bind_mesh(n, k, cabs, False)
            attrs[n, k] = {'_CID': cid, '_CABLE': cab}
            if (cid < 0).any():
                log('  %s: %d of %d vertices not on a cable (skinned only)' % (n, (cid < 0).sum(), len(cid)))
    flex = [t for key in ('FDS', 'FDP', 'FPL') for t in tend[key]]
    for n in SHEATHS:                                        # sheaths wrap the flexors: follow their paths, no slide
        for k, (v, f, mt) in enumerate(D[n]):
            cid, cab = bind_mesh(n, k, flex, False, reach=6e-3)
            cid[cid >= 0] = -cid[cid >= 0] - 2
            attrs[n, k] = {'_CID': cid, '_CABLE': cab}
            log('  %s: %d of %d vertices follow flexor cables' % (n, (cid <= -2).sum(), len(cid)))
    lat = {}
    J = {j['id']: j for j in rig['joints']}
    for d, fn in C.FING.items():
        n = 'Lateral bands of %s finger' % fn
        ax = J['pip%d' % d]['axes'][0]
        c, a = np.array(ax['center']), F.unit(ax['axis'])
        pa, pb, _ = S.seg[K.ph('Proximal', d)]
        lng = F.unit(pb - pa) if (pb - pa) @ (c - pa) > 0 else F.unit(pa - pb)
        vol = F.unit(np.cross(a, lng))
        fdpP = np.array(B.cables[fdp[d][0]]['pts'])
        q = fdpP[np.linalg.norm(fdpP - c, axis=1).argmin()]
        if vol @ (q - c) < 0:
            vol = -vol
        lat[n] = dict(dof='pip%d' % d, bone=K.ph('Proximal', d), dir=np.round(vol, 4).tolist(), max_m=0.003,
                      center=np.round(c, 6).tolist(), radius_m=0.008)
        for k, (v, f, mt) in enumerate(D[n]):
            attrs[n, k] = {'_SHIFT': np.exp(-(np.linalg.norm(v - c, axis=1) / 8e-3) ** 2)}

    # 3. write
    objs = []
    for n in names:
        ps = []
        for k, (v, f, mt) in enumerate(D[n]):
            j, w = K.top4(Wp[n, k])
            ps.append(dict(v=v, f=f, mat=mt, J=j, W=w, attrs=attrs.get((n, k))))
        objs.append((n, ps))
    ax = rig['joints'][0]['axes'][0]
    jx = {t: dict(twist=s, about='forearm', center=ax['center'], axis=ax['axis']) for t, s in zip(K.TWIST, K.TWIST_S)}
    glbskin.write(RAW, objs, K.JOINTS, jx, extras=dict(note='Phase E2 bind_tissues.py; see assets/cables.json'))
    out = dict(
        note='Phase E2 cables. Units m, rest = hand.glb geometry. Sample i of a cable is carried by bones j[i] with weights '
             'w[i] (indices into joints, same as the glTF skin). Twist joint k = rotation about the forearm axis by '
             'twist x (pronation - rest). Tendon: insertion end (last sample) fixed, constant length from the insertion, '
             'muscle-tendon junction at u_mtj; belly moves f x excursion, radius x 1/sqrt(axial scale). Nerves/vessels: '
             'both ends fixed; slack shows as a wave (wavelength glide.wave_m), when taut they glide (uniform strain). '
             'E2b zones[k] = {child, parent, i0, i1, dofs}: samples i0..i1 between the zone ends i0-1 and i1+1 are posed in '
             'polar coordinates about arc_joints[child].center (axis = parent->child rotation axis, parent frame): each keeps '
             'its distance from the axis and takes the ends\' changes of angle, radius and axial position in proportion to '
             'its rest angle between them (tisspose.zone). '
             'Tendon mechanics: mech[dof] = excursion table F (m) from deg0 in steps of step deg = integral of the ARMS '
             'moment arm (tools/arms_ma.json, scaled by mech_scale); a zone\'s material slides by sum F (dofs), the rest of '
             'the difference to the visual path is taken up in the zone. Zones without dofs (CMC1) use the visual path.',
        mech_scale=B.scale,
        joints=K.JOINTS, twist=[dict(joint=t, fraction=s) for t, s in zip(K.TWIST, K.TWIST_S)],
        forearm=dict(center=ax['center'], axis=ax['axis'], dof='pronation'),
        glide=dict(wave_m=0.008, wave_max_m=0.002),
        arc_joints={c: dict(parent=a[0], center=np.round(a[1], 6).tolist()) for c, a in B.arcs.items()},
        lateral_bands=lat, muscles=B.muscles, cables=B.cables)
    json.dump(out, open(os.path.join(ROOT, 'assets/cables.json'), 'w'), separators=(',', ':'))
    subprocess.run('npx -y -q @gltf-transform/cli meshopt "%s" "%s" --level medium' % (
        RAW, os.path.join(ROOT, 'assets/hand_rig.glb')), shell=True, check=True, stdout=subprocess.DEVNULL)
    log('cables', len(B.cables), 'cables.json %.0f kB' % (os.path.getsize(os.path.join(ROOT, 'assets/cables.json')) / 1e3),
        'hand_rig.glb raw %.1f MB, compressed %.1f MB' % (os.path.getsize(RAW) / 1e6,
                                                          os.path.getsize(os.path.join(ROOT, 'assets/hand_rig.glb')) / 1e6))
    open(os.path.join(ROOT, 'data/raw/bind_log.txt'), 'w').write('\n'.join(LOG) + '\n')


if __name__ == '__main__':
    main()
