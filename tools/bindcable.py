"""Phase E2: cables = sliding centrelines for extrinsic tendons, nerves and vessels.
Each cable: rest samples (2 mm) + carrier skin weights (which bones carry each sample) + frame normals.
Anchor zones override the carriers: insertion, pulleys, carpal tunnel, Guyon's canal, extensor retinaculum
(dorsal compartments), sagittal bands (extensor centred over the metacarpal head)."""
import numpy as np
import rigfit as F, bindskin as K, cablegeom as G

FING = {2: 'index', 3: 'middle', 4: 'ring', 5: 'little'}
# name: (meshes with the belly, mesh(es) with the tendon, digits, insertion bone kind, group)
EXTRINSIC = {
    'FDS': (['Humero-ulnar head of flexor digitorum superficialis', 'Radial head of flexor digitorum superficialis'],
            ['Humero-ulnar head of flexor digitorum superficialis'], [2, 3, 4, 5], 'Middle', 'flexor'),
    'FDP': (['Flexor digitorum profundus'], ['Flexor digitorum profundus'], [2, 3, 4, 5], 'Distal', 'flexor'),
    'FPL': (['Flexor pollicis longus'], ['Flexor pollicis longus'], [1], 'Distal', 'flexor'),
    'EDC': (['Extensor digitorum'], ['Extensor digitorum'], [2, 3, 4, 5], 'Middle', 'extensor'),
    'EIP': (['Extensor indicis'], ['Extensor indicis'], [2], 'Middle', 'extensor'),
    'EDM': (['Extensor digiti minimi'], ['Extensor digiti minimi'], [5], 'Middle', 'extensor'),
    'EPL': (['Extensor pollicis longus'], ['Extensor pollicis longus'], [1], 'Distal', 'extensor'),
    'EPB': (['Extensor pollicis brevis'], ['Extensor pollicis brevis'], [1], 'Proximal', 'extensor'),
    'APL': (['Abductor pollicis longus'], ['Abductor pollicis longus'], [None], 'First metacarpal bone', 'extensor'),
    'ECRL': (['Extensor carpi radialis longus'], ['Extensor carpi radialis longus'], [None], 'Second metacarpal bone', 'extensor'),
    'ECRB': (['Extensor carpi radialis brevis'], ['Extensor carpi radialis brevis'], [None], 'Third metacarpal bone', 'extensor'),
    'ECU': (['Humeral head of extensor carpi ulnaris', 'Ulnar head of extensor carpi ulnaris'],
            ['Humeral head of extensor carpi ulnaris', 'Ulnar head of extensor carpi ulnaris'], [None], 'Fifth metacarpal bone', 'extensor'),
    'FCR': (['Flexor carpi radialis'], ['Flexor carpi radialis'], [None], 'Second metacarpal bone', 'wrist_flexor'),
    'FCU': (['Humeral head of flexor carpi ulnaris', 'Ulnar head of flexor carpi ulnaris'],
            ['Humeral head of flexor carpi ulnaris', 'Ulnar head of flexor carpi ulnaris'], [None], 'Pisiform bone', 'wrist_flexor'),
    'PL': (['Palmaris longus muscle'], ['Palmaris longus muscle'], [None], None, 'pl'),
}
PULLEY_BONE = {'A1': 'Proximal', 'A2': 'Proximal', 'C1': 'Proximal', 'A3': 'Middle', 'A4': 'Middle',
               'C2': 'Middle', 'C3': 'Middle', 'A5': 'Distal'}
THUMB_PULLEY = {'A1 pulley of thumb': 'Proximal', 'Oblique pulley of thumb': 'Proximal', 'A2 pulley of thumb': 'Distal'}


def pulleys_of(digit):
    """-> [(mesh name, bone)] for the flexor pulleys of one digit."""
    if digit == 1:
        return [(n, K.ph(b, 1)) for n, b in THUMB_PULLEY.items()]
    f = FING[digit]
    out = [('%s pulley of %s finger' % (a, f), K.ph(PULLEY_BONE[a], digit)) for a in ('A1', 'A2', 'A3', 'A4', 'A5')]
    return out + [('%s cruciate pulley of %s finger' % (c, f), K.ph(PULLEY_BONE[c], digit)) for c in ('C1', 'C2', 'C3')]


def insertion_bone(kind, digit):
    if kind is None or kind.endswith('bone'):
        return kind
    return K.ph(kind, digit)


class CableBuilder:
    def __init__(self, D, S, rig, fa, log):
        self.D, self.S, self.rig, self.fa, self.log = D, S, rig, fa, log
        self.cables, self.muscles, self.common = [], {}, {}
        J = {j['id']: j for j in rig['joints']}
        self.mcp = {i: np.array(J['mcp%d' % i]['axes'][0]['center']) for i in range(2, 6)}
        # joint zones (E2b): the cable between the zone ends is posed in polar coordinates about the joint
        # (tisspose.zone). Half-width Z = clip(k * tendon height, Z0, Zmax), k = tan(max flexion / 2 + 10 deg), so the
        # zone's angular span stays larger than the joint angle (no loops).
        cap = S.pts['Capitate bone'].mean(0)
        self.arcs = {'Capitate bone': ('Radius', cap - 6e-3 * fa['d'], fa['d'], 12e-3, None)}
        self.zk = {'Capitate bone': (1.19, 20e-3)}
        self.zdofs = {'Capitate bone': ['wrist_fe', 'wrist_ru'], K.mc(1): []}
        for d in range(1, 6):
            names = [('cmc1', K.mc(1), 'Trapezium bone', 8e-3, 0.84, 12e-3)] if d == 1 else []
            prox = K.ray(d)
            for jid, child, parent, Z, k, Zx in names + [
                    (('mcp%d' % d), prox[1], prox[0], 8e-3, 1.43, 12e-3 if d == 1 else 16e-3),
                    (('ip1' if d == 1 else 'pip%d' % d), prox[2], prox[1], 6e-3, 1.19 if d == 1 else 2.14, 8e-3 if d == 1 else 11e-3)] + \
                    ([] if d == 1 else [('dip%d' % d, prox[3], prox[2], 4e-3, 1.19, 7e-3)]):
                ce = np.array(J[jid]['axes'][0]['center'])
                a, e, _ = S.seg[child]
                dr = F.unit(e - a) if np.linalg.norm(e - ce) > np.linalg.norm(a - ce) else F.unit(a - e)
                self.arcs[child] = (parent, ce, dr, Z, d)
                self.zk[child] = (k, Zx)
                if jid != 'cmc1':
                    self.zdofs[child] = ['mcp%d_flex' % d] if jid.startswith('mcp') and d > 1 else [jid]
        self.mech_tables(J)
        self.zone = {}
        for nm in ('Flexor retinaculum of wrist', 'Extensor retinaculum of wrist', "Volar carpal ligament (roof of Guyon's canal)"):
            v = np.vstack([p[0] for p in D[nm]])
            s = v @ fa['d']
            self.zone[nm] = (v, s.min(), s.max())

    # ---------- carriers ----------
    def carriers(self, P, kind, group=None, digit=None, ins_bone=None, name=''):
        W = self.S_weights(P)
        fixed = np.zeros(len(P), bool)
        def rigid(m, bone):
            W[m] = 0; W[m, K.JI[bone]] = 1; fixed[m] = True
        s = P @ self.fa['d']
        def in_zone(nm, pad, reach):
            v, lo, hi = self.zone[nm]
            from scipy.spatial import cKDTree
            d = cKDTree(v).query(P)[0]
            return (s > lo - pad) & (s < hi + pad) & (d < reach)
        _, ce, dr, Zw, _ = self.arcs['Capitate bone']
        xw = (P - ce) @ dr                                   # position along the forearm from the wrist arc centre
        wparent = 'Ulna' if name == 'ECU' else 'Radius'
        if group == 'flexor' or name == 'Median nerve' or name.startswith('FCR'):
            rigid(in_zone('Flexor retinaculum of wrist', 2e-3, 15e-3) & (xw > -Zw), 'Capitate bone')
        if name in ('Ulnar nerve', 'Ulnar artery', 'Superficial branch of ulnar nerve', 'Deep branch of ulnar nerve'):
            rigid(in_zone("Volar carpal ligament (roof of Guyon's canal)", 1e-3, 10e-3) & (xw > -Zw), 'Hamate bone')
        if group == 'extensor':                              # dorsal compartments: radius (6th, ECU: ulna)
            rigid(in_zone('Extensor retinaculum of wrist', 1e-3, 12e-3), wparent)
        wrist_arc = kind == 'tendon' or name in ('Median nerve', 'Ulnar nerve', 'Radial artery', 'Ulnar artery')
        if wrist_arc:                                        # blend into the arc's parent bone over 40 mm
            g = np.clip((xw + Zw + 40e-3) / 40e-3, 0, 1)
            m = (g > 0) & (xw < -Zw) & ~fixed
            e = np.zeros(K.NJ); e[K.JI[wparent]] = 1
            W[m] = (1 - g[m, None]) * W[m] + g[m, None] * e
            fixed |= m
            if digit and digit > 1:                       # sagittal bands: centred over the metacarpal head
                rigid(np.linalg.norm(P - self.mcp[digit], axis=1) < 11e-3, K.mc(digit))
        if group == 'flexor' and digit:
            for pn, bone in pulleys_of(digit):
                if pn not in self.D:
                    continue
                pv = np.vstack([p[0] for p in self.D[pn]])
                c = pv.mean(0)
                _, t = F.pca_axis(P[np.argsort(np.linalg.norm(P - c, axis=1))[:6]])
                a = (pv - c) @ t
                rr = np.linalg.norm((pv - c) - np.outer(a, t), axis=1).max()
                ap = (P - c) @ t
                pp = np.linalg.norm((P - c) - np.outer(ap, t), axis=1)
                rigid((np.abs(ap) < np.abs(a).max() + 0.5e-3) & (pp < rr), bone)
        if ins_bone:
            rigid(np.arange(len(P)) >= len(P) - 2, ins_bone)
        for _ in range(8):                               # smooth along the cable, anchors kept
            Ws = W.copy()
            Ws[1:-1] = 0.5 * W[1:-1] + 0.25 * (W[:-2] + W[2:])
            W = np.where(fixed[:, None], W, Ws)
        arc = [0] * len(P)
        if wrist_arc:
            best = np.full(len(P), 1.0)
            for child, (parent, ce, dr, Z, d) in self.arcs.items():
                if d is not None and d != digit and not (d == 1 and name in ('APL',) and child == K.mc(1)):
                    continue
                x = (P - ce) @ dr
                lat = np.linalg.norm((P - ce) - np.outer(x, dr), axis=1)
                near = (np.abs(x) < Z) & (lat < (30e-3 if d is None else 15e-3))
                if not near.any():
                    continue
                k, Zx = self.zk[child]                       # widen the zone with the tendon's height h
                Z = float(np.clip(k * np.hypot(x, lat)[near].min(), Z, Zx))
                q = np.abs(x) / Z
                for i in np.where((q < best) & (lat < (30e-3 if d is None else 15e-3)))[0]:
                    arc[i] = [child, round(float((x[i] + Z) / (2 * Z)), 4)] + ([wparent] if d is None else [])
                    best[i] = q[i]
        return W, arc

    # ---------- E2b: zones and tendon mechanics ----------
    def zones(self, arc):
        """Contiguous runs of samples per joint zone -> [{child, parent, i0, i1, dofs}] (i0-1, i1+1 = zone ends)."""
        out, n = [], len(arc)
        for child in dict.fromkeys(a[0] for a in arc if a):
            idx = [i for i, a in enumerate(arc) if a and a[0] == child]
            i0, i1 = max(min(idx), 1), min(max(idx), n - 2)
            if i1 < i0:
                continue
            a = arc[idx[0]]
            out.append(dict(child=child, parent=a[2] if len(a) > 2 else self.arcs[child][0], i0=i0, i1=i1,
                            dofs=self.zdofs.get(child, [])))
        return out

    def mech_tables(self, J):
        """ARMS moment arms (tools/arms_ma.json) scaled to our hand -> excursion tables F(deg) = integral r dq (m)."""
        import json, os
        A = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'arms_ma.json')))
        c = lambda j: np.array(J[j]['axes'][0]['center'])
        wc = self.arcs['Capitate bone'][1]
        sc = {'wrist': np.linalg.norm(c('mcp3') - wc) * 1e3 / A['segment_mm']['wrist'],
              'thumb': np.linalg.norm(c('ip1') - c('mcp1')) * 1e3 / A['segment_mm']['thumb']}
        for d in range(2, 6):
            sc['digit%d' % d] = np.linalg.norm(c('pip%d' % d) - c('mcp%d' % d)) * 1e3 / A['segment_mm']['digit%d' % d]
        self.scale = {k: round(float(v), 3) for k, v in sc.items()}
        self.log('E2b ARMS scale (ours / ARMS segment length): %s' % self.scale)
        grid = np.arange(-90, 125, 5.0)
        self.mech = {}
        for tn, curves in A['moment_arms'].items():
            tab = {}
            for dof, rows in curves.items():
                if dof.endswith('_abd') or dof.startswith('cmc1'):
                    continue                                 # sign convention vs our rig not checked: visual path
                q, r = np.array(rows, float).T
                s = sc['wrist'] if dof.startswith('wrist') else sc['thumb'] if dof in ('mcp1', 'ip1') else \
                    sc['digit%s' % dof[3]]
                rr = np.interp(grid, q, r) * 1e-3 * s       # clamped beyond the ARMS range
                Fc = np.r_[0, np.cumsum(0.5 * (rr[1:] + rr[:-1]) * np.radians(5))]
                Fc -= np.interp(0, grid, Fc)
                tab[dof] = dict(deg0=float(grid[0]), step=5.0, F=np.round(Fc, 6).tolist())
            self.mech[tn] = tab

    def S_weights(self, P):
        W = K.ray_weights(self.S, P)
        return K.twist_split(W, P, self.fa)

    def add(self, c):
        c['id'] = len(self.cables)
        self.cables.append(c)
        return c['id']

    def finish(self, name, kind, P, r, Wa, extra):
        W, arc = Wa
        self.Wfull = getattr(self, 'Wfull', {})
        self.Wfull[len(self.cables)] = W
        T, N = G.frames(P)
        j, w = K.top4(W)
        c = dict(name=name, kind=kind, pts=np.round(P, 6).tolist(), r=np.round(r, 5).tolist(),
                 nrm=np.round(N, 4).tolist(), j=j.tolist(), w=np.round(w, 3).tolist(),
                 L=round(float(G.arclen(P)[-1]), 5), arc=arc, zones=self.zones(arc))
        c.update(extra)
        return self.add(c), P, N

    # ---------- tendons ----------
    def tendons(self, key):
        bellies, tmeshes, digits, ins_kind, group = EXTRINSIC[key]
        tv, tf = F.weld(*F.merge([p for m in tmeshes for p in self.D[m]], 'Tendon'))
        dense = G.densify(tv, tf)
        bv = np.vstack([v for m in bellies for v, f, mt in self.D[m] if mt != 'Tendon'])
        bc = bv.mean(0)
        sb = bv @ self.fa['d']
        O = bv[sb < np.percentile(sb, 3)].mean(0)              # proximal end of the belly (origin side)
        nb, _ = K.nearest_bone(self.S, tv)
        out, p0s = [], []
        for k in digits:
            cand = tv if k is None else tv[np.isin(nb, K.ray(k))]
            if group == 'pl':                                    # tendon ends where the palmar aponeurosis starts
                lo = self.zone['Flexor retinaculum of wrist'][1]
                tip = tv[np.where(tv @ self.fa['d'] < lo, tv @ self.fa['d'], -9).argmax()]
            elif k is None:
                tip = tv[(tv @ self.fa['d']).argmax()]
            else:
                tip = cand[np.linalg.norm(cand - bc, axis=1).argmax()]
            Pt, rt = G.trace(dense, tip, bc)
            Pt, rt = Pt[::-1], rt[::-1]                          # proximal -> distal
            P0 = Pt[0]
            nbel = max(2, int(np.linalg.norm(P0 - O) / 4e-3) + 1)
            Pb = O + np.linspace(0, 1, nbel)[:-1, None] * (P0 - O)
            P = G.resample(G.smooth_line(np.vstack([Pb, Pt]), 2))
            s_all = G.arclen(np.vstack([Pb, Pt]))
            u_mtj = s_all[len(Pb)]
            sP = G.arclen(P)
            r = np.interp(sP, s_all[len(Pb):], rt, left=rt[0])
            ib = insertion_bone(ins_kind, k)
            if ib and self.S.surf(Pt[-1:], ib)[0] > 4e-3:          # tendon mesh ends short of that bone
                ib = K.nearest_bone(self.S, Pt[-1:])[0][0]
            nm = key + ('' if k is None else str(k))
            W = self.carriers(P, 'tendon', group, k, ib, nm)
            if ib is None:                                       # PL: palmar aponeurosis end follows the hand skin
                W[0][-3:] = self.S_weights(P[-3:])
            cid, P, N = self.finish(nm, 'tendon', P, r, W, dict(
                muscle=key, digit=k, insertion=ib, u_mtj=round(float(u_mtj), 5), group=group,
                bellies=bellies, origin_end=0, insertion_end='last', mech=self.mech.get(nm, {})))
            out.append((cid, P, N, u_mtj))
            p0s.append(P0)
            self.log('cable %-5s len %5.0f mm  tendon %5.0f mm  r %.1f-%.1f mm  ins %s' % (
                nm, sP[-1] * 1e3, (sP[-1] - u_mtj) * 1e3, r.min() * 1e3, r.max() * 1e3, ib))
        if len(out) > 1:                                          # common path: mean of the tendons' rest paths
            Ls = [G.arclen(o[1])[-1] for o in out]
            um = float(np.mean([o[3] for o in out]))
            s = np.arange(0, min(min(Ls), um + 80e-3), 2e-3)
            Pc = np.mean([np.stack([np.interp(s, G.arclen(o[1]), o[1][:, k]) for k in range(3)], 1) for o in out], 0)
            Wc = np.mean([[np.interp(s, G.arclen(o[1]), wk) for wk in self.Wfull[o[0]].T] for o in out], 0).T
            cid, Pc, Nc = self.finish(key + ' common', 'tendon', Pc, np.full(len(Pc), 2e-3), (Wc, [0] * len(Pc)), dict(
                muscle=key, common=True, members=[o[0] for o in out], u_mtj=round(um, 5), group=group))
            self.common[key] = (cid, Pc, Nc)
        P0 = np.mean(p0s, 0)                                    # one straight belly axis per muscle
        self.muscles[key] = dict(origin=np.round(O, 6).tolist(), junction=np.round(P0, 6).tolist(),
                                 L=round(float(np.linalg.norm(P0 - O)), 5), cables=[o[0] for o in out])
        return out

    # ---------- nerves and vessels ----------
    def curve_cables(self, name, kind, lines, offset):
        out = []
        for i, L in enumerate(lines):
            P = np.asarray(L) + offset
            if (P[0] - P[-1]) @ self.fa['d'] > 0:            # proximal end first
                P = P[::-1]
            if G.arclen(P)[-1] < 4e-3:
                continue
            P = G.resample(P)
            W = self.carriers(P, kind, None, None, None, name)
            cid, P, N = self.finish('%s #%d' % (name, i + 1) if len(lines) > 1 else name, kind, P,
                                    np.full(len(P), 1e-3), W, dict(structure=name, ends='fixed'))
            out.append((cid, P, N, None))
        return out
