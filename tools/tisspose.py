"""Phase E2 reference poser for hand_rig.glb + cables.json (the JS version in E3 must match this).
pose dict -> joint matrices -> skinned vertices, posed cables (tendon slide, nerve glide/slack), belly slide + bulge."""
import json, os
import numpy as np
from scipy.spatial.transform import Rotation
import rigpose as RP, rigfit as F, cablegeom as G, glbskin

ROOT = RP.ROOT


class Rig:
    def __init__(self, raw=os.path.join(ROOT, 'data/raw/hand_rig_raw.glb')):
        self.rig, self.cm = RP.load()
        self.cab = json.load(open(os.path.join(ROOT, 'assets/cables.json')))
        self.gl, self.M = glbskin.read(raw)
        self.joints = self.cab['joints']

    def matrices(self, pose):
        W = RP.solve(self.rig, self.cm, pose)
        fa = self.cab['forearm']
        rest = self.rig['dofs']['pronation']['rest']
        sign = self.rig['joints'][0]['axes'][0].get('sign', 1)
        ang = (RP.dof_value(self.rig, pose, 'pronation') - rest) * sign
        for t in self.cab['twist']:
            W[t['joint']] = W['Ulna'] @ RP.about(fa['center'], F.axis_angle(fa['axis'], t['fraction'] * ang))
        return np.stack([W[j] for j in self.joints])

    @staticmethod
    def skin(M, v, J, Wt):
        out = np.zeros_like(v)
        for k in range(4):
            Mk = M[J[:, k]]
            out += Wt[:, k:k + 1] * (np.einsum('nij,nj->ni', Mk[:, :3, :3], v) + Mk[:, :3, 3])
        return out

    def cable(self, M, c):
        """-> posed samples, normals, rest arc, posed arc, and the material map sigma(u) (posed arc of rest point u)."""
        P, N = np.array(c['pts']), np.array(c['nrm'])
        J, Wt = np.array(c['j']), np.array(c['w'])
        Pp = self.skin(M, P, J, Wt)
        Mr = np.zeros((len(P), 3, 3))
        for k in range(4):
            Mr += Wt[:, k, None, None] * M[J[:, k], :3, :3]
        Np = np.einsum('nij,nj->ni', Mr, N)
        zd = [(z,) + self.zone(M, c, z, P, N, Pp, Np) for z in c.get('zones', [])]   # (zone, V, E)
        T = G.tangents(Pp)
        Np = G.unit(Np - (Np * T).sum(1, keepdims=True) * T)
        sr, sp = G.arclen(P), G.arclen(Pp)
        info = dict(Lr=sr[-1], Lp=sp[-1])
        if c.get('common'):                                  # fused sheet: slides with the mean of its tendons
            d = np.mean([self.dL[m] for m in c['members']])
            info.update(dT=0.0, sigma=lambda u: u + d)
        elif c['kind'] == 'tendon':
            # material map: insertion fixed; outside zones the tendon keeps its rest length (any visual stretch is
            # strain), a zone holds (rest length - mechanical shortening E) spread like its visual length; a shortfall
            # (zone shorter than E, e.g. DIP near the insertion) is carried to the tendon just proximal to it
            dr, dp = np.diff(sr), np.diff(sp)
            dm = dr.copy()
            for z, V, E in zd:
                a, b = z['i0'] - 1, z['i1'] + 1
                dm[a:b] = dp[a:b] * (dr[a:b].sum() - E) / max(dp[a:b].sum(), 1e-9)
            lo, carry = 0.2 * dr, 0.0
            for k in range(len(dm) - 1, -1, -1):
                dm[k] += carry
                carry = min(dm[k] - lo[k], 0.0)
                dm[k] -= carry
            m = sr[-1] - np.r_[np.cumsum(dm[::-1])[::-1], 0.0]
            def sigma(u, m=m, sp=sp):
                u = np.asarray(u, float)
                return np.where(u < m[0], sp[0] + u - m[0], np.where(u > m[-1], sp[-1] + u - m[-1], np.interp(u, m, sp)))
            slide = lambda u, m=m: u - np.interp(u, sr, m)                  # slide past the carrying bone, < 0 = proximal
            dT = float(slide(0.0))                                           # = - sum of zone E (whole muscle-tendon unit)
            self.dL[c['id']] = dT
            info.update(dT=dT, excursion=-dT, sigma=sigma, slide=slide, zones={z['child']: (V, E) for z, V, E in zd})
        else:
            if sp[-1] >= sr[-1]:                                             # taut: uniform glide
                info['sigma'] = lambda u: u * sp[-1] / sr[-1]
            else:                                                            # slack: wave where the bed shortened
                info['sigma'] = lambda u: np.interp(u, sr, sp)
                ratio = np.diff(sp) / np.maximum(np.diff(sr), 1e-9)
                ratio = np.convolve(np.r_[ratio[0], ratio, ratio[-1]], np.ones(5) / 5, 'same')[1:]
                lam = self.cab['glide']['wave_m']
                A = np.minimum(lam / np.pi * np.sqrt(np.maximum(1 / np.minimum(ratio, 1) - 1, 0)), self.cab['glide']['wave_max_m'])
                A[0] = A[-1] = 0
                Pp = Pp + (A * np.sin(2 * np.pi * sp / lam))[:, None] * Np
                info['wave_mm'] = float(A.max() * 1e3)
        return Pp, Np, sr, sp, info

    def zone(self, M, c, z, P, N, Pp, Np):
        """E2b: samples i0..i1 between the zone ends a = i0-1 and b = i1+1 (in place), in polar coordinates about
        the joint (axis n = relative rotation axis through the arc centre, parent frame): each sample keeps its
        distance from the axis (tendon held on the joint by pulleys/retinacula) and takes the ends' changes of
        angle, radius and axial position in proportion to its rest angle between them. Monotone (no loops) while
        the joint angle is below the zone's angular span (zones widened for this in bindcable).
        -> (visual shortening V, mechanical shortening E) of the zone (m)."""
        ce = np.array(self.cab['arc_joints'][z['child']]['center'])
        Mp = M[self.joints.index(z['parent'])]
        iMp = np.linalg.inv(Mp)
        rel = iMp @ M[self.joints.index(z['child'])]
        w = Rotation.from_matrix(rel[:3, :3]).as_rotvec()
        a, b = z['i0'] - 1, z['i1'] + 1
        idx = np.arange(z['i0'], z['i1'] + 1)
        s0 = G.arclen(P[a:b + 1])
        th = np.linalg.norm(w)
        n = w / th if th > 1e-4 else G.unit(np.cross(P[b] - P[a], P[idx].mean(0) - ce))
        Q = np.vstack([P[a:b + 1], [iMp[:3, :3] @ Pp[a] + iMp[:3, 3], iMp[:3, :3] @ Pp[b] + iMp[:3, 3]]]) - ce
        zz = Q @ n
        d = Q - np.outer(zz, n)
        e1 = G.unit(d[np.linalg.norm(d[:-2], axis=1).argmax()])
        e2 = np.cross(n, e1)
        r, be = np.linalg.norm(d, axis=1), np.arctan2(d @ e2, d @ e1)
        be[1:-2] = be[0] + np.cumsum((np.diff(be[:-2]) + np.pi) % (2 * np.pi) - np.pi)   # unwrap along the cable
        be[-2:] = be[[0, -3]] + (be[-2:] - be[[0, -3]] + np.pi) % (2 * np.pi) - np.pi     # posed ends near rest ends
        span = be[-3] - be[0]
        t = (be[1:-3] - be[0]) / span if abs(span) > 0.05 else s0[1:-1] / s0[-1]
        t = np.clip(t, 0, 1)
        dA, dB = np.array([be[-2] - be[0], r[-2] - r[0], zz[-2] - zz[0]]), np.array([be[-1] - be[-3], r[-1] - r[-3], zz[-1] - zz[-3]])
        dl = np.outer(1 - t, dA) + np.outer(t, dB)
        bi, ri, zi = be[1:-3] + dl[:, 0], np.maximum(r[1:-3] + dl[:, 1], 0), zz[1:-3] + dl[:, 2]
        X = ce + np.outer(ri * np.cos(bi), e1) + np.outer(ri * np.sin(bi), e2) + np.outer(zi, n)
        Pp[idx] = X @ Mp[:3, :3].T + Mp[:3, 3]
        Rf = Rotation.from_rotvec(np.outer(dl[:, 0], n)).as_matrix()
        Np[idx] = np.einsum('ij,njk,nk->ni', Mp[:3, :3], Rf, N[idx])
        V = s0[-1] - G.arclen(Pp[a:b + 1])[-1]
        mech = c.get('mech')
        if mech is None or not z['dofs']:
            return V, V
        E = 0.0
        for dof in z['dofs']:
            if dof in mech:
                t = mech[dof]
                F = lambda q: np.interp((q - t['deg0']) / t['step'], np.arange(len(t['F'])), t['F'])
                E += F(RP.dof_value(self.rig, self._pose, dof)) - F(self.rig['dofs'][dof]['rest'])
        return V, float(E)

    def frame_at(self, Pp, Np, sigma):
        c, i, f = G.at(Pp, sigma)
        T = G.tangents(Pp)
        t = G.unit(T[i] * (1 - f)[:, None] + T[i + 1] * f[:, None])
        n = Np[i] * (1 - f)[:, None] + Np[i + 1] * f[:, None]
        n = G.unit(n - (n * t).sum(1, keepdims=True) * t)
        s = G.arclen(Pp)
        under = sigma < 0
        c[under] = Pp[0] + np.outer(sigma[under], t[0]) if under.any() else c[under]
        return c, t, n, np.cross(t, n)

    def pose(self, pose, names=None):
        """-> ({name: [posed vertex arrays per primitive]}, {cable name: info})"""
        M = self.matrices(pose)
        self._pose = pose
        cabs, self.dL = {}, {}
        for c in sorted(self.cab['cables'], key=lambda c: bool(c.get('common'))):
            cabs[c['id']] = (c,) + self.cable(M, c)
        mus = {}                                             # belly excursion = mean over the muscle's tendons
        for k, x in cabs.items():
            if x[0]['kind'] == 'tendon' and not x[0].get('common'):
                mus.setdefault(x[0]['muscle'], []).append(x[5]['dT'])
        out = {}
        lat = self.cab['lateral_bands']
        for n, prims in self.M.items():
            if names and n not in names:
                continue
            res = []
            for p in prims:
                a = p['attrs']
                v = self.skin(M, p['v'], a['JOINTS_0'].astype(int), a['WEIGHTS_0'].astype(float))
                if '_SLIDE' in a:
                    sl = a['_SLIDE'].astype(float)
                    for cid in set(sl[:, 0].astype(int)) - {-1}:
                        m = sl[:, 0] == cid
                        c, Pp, Np, sr, sp, info = cabs[cid]
                        u = sl[m, 1]
                        if sl[m, 3].min() < 0:                       # lumbrical: moves with the FDP material at u
                            dT = info['slide'](u)
                        else:                                        # belly: mean excursion of the muscle's tendons
                            dT = float(np.mean(mus[c['muscle']]))
                        if sl[m, 3].min() < 0:                       # lumbrical: along its FDP cable at u
                            P0 = np.array(c['pts'])
                            c0, i0, f0 = G.at(P0, u)
                            T0 = G.tangents(P0)
                            t0 = G.unit(T0[i0] * (1 - f0)[:, None] + T0[i0 + 1] * f0[:, None])
                            off = (sl[m, 2] * dT)[:, None] * t0
                        else:                                        # belly: straight muscle axis, bulge
                            mu = self.cab['muscles'][c['muscle']]
                            O = np.array(mu['origin'])
                            t0 = G.unit(np.array(mu['junction']) - O)
                            d = p['v'][m] - O
                            perp = d - np.outer(d @ t0, t0)
                            bul = 1 / np.sqrt(max((mu['L'] + dT) / mu['L'], 0.3)) - 1
                            off = np.outer(sl[m, 2] * dT, t0) + (bul * sl[m, 3])[:, None] * perp
                        Rv = np.zeros((m.sum(), 3, 3))
                        J, Wt = a['JOINTS_0'][m].astype(int), a['WEIGHTS_0'][m].astype(float)
                        for q in range(4):
                            Rv += Wt[:, q, None, None] * M[J[:, q], :3, :3]
                        v[m] = v[m] + np.einsum('nij,nj->ni', Rv, off)
                if '_CID' in a:
                    cid = a['_CID'].astype(int)
                    cb = a['_CABLE'].astype(float)
                    for k in set(cid) - {-1}:
                        m = cid == k
                        c, Pp, Np, sr, sp, info = cabs[k if k >= 0 else -k - 2]
                        sg = info['sigma'](cb[m, 0]) if k >= 0 else np.interp(cb[m, 0], sr, sp)
                        cc, t, nn, bb = self.frame_at(Pp, Np, sg)
                        v[m] = cc + nn * cb[m, 1:2] + bb * cb[m, 2:3] + t * cb[m, 3:4]
                if '_SHIFT' in a and n in lat:
                    L = lat[n]
                    R = M[self.joints.index(L['bone'])][:3, :3]
                    pip = RP.dof_value(self.rig, pose, L['dof'])
                    v = v + np.outer(a['_SHIFT'] * L['max_m'] * np.clip(pip / 90, 0, 1), R @ np.array(L['dir']))
                res.append(v)
            out[n] = res
        info = {cabs[k][0]['name']: cabs[k][5] for k in cabs}
        return out, info, M
