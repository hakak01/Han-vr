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
        for i, a in enumerate(c.get('arc', [])):
            if a:
                A = self.partial(M, a[0], a[1], a[2] if len(a) > 2 else None)
                Pp[i] = A[:3, :3] @ P[i] + A[:3, 3]
                Np[i] = A[:3, :3] @ N[i]
        T = G.tangents(Pp)
        Np = G.unit(Np - (Np * T).sum(1, keepdims=True) * T)
        sr, sp = G.arclen(P), G.arclen(Pp)
        info = dict(Lr=sr[-1], Lp=sp[-1])
        if c.get('common'):                                  # fused sheet: slides with the mean of its tendons
            d = np.mean([self.dL[m] for m in c['members']])
            info.update(dT=0.0, sigma=lambda u: u + d)
        elif c['kind'] == 'tendon':
            umtj = c['u_mtj']
            self.dL[c['id']] = sp[-1] - sr[-1]
            dT = (sp[-1] - np.interp(umtj, sr, sp)) - (sr[-1] - umtj)      # tendon path change (m), < 0 = pulled proximally
            info.update(dT=dT, excursion=-dT, sigma=lambda u: u + sp[-1] - sr[-1])
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

    def partial(self, M, child, f, parent=None):
        """World matrix of a joint-arc sample: parent @ (fraction f of parent->child motion about the arc centre)."""
        aj = self.cab['arc_joints'][child]
        Mp, Mc = M[self.joints.index(parent or aj['parent'])], M[self.joints.index(child)]
        rel = np.linalg.inv(Mp) @ Mc
        c = np.array(aj['center'])
        Rf = Rotation.from_rotvec(Rotation.from_matrix(rel[:3, :3]).as_rotvec() * f).as_matrix()
        mc = rel[:3, :3] @ c + rel[:3, 3]
        out = np.eye(4)
        out[:3, :3] = Rf
        out[:3, 3] = c + f * (mc - c) - Rf @ c
        return Mp @ out

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
                            dT = (sp[-1] - np.interp(u, sr, sp)) - (sr[-1] - u)
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
