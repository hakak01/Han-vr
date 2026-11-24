"""Pose engine for assets/rig.json (reference for js/rig.js) + bone-bone overlap test.
Pose = {dof id: anatomical degrees}. Missing dofs stay at their rest value.
Bone world matrix = parent world matrix @ joint matrix (all in rest/world coordinates)."""
import json, os
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import rigfit as F

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load():
    rig = json.load(open(os.path.join(ROOT, 'assets/rig.json')))
    cm = json.load(open(os.path.join(ROOT, 'assets/carpal_model.json')))
    return rig, cm


def about(c, R):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(c) - R @ np.asarray(c)
    return M


def quad(p, fe, ru):
    return p[0] * ru + p[1] * fe + p[2] * ru * ru + p[3] * ru * fe + p[4] * fe * fe


def carpal_motion(rig, cm, model, fe, ru):
    """Rotation (3x3) and centroid translation (m) of one carpal bone from dataset neutral."""
    W = rig['wrist']
    co = cm['bones'][model]
    w = np.radians(quad(co['FE'], fe, ru)) * np.array(W['flex_axis']) + \
        np.radians(quad(co['RU'], fe, ru)) * np.array(W['ulnar_axis'])   # SP = 0
    t = W['scale_m'] * (quad(co['DPT'], fe, ru) * np.array(W['distal']) +
                        quad(co['RUT'], fe, ru) * np.array(W['ulnar']) +
                        quad(co['VDT'], fe, ru) * np.array(W['dorsal']))
    return F.rotvec(w), t


def carpal_rt(rig, cm, bone, fe, ru):
    """Table S1 rotation (SP = 0) and centroid translation + contact correction (bilinear in the W['corr'] grid)."""
    W = rig['wrist']
    b = W['bones'][bone]
    R, t = carpal_motion(rig, cm, b['model'], fe, ru)
    C = W.get('corr')
    if C and bone in C['bones']:
        g = np.array(C['bones'][bone])                     # [nFE][nRU][3]
        x = np.interp(fe, C['fe'], np.arange(len(C['fe'])))
        y = np.interp(ru, C['ru'], np.arange(len(C['ru'])))
        i, j = min(int(x), len(C['fe']) - 2), min(int(y), len(C['ru']) - 2)
        u, v = x - i, y - j
        def bil(g):
            return (g[i, j] * (1 - u) * (1 - v) + g[i + 1, j] * u * (1 - v) +
                    g[i, j + 1] * (1 - u) * v + g[i + 1, j + 1] * u * v)
        t = t + bil(g)
        if bone in C.get('rot', {}):                       # rotation correction about the centroid
            R = F.rotvec(bil(np.array(C['rot'][bone]))) @ R
    return R, t


def carpal_matrix(rig, cm, bone, fe, ru):
    """Rest-pose geometry -> pose (fe, ru), relative to the radius."""
    b = rig['wrist']['bones'][bone]
    W = rig['wrist']
    R, t = carpal_rt(rig, cm, bone, fe, ru)
    R0, t0 = carpal_rt(rig, cm, bone, W['rest']['wrist_fe'], W['rest']['wrist_ru'])
    c = np.array(b['centroid'])
    M = np.eye(4)
    M[:3, :3] = R @ R0.T
    M[:3, 3] = c - t0 + t - M[:3, :3] @ c
    return M


def dof_value(rig, pose, d):
    return pose.get(d, rig['dofs'][d]['rest'])


def joint_matrix(rig, cm, j, pose):
    if j['type'] == 'carpal':
        return carpal_matrix(rig, cm, j['child'], dof_value(rig, pose, 'wrist_fe'),
                             dof_value(rig, pose, 'wrist_ru'))
    M = np.eye(4)
    for ax in j['axes']:            # first axis is fixed in the parent, later ones ride on the child
        d = ax['dof']
        ang = (dof_value(rig, pose, d) - rig['dofs'][d]['rest']) * ax.get('sign', 1)
        M = M @ about(ax['center'], F.axis_angle(ax['axis'], ang))
    return M


def solve(rig, cm, pose):
    """-> {bone: 4x4 world matrix}"""
    W = {b: np.eye(4) for b in rig['fixed']}
    for j in rig['joints']:          # stored in parent-before-child order
        W[j['child']] = W[j['parent']] @ joint_matrix(rig, cm, j, pose)
    return W


def apply(M, p):
    return p @ M[:3, :3].T + M[:3, 3]


class Overlap:
    """Signed-distance overlap between posed bones from dense surface samples."""

    def __init__(self, meshes, spacing=0.4e-3, reach=5e-3):
        self.S = {}
        self.imax, self.qmax = reach, reach + 1e-3     # deepest penetration that is detected
        rng = np.random.default_rng(0)
        for name, (v, f) in meshes.items():
            a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
            cr = np.cross(b - a, c - a)
            ar = np.linalg.norm(cr, axis=1) / 2
            n = max(int(ar.sum() / spacing ** 2), 200)
            fi = rng.choice(len(f), n, p=ar / ar.sum())
            r1, r2 = rng.random(n), rng.random(n)
            s = np.sqrt(r1)
            p = (1 - s)[:, None] * a[fi] + (s * (1 - r2))[:, None] * b[fi] + (s * r2)[:, None] * c[fi]
            nn = cr[fi] / np.maximum(np.linalg.norm(cr[fi], axis=1, keepdims=True), 1e-15)
            p = np.vstack([p, v])
            vn = np.zeros_like(v)
            for k in range(3):
                np.add.at(vn, f[:, k], cr)
            nn = np.vstack([nn, vn / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-15)])
            cen = p.mean(0)
            self.S[name] = dict(p=p, n=nn, tree=cKDTree(p), c=cen,
                                r=np.linalg.norm(p - cen, axis=1).max())

    def depth(self, A, MA, B, MB):
        """Max penetration (m) of A's samples into B (and B into A)."""
        out = 0.0
        for X, MX, Y, MY in ((A, MA, B, MB), (B, MB, A, MA)):
            sx, sy = self.S[X], self.S[Y]
            rel = np.linalg.inv(MY) @ MX
            cx = apply(rel, sx['c'][None])[0]
            if np.linalg.norm(cx - sy['c']) > sx['r'] + sy['r']:
                return 0.0
            p = apply(rel, sx['p'])
            near = np.linalg.norm(p - sy['c'], axis=1) < sy['r'] + 1e-3
            if not near.any():
                continue
            p = p[near]
            d, i = sy['tree'].query(p, distance_upper_bound=self.qmax)
            ok = np.isfinite(d)
            if not ok.any():
                continue
            sd = np.einsum('ij,ij->i', p[ok] - sy['p'][i[ok]], sy['n'][i[ok]])
            inside = (sd < 0) & (d[ok] < self.imax)
            if inside.any():
                out = max(out, float(-sd[inside].min()))
        return out

    def push(self, A, MA, B, MB):
        """World vector that moves A out of B (mean of the deepest contacts), max depth, contact point."""
        vecs, pts, dmax = [], [], 0.0
        for X, MX, Y, MY, sgn in ((A, MA, B, MB, 1), (B, MB, A, MA, -1)):
            sx, sy = self.S[X], self.S[Y]
            rel = np.linalg.inv(MY) @ MX
            p = apply(rel, sx['p'])
            near = np.linalg.norm(p - sy['c'], axis=1) < sy['r'] + 1e-3
            if not near.any():
                continue
            p = p[near]
            d, i = sy['tree'].query(p, distance_upper_bound=self.qmax)
            ok = np.isfinite(d)
            sd = np.einsum('ij,ij->i', p[ok] - sy['p'][i[ok]], sy['n'][i[ok]])
            ins = (sd < 0) & (d[ok] < self.imax)
            if ins.any():
                dep = -sd[ins]
                dmax = max(dmax, dep.max())
                n = sy['n'][i[ok]][ins] @ MY[:3, :3].T
                sel = dep > 0.5 * dep.max()
                vecs.append(sgn * (n[sel] * dep[sel, None]).mean(0))
                pts.append(apply(MY, p[ok][ins][sel]).mean(0))
        if not vecs:
            return np.zeros(3), dmax, None
        return np.mean(vecs, 0), dmax, np.mean(pts, 0)

    def all_pairs(self, W, names):
        res = {}
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                d = self.depth(a, W[a], b, W[b])
                if d > 0:
                    res[(a, b)] = d
        return res
