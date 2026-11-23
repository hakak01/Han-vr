"""Subcutaneous fat: a shell under each skin region, filling the gap down to the deep
structures (fascia, retinacula, muscle, tendon, sheath, ligament, bone). Thickness = local gap."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
import gapgeom as G
import glbio
from gapgeom import MM

GROUPS = {
    'Subcutaneous fat of hand and fingers': ['Palm', 'Dorsum of hand', 'Dorsal surfaces of digits of hand',
                                             'Palmar surfaces of digits of hand'],
    'Subcutaneous fat of wrist and forearm': ['Anterior region of wrist', 'Posterior region of wrist',
                                              'Anterior region of forearm', 'Posterior region of forearm',
                                              'Lateral border of forearm', 'Medial border of forearm', 'Radial foveola'],
}
SKIP = {'Nail plate', 'Perionyx'}
GAP_SKIN = 0.1 * MM          # fat starts just under the skin
GAP_DEEP = 0.4 * MM          # and stops just short of the deep structures
TMIN, TMAX = 0.3 * MM, 10 * MM


def shell(v, f, deep_tree):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    fn = np.cross(b - a, c - a); fc = (a + b + c) / 3
    _, qi = deep_tree.query(fc)
    inner = np.einsum('ij,ij->i', fn, deep_tree.data[qi] - fc) > 0
    frac = inner.mean()
    if 0.25 < frac < 0.75:           # solidified skin: keep the inner surface (normals face inwards)
        f, sign = f[inner], 1.0
    else:                            # single sheet
        sign = 1.0 if frac >= 0.75 else -1.0
    used = np.unique(f); remap = -np.ones(len(v), int); remap[used] = np.arange(len(used))
    v, f = v[used], remap[f]
    n = sign * glbio.vertex_normals(v, f)
    d = deep_tree.query(v + n * GAP_SKIN)[0]
    t = np.clip(d - GAP_DEEP - GAP_SKIN, TMIN, TMAX)
    e = np.vstack([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    A = coo_matrix((np.ones(len(e) * 2), (np.r_[e[:, 0], e[:, 1]], np.r_[e[:, 1], e[:, 0]])), shape=(len(v),) * 2).tocsr()
    A.data[:] = 1; deg = np.asarray(A.sum(1)).ravel() + 1
    for _ in range(10):
        t = np.minimum(t, (A @ t + t) / deg + 0.3 * MM)    # smooth, never thicker than the gap allows
    for _ in range(4):
        t = (A @ t + t) / deg
    outer = v + n * GAP_SKIN; inner_v = v + n * (GAP_SKIN + t)[:, None]
    nv = len(v)
    V = np.vstack([outer, inner_v])
    F = [f[:, ::-1] if sign > 0 else f, (f if sign > 0 else f[:, ::-1]) + nv]
    # boundary walls
    E = np.vstack([F[0][:, [0, 1]], F[0][:, [1, 2]], F[0][:, [2, 0]]])
    key = np.sort(E, 1); _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    B = E[cnt[inv.ravel()] == 1]
    if len(B):
        a_, b_ = B[:, 0], B[:, 1]
        F += [np.c_[b_, a_, a_ + nv], np.c_[b_, a_ + nv, b_ + nv]]
    F = np.vstack(F)
    # outer faces must face away from the deep side
    o = F[:len(f)]
    on = np.cross(V[o[:, 1]] - V[o[:, 0]], V[o[:, 2]] - V[o[:, 0]])
    if np.einsum('ij,ij->', on, n[o[:, 0]]) > 0:
        F = F[:, ::-1]
    return V, F, t


def build(get, skin_names, deep_pts):
    tree = cKDTree(deep_pts)
    skin = [n for n in skin_names if n not in SKIP]
    groups = dict(GROUPS)
    taken = {x for g in GROUPS.values() for x in g}
    groups['Subcutaneous fat of elbow and arm'] = [n for n in skin if n not in taken]
    out, stats = [], {}
    for name, regions in groups.items():
        parts = []
        for rn in regions:
            if rn not in skin:
                continue
            V, F, t = shell(*get(rn), tree)
            parts.append((V, F)); stats[rn] = (np.median(t) / MM, t.max() / MM)
        if parts:
            out.append(dict(name=name, layer='fat', mat='Fat', mesh=G.merge(parts),
                            ref='filled to the skin-deep fascia gap'))
    return out, stats
