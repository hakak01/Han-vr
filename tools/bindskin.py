"""Phase E2: skin weights for bind_tissues.py.
Joints = the 30 rig bones + 3 forearm twist segments. All weights are per vertex (dense N x NJ arrays).
Methods: rigid (one bone), ray (distance to bone segments of the same digit ray + smoothing),
liga (bones the structure touches, blended by distance), twist split of forearm soft tissue."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix, diags
import rigfit as F

ORD = ['first', 'second', 'third', 'fourth', 'fifth']
CARP = ['Scaphoid bone', 'Lunate bone', 'Triquetrum bone', 'Pisiform bone', 'Capitate bone',
        'Hamate bone', 'Trapezoid bone', 'Trapezium bone']
FORE = ['Humerus', 'Ulna', 'Radius']


def mc(i):
    return '%s metacarpal bone' % ORD[i - 1].capitalize()


def ph(kind, i):
    return '%s phalanx of %s finger of hand' % (kind, ORD[i - 1])


def ray(i):
    return [mc(i)] + [ph(k, i) for k in (['Proximal', 'Distal'] if i == 1 else ['Proximal', 'Middle', 'Distal'])]


BONES = FORE + CARP + [b for i in range(1, 6) for b in ray(i)]
TWIST = ['Forearm twist 1', 'Forearm twist 2', 'Forearm twist 3']
TWIST_S = [0.25, 0.5, 0.75]            # position along the forearm (0 elbow, 1 wrist) = rotation fraction
JOINTS = BONES + TWIST
JI = {n: i for i, n in enumerate(JOINTS)}
NJ = len(JOINTS)
RAY_OF = {b: i for i in range(1, 6) for b in ray(i)}


class BoneShapes:
    """Bone = segment (long bones) or ball (carpals) with a mean radius; plus a surface KD-tree."""

    def __init__(self, D):
        self.seg, self.tree, self.pts = {}, {}, {}
        for b in BONES:
            v, f = F.weld(*F.merge(D[b]))
            self.pts[b] = v
            self.tree[b] = cKDTree(v)
            c = v.mean(0)
            if b in CARP:
                r = np.median(np.linalg.norm(v - c, axis=1))
                self.seg[b] = (c, c, r)
                continue
            _, a = F.pca_axis(v)
            s = (v - c) @ a
            lo, hi = np.percentile(s, [2, 98])
            mid = (s > lo + 0.2 * (hi - lo)) & (s < hi - 0.2 * (hi - lo))
            r = np.median(np.linalg.norm(np.cross(v[mid] - c, a), axis=1))
            self.seg[b] = (c + a * (lo + r), c + a * (hi - r), r)

    def dist(self, p, b):
        a, e, r = self.seg[b]
        d = e - a
        t = np.clip(((p - a) @ d) / max(d @ d, 1e-12), 0, 1)
        return np.maximum(np.linalg.norm(p - (a + t[:, None] * d), axis=1) - r, 0)

    def surf(self, p, b):
        return self.tree[b].query(p)[0]


def nearest_bone(S, p, names=BONES, surface=True):
    d = np.stack([S.surf(p, b) if surface else S.dist(p, b) for b in names], 1)
    return np.array(names)[d.argmin(1)], d.min(1)


def candidates(b):
    """Bones allowed to influence a vertex whose nearest bone is b."""
    if b in RAY_OF and 'phalanx' in b:
        i = RAY_OF[b]
        return ray(i) + (['Trapezium bone'] if i == 1 else [])
    if b in RAY_OF:                                    # metacarpal: palm
        return [mc(i) for i in range(1, 6)] + [ph('Proximal', i) for i in range(1, 6)] + CARP
    return FORE + CARP + [mc(i) for i in range(1, 6)]


def ray_weights(S, p, power=4, soft=1e-3):
    """Inverse-distance weights to bone segments, restricted to the nearest bone's ray."""
    W = np.zeros((len(p), NJ))
    if not len(p):
        return W
    D = np.stack([S.dist(p, b) for b in BONES], 1)
    near = np.array(BONES)[D.argmin(1)]
    for b in set(near):
        m = near == b
        cs = candidates(b)
        idx = [BONES.index(c) for c in cs]
        w = 1.0 / (D[np.ix_(m, idx)] + soft) ** power
        W[np.ix_(m, [JI[c] for c in cs])] = w / w.sum(1, keepdims=True)
    return W


def liga_weights(S, p, touch=1.5e-3, power=4, fixed=0.7e-3):
    """Ligaments/capsules: bones touched by the structure; vertices on a bone are fixed to it,
    the rest blended by surface distance between the touched bones."""
    W = np.zeros((len(p), NJ))
    d = {b: S.surf(p, b) for b in BONES}
    touched = [b for b in BONES if (d[b] < touch).sum() >= 3]
    if not touched:
        return None, []
    Dm = np.stack([d[b] for b in touched], 1)
    w = 1.0 / (Dm + 0.3e-3) ** power
    on = Dm.min(1) < fixed
    w[on] = 0
    w[on, Dm[on].argmin(1)] = 1
    W[:, [JI[b] for b in touched]] = w / w.sum(1, keepdims=True)
    return W, touched


def rigid_weights(p, bones):
    W = np.zeros((len(p), NJ))
    W[np.arange(len(p)), [JI[b] for b in bones]] = 1
    return W


def weld_graph(P, faces_list, tol=2e-5):
    """Global vertex graph over several meshes: coincident vertices (within tol) become one node.
    P: all vertices (N x 3); faces_list: faces already offset into P. -> (node of each vertex, row-normalised adjacency)."""
    key = np.round(P / tol).astype(np.int64)
    _, node = np.unique(key, axis=0, return_inverse=True)
    node = node.reshape(-1)
    n = node.max() + 1
    f = np.vstack(faces_list) if faces_list else np.zeros((0, 3), int)
    f = node[f]
    e = np.vstack([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    e = e[e[:, 0] != e[:, 1]]
    A = coo_matrix((np.ones(2 * len(e)), (np.r_[e[:, 0], e[:, 1]], np.r_[e[:, 1], e[:, 0]])), shape=(n, n)).tocsr()
    A.data[:] = 1
    deg = np.asarray(A.sum(1)).ravel()
    return node, diags(1 / np.maximum(deg, 1)) @ A, deg > 0


def smooth(W, node, A, has, passes=10, keep=None):
    """Average weights over coincident vertices, then `passes` Laplacian passes (w = 0.5 w + 0.5 mean of neighbours).
    keep: boolean mask of vertices whose weights must not change (e.g. fixed ligament ends)."""
    n = A.shape[0]
    cnt = np.bincount(node, minlength=n)[:, None]
    Wn = np.zeros((n, W.shape[1]))
    np.add.at(Wn, node, W)
    Wn /= np.maximum(cnt, 1)
    for _ in range(passes):
        Wn = np.where(has[:, None], 0.5 * Wn + 0.5 * (A @ Wn), Wn)
    out = Wn[node]
    if keep is not None:
        out[keep] = W[keep]
    return out


def spatial_smooth(P, W, r, passes):
    """Average weights over all vertices within r (crosses separate meshes, e.g. skin regions)."""
    nb = cKDTree(P).query_ball_point(P, r)
    rows = np.repeat(np.arange(len(P)), [len(x) for x in nb])
    A = coo_matrix((np.ones(len(rows)), (rows, np.concatenate(nb))), shape=(len(P), len(P))).tocsr()
    A = diags(1 / np.asarray(A.sum(1)).ravel()) @ A
    for _ in range(passes):
        W = A @ W
    return W


def twist_split(W, p, fa, uniform=None):
    """Forearm soft tissue: move the Radius weight onto twist joints by position along the forearm.
    uniform (mask): superficial layers also move their Ulna weight, so they twist evenly around the forearm."""
    s = np.clip(((p - fa['c']) @ fa['d']) / fa['len'], 0, 1)
    if uniform is not None:
        W[uniform, JI['Radius']] += W[uniform, JI['Ulna']]
        W[uniform, JI['Ulna']] = 0
    knots = [0.0] + TWIST_S + [1.0]
    tgt = [JI['Ulna']] + [JI[t] for t in TWIST] + [JI['Radius']]
    r = W[:, JI['Radius']].copy()
    W[:, JI['Radius']] = 0
    k = np.clip(np.searchsorted(knots, s, side='right') - 1, 0, len(knots) - 2)
    t = (s - np.array(knots)[k]) / np.diff(knots)[k]
    np.add.at(W, (np.arange(len(p)), np.array(tgt)[k]), r * (1 - t))
    np.add.at(W, (np.arange(len(p)), np.array(tgt)[k + 1]), r * t)
    return W


def top4(W):
    """-> joints (N x 4 int), weights (N x 4 float, sum 1)."""
    j = np.argsort(-W, 1)[:, :4]
    w = np.take_along_axis(W, j, 1)
    w[w < 0.01] = 0
    s = w.sum(1, keepdims=True)
    bad = s[:, 0] <= 0
    w[bad, 0], s[bad] = 1, 1
    w = w / s
    j[w == 0] = 0
    return j.astype(np.uint8), w.astype(np.float32)
