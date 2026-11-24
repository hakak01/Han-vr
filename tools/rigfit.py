"""Geometry fits for the rig: volume/centroid, PCA, sphere, cylinder, saddle quadric, components."""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse.csgraph import connected_components
from scipy.sparse import coo_matrix


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def merge(prims, mat=None):
    """Concatenate primitives (optionally only material `mat`); returns verts, faces."""
    vs, fs, o = [], [], 0
    for v, f, m in prims:
        if mat is not None and m != mat:
            continue
        vs.append(v); fs.append(f + o); o += len(v)
    if not vs:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    return np.vstack(vs), np.vstack(fs)


def weld(v, f, tol=1e-6):
    """Merge coincident vertices (primitives share no indices)."""
    key = np.round(v / tol).astype(np.int64)
    _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return v[idx], inv.reshape(-1)[f]


def volume_centroid(v, f):
    """Signed volume and volumetric centroid of a closed triangle mesh."""
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    vol6 = np.einsum('ij,ij->i', a, np.cross(b, c))
    V = vol6.sum() / 6.0
    C = ((a + b + c) * vol6[:, None]).sum(0) / (24.0 * V)
    return abs(V), C


def components(v, f, min_faces=10):
    """Connected components (after welding) -> list of vertex arrays."""
    w, fw = weld(v, f)
    n = len(w)
    e = np.vstack([fw[:, [0, 1]], fw[:, [1, 2]], fw[:, [2, 0]]])
    g = coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n))
    k, lab = connected_components(g, directed=False)
    flab = lab[fw[:, 0]]
    out = []
    for i in range(k):
        if (flab == i).sum() >= min_faces:
            out.append(w[np.unique(fw[flab == i])])
    return out


def pca_axis(p, toward=None):
    c = p.mean(0)
    _, _, vt = np.linalg.svd(p - c, full_matrices=False)
    a = vt[0]
    if toward is not None and np.dot(a, toward) < 0:
        a = -a
    return c, a


def fit_sphere(p):
    A = np.hstack([2 * p, np.ones((len(p), 1))])
    b = (p ** 2).sum(1)
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    c = x[:3]
    r = np.sqrt(x[3] + c @ c)
    res = np.linalg.norm(p - c, axis=1) - r
    return c, r, float(np.sqrt((res ** 2).mean()))


def _perp(a):
    t = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    u = unit(np.cross(a, t))
    return u, np.cross(a, u)


def fit_cylinder(p, a0, c0=None, perp_to=None):
    """Least-squares cylinder; a0 initial axis direction; perp_to: keep the axis normal to this.
    -> point on axis, axis, radius, rms."""
    a0 = unit(a0)
    if perp_to is not None:
        L = unit(perp_to)
        a0 = unit(a0 - (a0 @ L) * L)
    c0 = p.mean(0) if c0 is None else np.asarray(c0, float)
    u, w = _perp(a0)
    q = p - c0
    pr = np.c_[q @ u, q @ w]
    # algebraic circle fit in the plane normal to a0 for the start values
    A = np.hstack([2 * pr, np.ones((len(pr), 1))])
    x = np.linalg.lstsq(A, (pr ** 2).sum(1), rcond=None)[0]
    r0 = np.sqrt(max(x[2] + x[:2] @ x[:2], 1e-8))

    def axis(t):
        if perp_to is not None:
            b = unit(np.cross(L, a0))
            return unit(np.cos(t[0]) * a0 + np.sin(t[0]) * b + 0 * t[1])
        return unit(a0 + t[0] * u + t[1] * w)

    def res(t):
        a = axis(t)
        c = c0 + t[2] * u + t[3] * w
        d = p - c
        d = d - np.outer(d @ a, a)
        return np.linalg.norm(d, axis=1) - t[4]
    s = least_squares(res, [0, 0, x[0], x[1], r0], loss='soft_l1', f_scale=2e-4)
    a = axis(s.x)
    c = c0 + s.x[2] * u + s.x[3] * w
    c = c + ((p - c) @ a).mean() * a        # centre the axis point on the surface patch
    return c, a, float(s.x[4]), float(np.sqrt((res(s.x) ** 2).mean()))


def fit_saddle(p, n0):
    """Quadric z = A u^2 + B uv + C v^2 + D u + E v + F in a frame with z ~ n0 (outward normal).
    -> apex point, normal, [(k1, e1), (k2, e2)] principal curvatures (+ = convex outward)."""
    n0 = unit(n0)
    c = p.mean(0)
    u, v = _perp(n0)
    q = p - c
    U, Vv, Z = q @ u, q @ v, q @ n0
    M = np.c_[U * U, U * Vv, Vv * Vv, U, Vv, np.ones_like(U)]
    A, B, C, D, E, F = np.linalg.lstsq(M, Z, rcond=None)[0]
    H = np.array([[2 * A, B], [B, 2 * C]])
    # stationary-free: evaluate at the patch centre (u=v=0); include slope via normal
    nrm = unit(n0 - D * u - E * v)
    apex = c + F * n0
    k, ev = np.linalg.eigh(H)
    out = []
    for i in range(2):
        e = unit(ev[0, i] * u + ev[1, i] * v)
        e = unit(e - (e @ nrm) * nrm)
        out.append((float(-k[i]), e))   # surface bending away from the normal = convex
    return apex, nrm, out


def rotvec(w):
    """Rodrigues rotation matrix from rotation vector (radians)."""
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def axis_angle(a, deg):
    return rotvec(unit(a) * np.radians(deg))


def signed_angle(v1, v2, a):
    """Angle (deg) from v1 to v2 about axis a, after projecting onto the plane normal to a."""
    a = unit(a)
    p1 = v1 - (v1 @ a) * a
    p2 = v2 - (v2 @ a) * a
    return float(np.degrees(np.arctan2(np.cross(p1, p2) @ a, p1 @ p2)))
