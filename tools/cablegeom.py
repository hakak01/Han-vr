"""Phase E2: polyline helpers for cables (resample, frames, projection, tendon tracing)."""
import numpy as np


def unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, 1e-15)


def arclen(P):
    return np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]


def resample(P, step=2e-3):
    s = arclen(P)
    n = max(2, int(round(s[-1] / step)) + 1)
    t = np.linspace(0, s[-1], n)
    return np.stack([np.interp(t, s, P[:, k]) for k in range(3)], 1)


def smooth_line(P, passes=3):
    P = P.copy()
    for _ in range(passes):
        P[1:-1] = 0.5 * P[1:-1] + 0.25 * (P[:-2] + P[2:])
    return P


def tangents(P):
    T = np.gradient(P, axis=0)
    return unit(T)


def frames(P, n0=None):
    """Parallel-transported normals N (perpendicular to the tangent) along P."""
    T = tangents(P)
    if n0 is None:
        n0 = [0, 0, 1] if abs(T[0][2]) < 0.9 else [1, 0, 0]
    N = np.zeros_like(P)
    n = np.asarray(n0, float)
    for i in range(len(P)):
        n = n - (n @ T[i]) * T[i]
        if np.linalg.norm(n) < 1e-9:
            n = np.cross(T[i], [1, 0, 0])
        n = n / np.linalg.norm(n)
        N[i] = n
    return T, N


def project(P, X, chunk=4000):
    """Nearest point of polyline P for each X -> (arc-length u, distance, segment index, t)."""
    s = arclen(P)
    A, B = P[:-1], P[1:]
    d = B - A
    dd = np.maximum((d * d).sum(1), 1e-18)
    U, Dist, Seg, Tt = [], [], [], []
    for k in range(0, len(X), chunk):
        x = X[k:k + chunk]
        t = np.clip(np.einsum('nsk,sk->ns', x[:, None, :] - A[None], d) / dd, 0, 1)
        q = A[None] + t[..., None] * d[None]
        dist = np.linalg.norm(x[:, None, :] - q, axis=2)
        j = dist.argmin(1)
        r = np.arange(len(x))
        U.append(s[j] + t[r, j] * np.sqrt(dd[j])); Dist.append(dist[r, j]); Seg.append(j); Tt.append(t[r, j])
    return np.concatenate(U), np.concatenate(Dist), np.concatenate(Seg), np.concatenate(Tt)


def at(P, u):
    """Point, tangent and sample interpolation (index, frac) at arc-length u."""
    s = arclen(P)
    u = np.clip(u, 0, s[-1])
    i = np.clip(np.searchsorted(s, u, side='right') - 1, 0, len(P) - 2)
    f = (u - s[i]) / np.maximum(s[i + 1] - s[i], 1e-15)
    return P[i] + f[:, None] * (P[i + 1] - P[i]), i, f


def bind_offsets(P, N, X):
    """Vertex binding to a centreline: u (rest arc length), offsets along N, B = T x N and T (beyond the ends)."""
    T = tangents(P)
    u, dist, j, t = project(P, X)
    s = arclen(P)
    _, i, f = at(P, u)
    Tn = unit(T[i] * (1 - f)[:, None] + T[i + 1] * f[:, None])
    Nn = N[i] * (1 - f)[:, None] + N[i + 1] * f[:, None]
    Nn = unit(Nn - (Nn * Tn).sum(1, keepdims=True) * Tn)
    Bn = np.cross(Tn, Nn)
    c, _, _ = at(P, u)
    d = X - c
    return u, (d * Nn).sum(1), (d * Bn).sum(1), (d * Tn).sum(1), dist


def trace(v, tip, toward, h=2e-3, slab=0.8e-3, rmin=2.2e-3, maxn=300):
    """March from a tendon tip toward `toward` through the vertex cloud v (slab centroids).
    -> centreline points (tip first) and radius per point."""
    p = v[np.linalg.norm(v - tip, axis=1) < 3e-3].mean(0)
    d = start_dir(v, p, toward)
    pts, rad, r, miss = [p], [], 1.5e-3, 0
    for _ in range(maxn):
        q = pts[-1] + h * d
        rel = v - q
        a = rel @ d
        perp = np.linalg.norm(rel - np.outer(a, d), axis=1)
        m = (np.abs(a) < slab) & (perp < max(rmin, 2.0 * r))
        if m.sum() < 4:
            miss += 1
            if miss > (3 if len(pts) < 4 else 1):
                break
            pts.append(q)
            rad.append(r)
            continue
        miss = 0
        c = v[m].mean(0)
        c = c - ((c - q) @ d) * d
        nd = unit(c - pts[-1])
        d = unit(0.6 * d + 0.4 * nd)
        r = float(np.median(np.linalg.norm((v[m] - c) - np.outer((v[m] - c) @ d, d), axis=1)))
        pts.append(c)
        rad.append(r)
    while miss and len(pts) > 2:              # drop trailing extrapolated points
        pts.pop(); rad.pop(); miss -= 1
    rad = [rad[0] if rad else 1.5e-3] + rad
    return np.array(pts), np.array(rad[:len(pts)])


def densify(v, f, spacing=0.4e-3, seed=0):
    """Random surface samples (about one per spacing^2) plus the vertices."""
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    ar = np.linalg.norm(np.cross(b - a, c - a), axis=1) / 2
    n = int(min(max(ar.sum() / spacing ** 2, 100), 300000))
    rng = np.random.default_rng(seed)
    fi = rng.choice(len(f), n, p=ar / ar.sum())
    r1, r2 = rng.random(n), rng.random(n)
    s = np.sqrt(r1)
    p = (1 - s)[:, None] * a[fi] + (s * (1 - r2))[:, None] * b[fi] + (s * r2)[:, None] * c[fi]
    return np.vstack([p, v])


def start_dir(v, tip, toward, rad=8e-3):
    q = v[np.linalg.norm(v - tip, axis=1) < rad]
    if len(q) < 5:
        return unit(np.asarray(toward) - tip)
    _, _, vt = np.linalg.svd(q - q.mean(0), full_matrices=False)
    d = vt[0]
    g = unit(np.asarray(toward) - tip)
    d = d if d @ g > 0 else -d
    return d if d @ g > 0.7 else g
