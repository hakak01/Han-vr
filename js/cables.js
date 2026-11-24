// Cables (tendons, nerves, vessels): port of tools/tisspose.py Rig.cable / zone / frame_at (assets/cables.json).
// Per pose: samples skinned by their bones; joint zones posed in polar coordinates about the joint axis;
// tendons slide (material map: insertion fixed, ARMS excursion taken up in the zones); nerves/vessels glide when
// their bed lengthens and show a gentle wave where it shortens. All arrays: metres, model (glTF) space.
const TAU = 2 * Math.PI;
const wrap = (x) => ((((x + Math.PI) % TAU) + TAU) % TAU) - Math.PI;

function arclen(P, n, out = new Float64Array(n)) {
  out[0] = 0;
  for (let i = 1; i < n; i++) out[i] = out[i - 1] + Math.hypot(P[3 * i] - P[3 * i - 3], P[3 * i + 1] - P[3 * i - 2], P[3 * i + 2] - P[3 * i - 1]);
  return out;
}
function tangents(P, n, T = new Float64Array(3 * n)) {     // np.gradient, normalised
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
    let x = P[3 * b] - P[3 * a], y = P[3 * b + 1] - P[3 * a + 1], z = P[3 * b + 2] - P[3 * a + 2];
    const l = Math.hypot(x, y, z) || 1e-15; T[3 * i] = x / l; T[3 * i + 1] = y / l; T[3 * i + 2] = z / l;
  }
  return T;
}
// index of arc length u on the increasing table s (clamped); the fraction is left in SEGF[0] (no allocation)
export const SEGF = new Float64Array(1);
export function seg(s, u) {
  const n = s.length;
  if (u <= s[0]) { SEGF[0] = 0; return 0; }
  if (u >= s[n - 1]) { SEGF[0] = 1; return n - 2; }
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (s[m] <= u) lo = m; else hi = m; }
  SEGF[0] = (u - s[lo]) / Math.max(s[lo + 1] - s[lo], 1e-15);
  return lo;
}
const interp = (u, xs, ys) => { const i = seg(xs, u), f = SEGF[0]; return ys[i] + f * (ys[i + 1] - ys[i]); };
const F3 = (a) => Float64Array.from(a.flat());

export class Cables {
  constructor(cab, rig) {
    this.cab = cab; this.rig = rig;
    const idx = rig.index;
    this.list = cab.cables.map((c) => {
      const n = c.pts.length, P = F3(c.pts);
      const o = { ...c, n, P, N: F3(c.nrm), J: Int32Array.from(c.j.flat()), Wt: F3(c.w), sr: arclen(P, n), T0: tangents(P, n),
        Pp: new Float64Array(3 * n), Np: new Float64Array(3 * n), T: new Float64Array(3 * n), sp: new Float64Array(n), sAt: new Float64Array(n) };
      o.zones = (c.zones || []).map((z) => {
        const a = z.i0 - 1, b = z.i1 + 1, ce = cab.arc_joints[z.child].center;
        const s0 = arclen(P.subarray(3 * a, 3 * b + 3), b - a + 1);
        let mx = 0, my = 0, mz = 0; const k = z.i1 - z.i0 + 1;     // fallback axis (joint at rest)
        for (let i = z.i0; i <= z.i1; i++) { mx += P[3 * i] / k; my += P[3 * i + 1] / k; mz += P[3 * i + 2] / k; }
        const u = [P[3 * b] - P[3 * a], P[3 * b + 1] - P[3 * a + 1], P[3 * b + 2] - P[3 * a + 2]], v = [mx - ce[0], my - ce[1], mz - ce[2]];
        const n0 = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]], l = Math.hypot(...n0) || 1;
        return { ...z, a, b, ce, s0, n0: n0.map((x) => x / l), pi: idx[z.parent], ci: idx[z.child] };
      });
      return o;
    });
    this.byId = new Map(this.list.map((c) => [c.id, c]));
    for (const c of this.list) {                 // bones this cable depends on (skip it when none of them moved)
      const d = new Set(c.zones.flatMap((z) => [z.pi, z.ci]));
      for (let i = 0; i < 4 * c.n; i++) if (c.Wt[i]) d.add(c.J[i]);
      c.deps = [...d];
    }
    this.prev = null; this.dL = {};
    this.order = [...this.list].sort((a, b) => !!a.common - !!b.common);
    this.muscle = {};
  }

  // M: array of THREE.Matrix4 (skin-joint order); pose: anatomical degrees
  update(M, pose) {
    this.E = M.map((m) => m.elements); this.pose = pose;
    const P = this.prev, ch = this.changed = this.E.map((e, j) => !P || e.some((x, k) => x !== P[j][k]));
    this.prev = this.E.map((e) => e.slice());
    for (const c of this.order) {
      c.fresh = c.common ? c.members.some((m) => this.byId.get(m).fresh) : c.deps.some((j) => ch[j]);
      if (c.fresh) this.cable(c);
    }
    this.muscle = {};
    for (const c of this.list) if (c.kind === 'tendon' && !c.common) (this.muscle[c.muscle] ||= []).push(c.dT);
    for (const k in this.muscle) this.muscle[k] = this.muscle[k].reduce((a, b) => a + b, 0) / this.muscle[k].length;
  }

  cable(c) {
    const { n, P, N, J, Wt, Pp, Np } = c, E = this.E;
    for (let i = 0; i < n; i++) {
      let px = 0, py = 0, pz = 0, nx = 0, ny = 0, nz = 0;
      const x = P[3 * i], y = P[3 * i + 1], z = P[3 * i + 2], ax = N[3 * i], ay = N[3 * i + 1], az = N[3 * i + 2];
      for (let k = 0; k < 4; k++) {
        const w = Wt[4 * i + k]; if (!w) continue;
        const e = E[J[4 * i + k]];
        px += w * (e[0] * x + e[4] * y + e[8] * z + e[12]); py += w * (e[1] * x + e[5] * y + e[9] * z + e[13]); pz += w * (e[2] * x + e[6] * y + e[10] * z + e[14]);
        nx += w * (e[0] * ax + e[4] * ay + e[8] * az); ny += w * (e[1] * ax + e[5] * ay + e[9] * az); nz += w * (e[2] * ax + e[6] * ay + e[10] * az);
      }
      Pp[3 * i] = px; Pp[3 * i + 1] = py; Pp[3 * i + 2] = pz; Np[3 * i] = nx; Np[3 * i + 1] = ny; Np[3 * i + 2] = nz;
    }
    const zE = c.zones.map((z) => this.zone(c, z));
    const T = tangents(Pp, n, c.T);
    for (let i = 0; i < n; i++) {
      const d = Np[3 * i] * T[3 * i] + Np[3 * i + 1] * T[3 * i + 1] + Np[3 * i + 2] * T[3 * i + 2];
      let x = Np[3 * i] - d * T[3 * i], y = Np[3 * i + 1] - d * T[3 * i + 1], z = Np[3 * i + 2] - d * T[3 * i + 2];
      const l = Math.hypot(x, y, z) || 1e-15; Np[3 * i] = x / l; Np[3 * i + 1] = y / l; Np[3 * i + 2] = z / l;
    }
    const sr = c.sr, sp = arclen(Pp, n, c.sp), Lr = sr[n - 1], Lp = sp[n - 1];
    c.wave = 0;
    if (c.common) {
      c.d = c.members.reduce((a, m) => a + this.dL[m], 0) / c.members.length; c.mode = 'common';
    } else if (c.kind === 'tendon') {        // material map (see tisspose.py)
      const dm = new Float64Array(n - 1), dr = new Float64Array(n - 1), dp = new Float64Array(n - 1);
      for (let k = 0; k < n - 1; k++) { dr[k] = sr[k + 1] - sr[k]; dp[k] = sp[k + 1] - sp[k]; dm[k] = dr[k]; }
      c.zones.forEach((z, q) => {
        let R = 0, Q = 0;
        for (let k = z.a; k < z.b; k++) { R += dr[k]; Q += dp[k]; }
        const f = (R - zE[q]) / Math.max(Q, 1e-9);
        for (let k = z.a; k < z.b; k++) dm[k] = dp[k] * f;
      });
      let carry = 0;
      for (let k = n - 2; k >= 0; k--) { dm[k] += carry; carry = Math.min(dm[k] - 0.2 * dr[k], 0); dm[k] -= carry; }
      const m = c.m || (c.m = new Float64Array(n));
      m[n - 1] = Lr;
      for (let k = n - 2; k >= 0; k--) m[k] = m[k + 1] - dm[k];
      c.dT = -m[0]; this.dL[c.id] = c.dT; c.mode = 'tendon';
    } else if (Lp >= Lr) c.mode = 'taut';
    else {                                     // slack: wave where the bed shortened
      c.mode = 'slack';
      const r = new Float64Array(n + 1);
      for (let k = 0; k < n - 1; k++) r[k + 1] = (sp[k + 1] - sp[k]) / Math.max(sr[k + 1] - sr[k], 1e-9);
      r[0] = r[1]; r[n] = r[n - 1];
      const lam = this.cab.glide.wave_m, amax = this.cab.glide.wave_max_m;
      for (let i = 1; i < n - 1; i++) {
        let s = 0; for (let k = i - 1; k <= i + 3; k++) if (k >= 0 && k <= n) s += r[k];   // convolve 'same', [1:]
        const ratio = Math.min(s / 5, 1), A = Math.min(lam / Math.PI * Math.sqrt(Math.max(1 / ratio - 1, 0)), amax);
        const g = A * Math.sin(TAU * sp[i] / lam);
        Pp[3 * i] += g * Np[3 * i]; Pp[3 * i + 1] += g * Np[3 * i + 1]; Pp[3 * i + 2] += g * Np[3 * i + 2];
        c.wave = Math.max(c.wave, A);
      }
      tangents(Pp, n, c.T);
    }
    arclen(Pp, n, c.sAt);
  }

  // E2b zone (samples i0..i1 between ends a, b) in polar coordinates about the joint -> mechanical shortening E (m)
  zone(c, z) {
    const E = this.E, Mp = E[z.pi], { P, N, Pp, Np } = c, ce = z.ce, a = z.a, b = z.b, K = b - a + 1;
    const iM = this.inv(Mp), Mc = E[z.ci];
    // relative rotation parent -> child (rotation part of iM * Mc) as axis-angle
    const R = [0, 1, 2].map((r) => [0, 1, 2].map((q) => iM[r] * Mc[4 * q] + iM[r + 4] * Mc[4 * q + 1] + iM[r + 8] * Mc[4 * q + 2]));
    const wx = R[2][1] - R[1][2], wy = R[0][2] - R[2][0], wz = R[1][0] - R[0][1], s = Math.hypot(wx, wy, wz) / 2;
    const th = Math.atan2(s, (R[0][0] + R[1][1] + R[2][2] - 1) / 2);
    const nn = th > 1e-4 && s > 1e-12 ? [wx / (2 * s), wy / (2 * s), wz / (2 * s)] : z.n0;
    const Q = new Float64Array(3 * (K + 2));
    for (let q = 0; q < K; q++) for (let k = 0; k < 3; k++) Q[3 * q + k] = P[3 * (a + q) + k] - ce[k];
    [a, b].forEach((i, q) => {
      const x = Pp[3 * i], y = Pp[3 * i + 1], zz = Pp[3 * i + 2];
      for (let k = 0; k < 3; k++) Q[3 * (K + q) + k] = iM[k] * x + iM[k + 4] * y + iM[k + 8] * zz + iM[k + 12] - ce[k];
    });
    const Z = new Float64Array(K + 2), Dx = new Float64Array(3 * (K + 2)), r = new Float64Array(K + 2), be = new Float64Array(K + 2);
    let best = -1, bi = 0;
    for (let q = 0; q < K + 2; q++) {
      Z[q] = Q[3 * q] * nn[0] + Q[3 * q + 1] * nn[1] + Q[3 * q + 2] * nn[2];
      for (let k = 0; k < 3; k++) Dx[3 * q + k] = Q[3 * q + k] - Z[q] * nn[k];
      r[q] = Math.hypot(Dx[3 * q], Dx[3 * q + 1], Dx[3 * q + 2]);
      if (q < K && r[q] > best) { best = r[q]; bi = q; }
    }
    const e1 = [Dx[3 * bi] / best, Dx[3 * bi + 1] / best, Dx[3 * bi + 2] / best];
    const e2 = [nn[1] * e1[2] - nn[2] * e1[1], nn[2] * e1[0] - nn[0] * e1[2], nn[0] * e1[1] - nn[1] * e1[0]];
    for (let q = 0; q < K + 2; q++) {
      const d = [Dx[3 * q], Dx[3 * q + 1], Dx[3 * q + 2]];
      be[q] = Math.atan2(d[0] * e2[0] + d[1] * e2[1] + d[2] * e2[2], d[0] * e1[0] + d[1] * e1[1] + d[2] * e1[2]);
    }
    for (let q = 1; q < K; q++) be[q] = be[q - 1] + wrap(be[q] - be[q - 1]);
    be[K] = be[0] + wrap(be[K] - be[0]); be[K + 1] = be[K - 1] + wrap(be[K + 1] - be[K - 1]);
    const span = be[K - 1] - be[0], s0 = z.s0;
    const dA = [be[K] - be[0], r[K] - r[0], Z[K] - Z[0]], dB = [be[K + 1] - be[K - 1], r[K + 1] - r[K - 1], Z[K + 1] - Z[K - 1]];
    for (let q = 1; q < K - 1; q++) {
      const i = a + q;
      let t = Math.abs(span) > 0.05 ? (be[q] - be[0]) / span : s0[q] / s0[K - 1];
      t = Math.min(1, Math.max(0, t));
      const d0 = (1 - t) * dA[0] + t * dB[0], ang = be[q] + d0;
      const ri = Math.max(r[q] + (1 - t) * dA[1] + t * dB[1], 0), zi = Z[q] + (1 - t) * dA[2] + t * dB[2];
      const co = ri * Math.cos(ang), si = ri * Math.sin(ang);
      const X0 = ce[0] + co * e1[0] + si * e2[0] + zi * nn[0], X1 = ce[1] + co * e1[1] + si * e2[1] + zi * nn[1], X2 = ce[2] + co * e1[2] + si * e2[2] + zi * nn[2];
      for (let k = 0; k < 3; k++) Pp[3 * i + k] = Mp[k] * X0 + Mp[k + 4] * X1 + Mp[k + 8] * X2 + Mp[k + 12];
      // normal: rest normal turned by d0 about n (Rodrigues), then by the parent
      const v0 = N[3 * i], v1 = N[3 * i + 1], v2 = N[3 * i + 2], c0 = Math.cos(d0), s1 = Math.sin(d0);
      const dv = (nn[0] * v0 + nn[1] * v1 + nn[2] * v2) * (1 - c0);
      const w0 = v0 * c0 + (nn[1] * v2 - nn[2] * v1) * s1 + nn[0] * dv, w1 = v1 * c0 + (nn[2] * v0 - nn[0] * v2) * s1 + nn[1] * dv;
      const w2 = v2 * c0 + (nn[0] * v1 - nn[1] * v0) * s1 + nn[2] * dv;
      for (let k = 0; k < 3; k++) Np[3 * i + k] = Mp[k] * w0 + Mp[k + 4] * w1 + Mp[k + 8] * w2;
    }
    let Lp = 0;
    for (let i = a; i < b; i++) Lp += Math.hypot(Pp[3 * i + 3] - Pp[3 * i], Pp[3 * i + 4] - Pp[3 * i + 1], Pp[3 * i + 5] - Pp[3 * i + 2]);
    const Vs = s0[K - 1] - Lp, mech = c.mech;
    if (!mech || !z.dofs.length) return Vs;
    let Em = 0;
    for (const d of z.dofs) {
      const t = mech[d]; if (!t) continue;
      const F = (q) => { const x = Math.min(Math.max((q - t.deg0) / t.step, 0), t.F.length - 1), i = Math.min(Math.floor(x), t.F.length - 2);
        return t.F[i] + (x - i) * (t.F[i + 1] - t.F[i]); };
      Em += F(this.rig.value(this.pose, d)) - F(this.rig.rest[d]);
    }
    return Em;
  }
  inv(e) {   // inverse of an affine column-major 4x4 (rotation + translation)
    const R = [e[0], e[1], e[2], e[4], e[5], e[6], e[8], e[9], e[10]];
    const det = R[0] * (R[4] * R[8] - R[5] * R[7]) - R[3] * (R[1] * R[8] - R[2] * R[7]) + R[6] * (R[1] * R[5] - R[2] * R[4]);
    const m = [(R[4] * R[8] - R[7] * R[5]) / det, (R[7] * R[2] - R[1] * R[8]) / det, (R[1] * R[5] - R[4] * R[2]) / det,
      (R[6] * R[5] - R[3] * R[8]) / det, (R[0] * R[8] - R[6] * R[2]) / det, (R[3] * R[2] - R[0] * R[5]) / det,
      (R[3] * R[7] - R[6] * R[4]) / det, (R[6] * R[1] - R[0] * R[7]) / det, (R[0] * R[4] - R[3] * R[1]) / det];
    const t = [0, 1, 2].map((k) => -(m[k] * e[12] + m[k + 3] * e[13] + m[k + 6] * e[14]));
    return [m[0], m[1], m[2], 0, m[3], m[4], m[5], 0, m[6], m[7], m[8], 0, t[0], t[1], t[2], 1];
  }

  // posed arc length of the material point at rest arc length u
  sigma(c, u) {
    if (c.mode === 'common') return u + c.d;
    const n = c.n;
    if (c.mode === 'tendon') {
      const m = c.m, sp = c.sp;
      return u < m[0] ? sp[0] + u - m[0] : u > m[n - 1] ? sp[n - 1] + u - m[n - 1] : interp(u, m, sp);
    }
    if (c.mode === 'taut') return u * c.sp[n - 1] / c.sr[n - 1];
    return interp(u, c.sr, c.sp);
  }
  follow(c, u) { return interp(u, c.sr, c.sp); }               // no sliding (sheaths)
  slide(c, u) { return u - interp(u, c.sr, c.m); }             // tendon material displacement, < 0 = proximal
  // frame at posed arc length s -> writes centre, tangent, normal, binormal into out (12 numbers)
  frame(c, s, out) {
    const { Pp, Np, T, sAt } = c, i = seg(sAt, s), f = SEGF[0], g = 1 - f;
    let tx = T[3 * i] * g + T[3 * i + 3] * f, ty = T[3 * i + 1] * g + T[3 * i + 4] * f, tz = T[3 * i + 2] * g + T[3 * i + 5] * f;
    let l = Math.hypot(tx, ty, tz) || 1e-15; tx /= l; ty /= l; tz /= l;
    let nx = Np[3 * i] * g + Np[3 * i + 3] * f, ny = Np[3 * i + 1] * g + Np[3 * i + 4] * f, nz = Np[3 * i + 2] * g + Np[3 * i + 5] * f;
    const d = nx * tx + ny * ty + nz * tz; nx -= d * tx; ny -= d * ty; nz -= d * tz;
    l = Math.hypot(nx, ny, nz) || 1e-15; nx /= l; ny /= l; nz /= l;
    const ext = s < 0 ? s : 0;                                  // beyond the origin end: straight along the tangent
    out[0] = Pp[3 * i] * g + Pp[3 * i + 3] * f + ext * tx; out[1] = Pp[3 * i + 1] * g + Pp[3 * i + 4] * f + ext * ty;
    out[2] = Pp[3 * i + 2] * g + Pp[3 * i + 5] * f + ext * tz;
    out[3] = tx; out[4] = ty; out[5] = tz; out[6] = nx; out[7] = ny; out[8] = nz;
    out[9] = ty * nz - tz * ny; out[10] = tz * nx - tx * nz; out[11] = tx * ny - ty * nx;
    return out;
  }
}
