// Phase E4 helper (browser): how deep soft-tissue vertices sit inside bone, in any pose of the live app.
// Same measure as tools/check_tissues.py: bone surface sampled every <= 0.6 mm (rest frame of its skin joint), depth
// = -(x - s).n for the nearest sample s (searched up to 6 mm), 0 outside. The nearest sample per grid cell is found
// once per bone (two-pass nearest-seed propagation), so a pose test is one lookup per vertex and nearby bone.
import * as THREE from 'three';

const SPACING = 0.6e-3, H = 0.8e-3, RMAX = 6e-3;
const Qof = (m) => new THREE.Matrix4().multiplyMatrices(m.skeleton.boneInverses[0], m.bindMatrix);

export class Inside {
  constructor(app) {
    this.app = app; this.move = app.move;
    const S = new Map();                                        // skin joint -> surface samples (model space, rest)
    for (const s of app.anatomy.structs) if (s.layer === 'bone') for (const m of s.meshes) if (m.isSkinnedMesh) this.sample(m, S);
    this.bones = [...S.entries()].map(([j, a]) => this.grid(j, Float64Array.from(a.p), Float64Array.from(a.n)));
    this.soft = [];
    for (const s of app.anatomy.structs) if (!['bone', 'attachment'].includes(s.layer))
      for (const m of s.meshes) if (m.isSkinnedMesh) this.soft.push({ m, s: s.name, Q: Qof(m) });
    this.count = this.soft.reduce((a, x) => a + x.m.geometry.attributes.position.count, 0);
  }

  sample(m, S) {
    const g = m.geometry, P = g.attributes.position, N = g.attributes.normal, I = g.index, Q = Qof(m), R = new THREE.Matrix3().setFromMatrix4(Q);
    const a = [0, 1, 2].map(() => new THREE.Vector3()), n = [0, 1, 2].map(() => new THREE.Vector3()), p = new THREE.Vector3(), q = new THREE.Vector3();
    const tri = I ? I.count / 3 : P.count / 3;
    for (let t = 0; t < tri; t++) {
      const v = [0, 1, 2].map((k) => (I ? I.getX(3 * t + k) : 3 * t + k));
      const j = g.attributes.skinIndex.getComponent(v[0], 0);
      if (!S.has(j)) S.set(j, { p: [], n: [] });
      const o = S.get(j);
      v.forEach((i, k) => { a[k].fromBufferAttribute(P, i).applyMatrix4(Q); n[k].fromBufferAttribute(N, i).applyMatrix3(R).normalize(); });
      const L = Math.max(a[0].distanceTo(a[1]), a[1].distanceTo(a[2]), a[2].distanceTo(a[0])), K = Math.max(1, Math.ceil(L / SPACING));
      for (let x = 0; x <= K; x++) for (let y = 0; y <= K - x; y++) {
        const u = x / K, w = y / K, z = 1 - u - w;
        p.copy(a[0]).multiplyScalar(z).addScaledVector(a[1], u).addScaledVector(a[2], w);
        q.copy(n[0]).multiplyScalar(z).addScaledVector(n[1], u).addScaledVector(n[2], w).normalize();
        o.p.push(p.x, p.y, p.z); o.n.push(q.x, q.y, q.z);
      }
    }
  }

  // grid over the bone (+ RMAX), nearest sample id per cell
  grid(j, p, n) {
    const ns = p.length / 3, lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
    for (let i = 0; i < ns; i++) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[3 * i + k]); hi[k] = Math.max(hi[k], p[3 * i + k]); }
    const o = lo.map((x) => x - RMAX), D = [0, 1, 2].map((k) => Math.ceil((hi[k] - lo[k] + 2 * RMAX) / H) + 1);
    const seed = new Int32Array(D[0] * D[1] * D[2]).fill(-1), best = new Float64Array(seed.length).fill(Infinity);
    const id = (x, y, z) => (x * D[1] + y) * D[2] + z;
    const d2 = (c, i, x, y, z) => { const a = o[0] + x * H - p[3 * i], b = o[1] + y * H - p[3 * i + 1], e = o[2] + z * H - p[3 * i + 2]; return a * a + b * b + e * e; };
    for (let i = 0; i < ns; i++) {                         // seeds: the 2x2x2 cells around each sample
      const x0 = Math.floor((p[3 * i] - o[0]) / H), y0 = Math.floor((p[3 * i + 1] - o[1]) / H), z0 = Math.floor((p[3 * i + 2] - o[2]) / H);
      for (let x = x0; x <= x0 + 1; x++) for (let y = y0; y <= y0 + 1; y++) for (let z = z0; z <= z0 + 1; z++) {
        const c = id(x, y, z), d = d2(c, i, x, y, z); if (d < best[c]) { best[c] = d; seed[c] = i; }
      }
    }
    const NB = [];                                           // half of the 26-neighbourhood (raster-earlier cells)
    for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let e = -1; e <= 1; e++) if (a * 9 + b * 3 + e < 0) NB.push([a, b, e]);
    const pass = (dir) => {
      const r = (k) => (dir > 0 ? [0, D[k], 1] : [D[k] - 1, -1, -1]), [x0, x1, sx] = r(0), [y0, y1, sy] = r(1), [z0, z1, sz] = r(2);
      for (let x = x0; x !== x1; x += sx) for (let y = y0; y !== y1; y += sy) for (let z = z0; z !== z1; z += sz) {
        const c = id(x, y, z);
        for (const [a, b, e] of NB) {
          const X = x + dir * a, Y = y + dir * b, Z = z + dir * e;
          if (X < 0 || Y < 0 || Z < 0 || X >= D[0] || Y >= D[1] || Z >= D[2]) continue;
          const s = seed[id(X, Y, Z)]; if (s < 0) continue;
          const d = d2(c, s, x, y, z); if (d < best[c]) { best[c] = d; seed[c] = s; }
        }
      }
    };
    pass(1); pass(-1);
    let cx = 0, cy = 0, cz = 0, r = 0;
    for (let i = 0; i < ns; i++) { cx += p[3 * i] / ns; cy += p[3 * i + 1] / ns; cz += p[3 * i + 2] / ns; }
    for (let i = 0; i < ns; i++) r = Math.max(r, Math.hypot(p[3 * i] - cx, p[3 * i + 1] - cy, p[3 * i + 2] - cz));
    return { j, p, n, o, D, seed, c: [cx, cy, cz], r, name: this.move.rig.names[j] };
  }

  // posed soft-tissue vertices (model space) of the current app pose: skin(position + aDelta)
  positions() {
    const L = this.move.mats.map((m) => m.elements), out = new Float64Array(3 * this.count), names = new Array(this.count);
    let q = 0; const v = new THREE.Vector3();
    for (const { m, s, Q } of this.soft) {
      const a = m.geometry.attributes, Dl = a.aDelta, n = a.position.count, e = Q.elements;
      for (let i = 0; i < n; i++, q++) {
        v.fromBufferAttribute(a.position, i); if (Dl) v.x += Dl.getX(i), v.y += Dl.getY(i), v.z += Dl.getZ(i);
        const x = e[0] * v.x + e[4] * v.y + e[8] * v.z + e[12], y = e[1] * v.x + e[5] * v.y + e[9] * v.z + e[13], z = e[2] * v.x + e[6] * v.y + e[10] * v.z + e[14];
        let X = 0, Y = 0, Z = 0;
        for (let k = 0; k < 4; k++) {
          const w = a.skinWeight.getComponent(i, k); if (!w) continue;
          const M = L[a.skinIndex.getComponent(i, k)];
          X += w * (M[0] * x + M[4] * y + M[8] * z + M[12]); Y += w * (M[1] * x + M[5] * y + M[9] * z + M[13]); Z += w * (M[2] * x + M[6] * y + M[10] * z + M[14]);
        }
        out[3 * q] = X; out[3 * q + 1] = Y; out[3 * q + 2] = Z; names[q] = s;
      }
    }
    this.names = names;
    return out;
  }

  // max depth (m) of each point inside any posed bone
  depth(X) {
    const n = X.length / 3, out = new Float64Array(n), L = this.move.mats;
    for (const B of this.bones) {
      const Mi = L[B.j].clone().invert().elements, { o, D, seed, p, n: N, c, r } = B;
      for (let i = 0; i < n; i++) {
        const a = X[3 * i], b = X[3 * i + 1], e = X[3 * i + 2];
        const x = Mi[0] * a + Mi[4] * b + Mi[8] * e + Mi[12], y = Mi[1] * a + Mi[5] * b + Mi[9] * e + Mi[13], z = Mi[2] * a + Mi[6] * b + Mi[10] * e + Mi[14];
        if (Math.hypot(x - c[0], y - c[1], z - c[2]) > r + 1e-3) continue;
        const gx = Math.round((x - o[0]) / H), gy = Math.round((y - o[1]) / H), gz = Math.round((z - o[2]) / H);
        if (gx < 0 || gy < 0 || gz < 0 || gx >= D[0] || gy >= D[1] || gz >= D[2]) continue;
        const s = seed[(gx * D[1] + gy) * D[2] + gz]; if (s < 0) continue;
        const dx = x - p[3 * s], dy = y - p[3 * s + 1], dz = z - p[3 * s + 2];
        if (dx * dx + dy * dy + dz * dz > RMAX * RMAX) continue;
        const sd = dx * N[3 * s] + dy * N[3 * s + 1] + dz * N[3 * s + 2];
        if (sd < 0 && -sd > out[i]) out[i] = -sd;
      }
    }
    return out;
  }
}
