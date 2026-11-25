// Nerve trees (Phase E4): a nerve trunk and its digital branches glide as one elastic system, fixed at the proximal
// cut and at the fingertips (spec E2.4), so finger motion moves the trunk at the wrist (Wright 1996).
// Every piece between two nodes (anchor, junction or tip) is a linear spring: tension = k (bed elongation +
// x_distal - x_proximal), k = area / rest length (cables.json radii are all 1 mm: areas from TREES below).
// Node displacement x (material, + = distal, m) from force balance at each junction; the material inside a piece
// is spread evenly. Where a piece's material is compressed (bed + dx < rest length) it shows the gentle slack wave.
// area: cross-section (mm2) per member, typical adult sizes (approximate, edit freely): median nerve at the wrist
// ~10, common palmar digital nerve ~3, proper palmar digital nerve ~1.8 (diameters about 3.5, 2 and 1.5 mm).
const TREES = [{ trunk: 'Median nerve', branches: /^(Common|Proper) palmar digital branches of median nerve/,
  area: [[/^Median nerve$/, 10], [/^Common/, 3], [/^Proper/, 1.8]] }];
const JOIN = 2e-3;                    // a branch starts on a member within 2 mm

export class NerveTrees {
  constructor(cables) {
    this.C = cables; this.trees = [];
    for (const T of TREES) {
      const trunk = cables.list.find((c) => c.name === T.trunk); if (!trunk) continue;
      const mem = [trunk, ...cables.list.filter((c) => T.branches.test(c.name) && !c.common)];
      const nodes = [{ fixed: true }], cut = new Map(mem.map((c) => [c, []]));   // cable -> [{u, node}]
      cut.get(trunk).push({ u: 0, node: 0 });
      for (const b of mem.slice(1)) {             // parent: member nearest the branch start (not sharing its start)
        let best = null;
        for (const a of mem) if (a !== b) for (let i = 0; i < a.n; i++) {
          const d = Math.hypot(a.P[3 * i] - b.P[0], a.P[3 * i + 1] - b.P[1], a.P[3 * i + 2] - b.P[2]);
          const s = d + (i === 0 ? 1 : 0);
          if (d < JOIN && (!best || s < best.s)) best = { s, a, u: a.sr[i] };
        }
        if (!best) { console.warn('nerve tree: no parent for', b.name); continue; }
        const list = cut.get(best.a);
        let j = list.find((p) => Math.abs(p.u - best.u) < JOIN);
        if (!j) { j = { u: best.u, node: nodes.length }; nodes.push({ fixed: false }); list.push(j); }
        cut.get(b).push({ u: 0, node: j.node });
      }
      const area = (c) => (T.area.find(([re]) => re.test(c.name)) || [0, 1])[1];
      const segs = [];
      for (const c of mem) {
        const L = c.sr[c.n - 1], list = cut.get(c).sort((p, q) => p.u - q.u);
        if (L - list[list.length - 1].u > JOIN) { list.push({ u: L, node: nodes.length }); nodes.push({ fixed: true }); }
        else list[list.length - 1].u = L;
        for (let i = 0; i + 1 < list.length; i++)
          segs.push({ c, u0: list[i].u, u1: list[i + 1].u, p: list[i].node, q: list[i + 1].node, k: area(c) / (list[i + 1].u - list[i].u) });
      }
      for (const c of mem) c.tree = { segs: segs.filter((s) => s.c === c) };
      const free = nodes.map((n, i) => (n.fixed ? -1 : i)).filter((i) => i >= 0);
      this.trees.push({ mem, nodes, segs, free, x: new Float64Array(nodes.length) });
    }
  }

  // after the members' beds (Pp, sp) are posed: solve node displacements, set the material map, add slack waves
  update(trees = this.trees) {
    const C = this.C;
    for (const T of trees) {
      const n = T.free.length, row = new Map(T.free.map((i, r) => [i, r]));
      const A = Array.from({ length: n }, () => new Float64Array(n + 1));
      for (const s of T.segs) {
        s.f0 = C.follow(s.c, s.u0); s.bed = C.follow(s.c, s.u1) - s.f0;
        const dL = s.bed - (s.u1 - s.u0), p = row.get(s.p), q = row.get(s.q);
        if (q !== undefined) { A[q][q] += s.k; if (p !== undefined) A[q][p] -= s.k; A[q][n] -= s.k * dL; }
        if (p !== undefined) { A[p][p] += s.k; if (q !== undefined) A[p][q] -= s.k; A[p][n] += s.k * dL; }
      }
      for (let i = 0; i < n; i++) {               // Gauss-Jordan (a few nodes)
        let m = i; for (let r = i + 1; r < n; r++) if (Math.abs(A[r][i]) > Math.abs(A[m][i])) m = r;
        [A[i], A[m]] = [A[m], A[i]];
        for (let r = 0; r < n; r++) if (r !== i) { const f = A[r][i] / A[i][i]; for (let k = i; k <= n; k++) A[r][k] -= f * A[i][k]; }
      }
      T.x.fill(0); T.free.forEach((i, r) => { T.x[i] = A[r][n] / A[r][r]; });
      for (const s of T.segs) { s.x0 = T.x[s.p]; s.rho = (s.bed + T.x[s.q] - s.x0) / (s.u1 - s.u0); }
      for (const c of T.mem) this.wave(c);
    }
  }
  sigma(c, u) {
    const S = c.tree.segs; let s = S[0];
    for (const t of S) if (u >= t.u0) s = t;
    return s.f0 + s.x0 + (u - s.u0) * s.rho;
  }
  // compressed pieces: gentle wave (amplitude as cables.js slack mode, tapered to 0 at the nodes)
  wave(c) {
    const { n, Pp, Np, sp, sr } = c, lam = this.C.cab.glide.wave_m, amax = this.C.cab.glide.wave_max_m;
    c.wave = 0;
    for (let i = 1; i < n - 1; i++) {
      const s = c.tree.segs.find((t) => sr[i] >= t.u0 && sr[i] <= t.u1); if (!s || s.rho >= 1) continue;
      const A = Math.min(lam / Math.PI * Math.sqrt(1 / Math.max(s.rho, 0.2) - 1), amax) * Math.sin(Math.PI * (sr[i] - s.u0) / (s.u1 - s.u0));
      const g = A * Math.sin(2 * Math.PI * sp[i] / lam);
      Pp[3 * i] += g * Np[3 * i]; Pp[3 * i + 1] += g * Np[3 * i + 1]; Pp[3 * i + 2] += g * Np[3 * i + 2];
      c.wave = Math.max(c.wave, A);
    }
  }
}
