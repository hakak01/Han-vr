// Phase E4 accuracy tests, run inside the live app page by tools/check_motion.mjs (headless Chromium).
// Every number comes from the same code the viewer uses (js/rig.js, cables.js, tissuemove.js, move.js).
import { Inside } from './motion_inside.js';

const F = [2, 3, 4, 5], ORD = { 1: 'first', 2: 'second', 3: 'third', 4: 'fourth', 5: 'fifth' };
const NAME = { 1: 'thumb', 2: 'index', 3: 'middle', 4: 'ring', 5: 'little' };
const tick = () => new Promise((r) => setTimeout(r, 0));

export async function run(app, log = () => {}) {
  const mv = app.move, rig = mv.rig, out = {};
  mv.stop(); mv.setTenodesis(false);
  const P = (part) => rig.clamp({ ...rig.neutral, ...part });
  const max = (d) => rig.limits(d)[1], min = (d) => rig.limits(d)[0];
  const fist = {}, hyper = {};
  for (const f of F) for (const d of [`mcp${f}_flex`, `pip${f}`, `dip${f}`]) { fist[d] = max(d); hyper[d] = min(d); }
  const thumb = { mcp1: max('mcp1'), ip1: max('ip1') }, thumbHyper = { mcp1: min('mcp1'), ip1: min('ip1') };
  const wExt = { wrist_fe: min('wrist_fe') }, wFlex = { wrist_fe: max('wrist_fe') };
  out.poses = { fist, thumb, hyper, thumbHyper, wExt, wFlex };

  // 1. tendon excursion (mm, + = pulled proximally) between two poses
  const glideOf = (pose, name) => { const c = mv.cables.list.find((x) => x.name === name); return mv.measure(pose)[c.id]; };
  const exc = (a, b, name) => -(glideOf(P(b), name) - glideOf(P(a), name)) * 1e3;
  out.excursion = {
    FDS3_fist: exc({}, fist, 'FDS3'), FDP3_fist: exc({}, fist, 'FDP3'), FPL1_fist: exc({}, thumb, 'FPL1'),
    FDS3_total: exc(wExt, { ...wFlex, ...fist }, 'FDS3'), FDP3_total: exc(wExt, { ...wFlex, ...fist }, 'FDP3'),
    FPL1_total: exc(wExt, { ...wFlex, ...thumb }, 'FPL1'),
  };
  // 2. median nerve glide at the wrist marker (mm, + = distal)
  const med = (a, b) => (glideOf(P(b), 'Median nerve') - glideOf(P(a), 'Median nerve')) * 1e3;
  out.median = { wrist: med({ wrist_fe: -60 }, { wrist_fe: 65 }), fingers: med({ ...hyper, ...thumbHyper }, { ...fist, ...thumb }) };
  log('excursions done');

  // 3. MCP collateral ligaments: end-to-end length (fixed-vertex centroids per side) at MCP 0 vs full flexion
  out.collateral = collaterals(app, P);
  // 4. tenodesis on: wrist 0 -> 60 extension
  mv.setTenodesis(true);
  mv.set({ wrist_fe: 0 }); const t0 = { ...mv.pose };
  mv.set({ wrist_fe: -60 }); const t1 = { ...mv.pose };
  mv.setTenodesis(false);
  out.tenodesis = [1, ...F].map((f) => f === 1 ? ['thumb', t0.mcp1, t1.mcp1, t0.ip1, t1.ip1]
    : [NAME[f], t0[`mcp${f}_flex`], t1[`mcp${f}_flex`], t0[`pip${f}`], t1[`pip${f}`]]);
  log('collaterals + tenodesis done');

  // 5. presets: tissue inside bone, flexor tendons in their pulleys
  const I = new Inside(app); await tick();
  log('bone grids ' + I.bones.length + ', soft vertices ' + I.count);
  const set = (pose) => { mv.pose = pose; mv.apply(true); };
  set(P({})); const X0 = I.positions(), d0 = I.depth(X0), pul0 = pulleys(app);
  const rows = [];
  for (const pr of mv.presets) {
    const base = P({}), keys = pr.keys.map((k) => mv.full(k, base));
    let worst = null;
    for (const [k, pose] of keys.entries()) {
      set(pose);
      const d = I.depth(I.positions()), cnt = new Map(), dep = new Map(); let bad = 0;
      for (let i = 0; i < d.length; i++) {
        const nd = d[i] - d0[i]; if (nd <= 1e-3) continue;
        bad++; const s = I.names[i]; cnt.set(s, (cnt.get(s) || 0) + 1); dep.set(s, Math.max(dep.get(s) || 0, nd));
      }
      const pu = pulleys(app, pul0);
      const r = { preset: pr.name, key: k, bad, frac: bad / d.length, maxNew: Math.max(0, ...dep.values()) * 1e3,
        top: [...cnt.entries()].sort((a, b) => b[1] - a[1]).slice(0, 2).map(([s, c]) => `${s} ${c} (${(dep.get(s) * 1e3).toFixed(1)} mm)`), pulley: pu };
      const pmax = worst && worst.pulley.d > pu.d ? worst.pulley : pu;
      if (!worst || r.frac > worst.frac) worst = r;
      worst.pulley = pmax;
      await tick();
    }
    rows.push(worst); log(pr.name + ' ' + worst.bad);
  }
  out.presets = rows; out.softCount = I.count;
  set(P({}));
  return out;
}

function collaterals(app, P) {
  const mv = app.move, rig = mv.rig, s = app.anatomy.byName.get('Collateral metacarpophalangeal ligaments');
  const res = [];
  if (!s) return res;
  const neutral = rig.solve(P({})).list;
  for (const f of [1, 2, 3, 4, 5]) {
    const pp = `Proximal phalanx of ${ORD[f]} finger of hand`, J = rig.joints.find((j) => j.child === pp);
    const jm = rig.index[J.parent], jp = rig.index[pp], ax = J.axes[0], dof = ax.dof;
    const acc = {};                                   // side (+/-) -> bone -> centroid
    for (const m of s.meshes) {
      const a = m.geometry.attributes, Q = m.skeleton.boneInverses[0].clone().multiply(m.bindMatrix);
      for (let i = 0; i < a.position.count; i++) {
        if (a.skinWeight.getComponent(i, 0) < 0.98) continue;
        const j = a.skinIndex.getComponent(i, 0); if (j !== jm && j !== jp) continue;
        const v = { x: a.position.getX(i), y: a.position.getY(i), z: a.position.getZ(i) }, e = Q.elements;
        const x = [e[0] * v.x + e[4] * v.y + e[8] * v.z + e[12], e[1] * v.x + e[5] * v.y + e[9] * v.z + e[13], e[2] * v.x + e[6] * v.y + e[10] * v.z + e[14]];
        const side = (x[0] - ax.c.x) * ax.a.x + (x[1] - ax.c.y) * ax.a.y + (x[2] - ax.c.z) * ax.a.z > 0 ? '+' : '-';
        const k = side + j, o = acc[k] || (acc[k] = { n: 0, p: [0, 0, 0] });
        o.n++; x.forEach((t, q) => { o.p[q] += t; });
      }
    }
    const flex = rig.solve(P({ [dof]: rig.limits(dof)[1] })).list;
    for (const side of ['+', '-']) {
      const A = acc[side + jm], B = acc[side + jp]; if (!A || !B || A.n < 8 || B.n < 8) continue;
      const len = (M) => {
        const a = A.p.map((t) => t / A.n), b = B.p.map((t) => t / B.n), m1 = M[jm].elements, m2 = M[jp].elements;
        const T = (m, p) => [0, 1, 2].map((r) => m[r] * p[0] + m[r + 4] * p[1] + m[r + 8] * p[2] + m[r + 12]);
        const u = T(m1, a), w = T(m2, b); return Math.hypot(u[0] - w[0], u[1] - w[1], u[2] - w[2]) * 1e3;
      };
      res.push({ digit: NAME[f], side, flexDeg: rig.limits(dof)[1], L0: len(neutral), L90: len(flex) });
    }
  }
  return res;
}

// flexor tendons vs their digit's pulleys: distance pulley centroid -> nearest posed centreline sample;
// returns the largest increase vs neutral (mm) for tendons that pass through the pulley at neutral (< 5 mm)
function pulleys(app, base) {
  const mv = app.move, M = mv.mats, A = app.anatomy, out = {}, dig = { thumb: 1, index: 2, middle: 3, ring: 4, little: 5 };
  for (const s of A.structs) {
    const m = /^(A\d|Oblique) pulley of (\w+)/.exec(s.name); if (!m) continue;
    const d = dig[m[2]], mesh = s.meshes[0], a = mesh.geometry.attributes, Q = mesh.skeleton.boneInverses[0].clone().multiply(mesh.bindMatrix).elements;
    const j = a.skinIndex.getComponent(0, 0), e = M[j].elements, c = [0, 0, 0], n = a.position.count;
    for (let i = 0; i < n; i++) {
      const x = a.position.getX(i), y = a.position.getY(i), z = a.position.getZ(i);
      const p = [0, 1, 2].map((r) => Q[r] * x + Q[r + 4] * y + Q[r + 8] * z + Q[r + 12]);
      for (let r = 0; r < 3; r++) c[r] += (e[r] * p[0] + e[r + 4] * p[1] + e[r + 8] * p[2] + e[r + 12]) / n;
    }
    for (const t of mv.cables.list) {
      if (t.group !== 'flexor' || t.common || t.digit !== d) continue;
      let best = 1e9;
      for (let i = 0; i < t.n; i++) best = Math.min(best, Math.hypot(t.Pp[3 * i] - c[0], t.Pp[3 * i + 1] - c[1], t.Pp[3 * i + 2] - c[2]));
      out[s.name + '|' + t.name] = best;
    }
  }
  if (!base) return out;
  let w = { d: 0, what: '-' };
  for (const k in out) if (base[k] < 5e-3 && out[k] - base[k] > w.d) w = { d: out[k] - base[k], what: k.replace('|', ' / ') };
  return { d: w.d * 1e3, what: w.what };
}

// fps estimate per view: the page loop is stopped, "Full fist" is stepped at 72 Hz for 3 s and the per-frame JS work
// of the viewer loop is timed (pose + tissue update, scene matrices). Draw calls / triangles from one real render.
// GPU time cannot be measured here (headless WebGL is software-rendered).
export async function perf(app, view) {
  if (app.realism) await app.realism.setView(view);
  const mv = app.move, r = app.renderer;
  r.setAnimationLoop(null);
  r.render(app.scene, app.camera);
  const calls = r.info.render.calls, tris = r.info.render.triangles;
  mv.loop = true; mv.play(mv.presets.find((p) => p.name === 'Full fist'));
  for (let i = 0; i < 20; i++) mv.update(1 / 72);              // warm-up (JIT)
  const t = [];
  for (let i = 0; i < 216; i++) {
    const t0 = performance.now();
    mv.update(1 / 72); if (app.realism) app.realism.update(); app.root.updateMatrixWorld(); app.anatomy.updateClip();
    t.push(performance.now() - t0);
  }
  mv.stop(); mv.reset(); mv.update(0);
  t.sort((a, b) => a - b);
  return { view, mean: t.reduce((a, b) => a + b, 0) / t.length, p95: t[Math.floor(0.95 * t.length)], max: t[t.length - 1], calls, tris };
}
