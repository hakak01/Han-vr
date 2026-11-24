// Pose engine: port of tools/rigpose.py (+ forearm twist joints of tools/tisspose.py) and tools/rigrange.py.
// Pose = { dof: anatomical degrees }; a missing dof keeps its rest value (the model as built).
// solve(pose) -> Matrix4 per skin joint (cables.json "joints" order): rest geometry -> posed, model (glTF) space.
import * as THREE from 'three';

const D2R = Math.PI / 180;
const quad = (p, fe, ru) => p[0] * ru + p[1] * fe + p[2] * ru * ru + p[3] * ru * fe + p[4] * fe * fe;
const V = (a) => new THREE.Vector3(a[0], a[1], a[2]);

function rotvec(w) {                       // Rodrigues, w = axis * angle (rad)
  const th = w.length();
  return th < 1e-12 ? new THREE.Matrix4() : new THREE.Matrix4().makeRotationAxis(w.clone().divideScalar(th), th);
}
function about(c, R) {                     // rotation R about the point c
  const M = R.clone(), t = c.clone().sub(c.clone().applyMatrix4(R));
  return M.setPosition(t);
}
// np.interp(x, xp, arange(len(xp))) -> fractional index, clamped
function fidx(x, xp) {
  if (x <= xp[0]) return 0;
  for (let i = 1; i < xp.length; i++) if (x <= xp[i]) return i - 1 + (x - xp[i - 1]) / (xp[i] - xp[i - 1]);
  return xp.length - 1;
}

export const GROUPS = {   // UI order and labels
  wrist: [['pronation', 'Pronation + / supination −'], ['wrist_fe', 'Wrist flexion + / extension −'], ['wrist_ru', 'Ulnar dev + / radial dev −']],
  thumb: [['cmc1_fe', 'CMC flexion'], ['cmc1_aa', 'CMC palmar abduction'], ['mcp1', 'MCP flexion'], ['ip1', 'IP flexion']],
  f23: [2, 3].flatMap((f) => finger(f)),
  f45: [4, 5].flatMap((f) => finger(f)).concat([['cmc4', 'Ring CMC (cupping)'], ['cmc5', 'Little CMC (cupping)']]),
};
function finger(f) {
  const n = { 2: 'Index', 3: 'Middle', 4: 'Ring', 5: 'Little' }[f];
  return [[`mcp${f}_flex`, n + ' MCP flexion'], [`mcp${f}_abd`, n + ' MCP abd (radial +)'], [`pip${f}`, n + ' PIP'], [`dip${f}`, n + ' DIP']];
}

export class Rig {
  constructor(rig, cm, J, cab) {
    this.rig = rig; this.cm = cm; this.J = J; this.cab = cab;
    this.dofs = Object.keys(rig.dofs);
    this.rest = Object.fromEntries(this.dofs.map((d) => [d, rig.dofs[d].rest]));
    this.names = cab.joints;
    this.index = Object.fromEntries(cab.joints.map((n, i) => [n, i]));
    const W = rig.wrist;
    this.w = { flex: V(W.flex_axis), uln: V(W.ulnar_axis), dis: V(W.distal), ul: V(W.ulnar), dor: V(W.dorsal) };
    this.joints = rig.joints.map((j) => ({ ...j, axes: (j.axes || []).map((a) => ({ ...a, c: V(a.center), a: V(a.axis).normalize() })) }));
    this.restCarpal = {};
    for (const b of Object.keys(W.bones)) this.restCarpal[b] = this.carpalRT(b, W.rest.wrist_fe, W.rest.wrist_ru);
    const fa = cab.forearm;
    this.fa = { c: V(fa.center), a: V(fa.axis).normalize(), sign: rig.joints[0].axes[0].sign ?? 1 };
    this.neutral = Object.fromEntries(this.dofs.map((d) => [d, 0]));
  }

  value(pose, d) { return pose[d] ?? this.rest[d]; }

  // (min, max) anatomical degrees (joints.json, user-editable); always includes the rest value
  limits(d, pose = {}) {
    const J = this.J; let r;
    if (d === 'pronation') r = [-J.forearm.supination, J.forearm.pronation];
    else if (d === 'wrist_fe') r = [-J.wrist.extension, J.wrist.flexion];
    else if (d === 'wrist_ru') r = [-J.wrist.radial, J.wrist.ulnar];
    else if (d.endsWith('_abd')) {
      const t = Math.min(Math.max(this.value(pose, d.replace('_abd', '_flex')), 0), 90) / 90;
      const m = J.mcp.abd_at_0 + t * (J.mcp.abd_at_90 - J.mcp.abd_at_0); r = [-m, m];
    } else if (d === 'cmc4') r = [0, J.cmc_cup.ring];
    else if (d === 'cmc5') r = [0, J.cmc_cup.little];
    else if (d.startsWith('cmc1')) r = [-J.thumb_cmc[d].neg, J.thumb_cmc[d].pos];
    else {
      const k = { mcp1: 'thumb_mcp', ip1: 'thumb_ip' }[d] || (d.startsWith('mcp') ? 'mcp' : d.startsWith('pip') ? 'pip' : 'dip');
      r = [-J[k].hyperextension, J[k].flexion];
    }
    const rest = this.rest[d];
    return d.endsWith('_abd') ? r : [Math.min(r[0], rest), Math.max(r[1], rest)];
  }
  // full pose (every dof), clamped; flexion before abduction (abd range depends on MCP flexion)
  clamp(pose) {
    const out = {};
    for (const d of this.dofs) if (!d.endsWith('_abd')) { const [a, b] = this.limits(d); out[d] = Math.min(b, Math.max(a, this.value(pose, d))); }
    for (const d of this.dofs) if (d.endsWith('_abd')) { const [a, b] = this.limits(d, out); out[d] = Math.min(b, Math.max(a, this.value(pose, d))); }
    return out;
  }

  // Table S1 rotation (SP = 0) + centroid translation, plus the contact correction grid (rig.json wrist.corr)
  carpalRT(bone, fe, ru) {
    const W = this.rig.wrist, b = W.bones[bone], co = this.cm.bones[b.model], w = this.w;
    let R = rotvec(w.flex.clone().multiplyScalar(quad(co.FE, fe, ru) * D2R).addScaledVector(w.uln, quad(co.RU, fe, ru) * D2R));
    const t = new THREE.Vector3().addScaledVector(w.dis, quad(co.DPT, fe, ru)).addScaledVector(w.ul, quad(co.RUT, fe, ru))
      .addScaledVector(w.dor, quad(co.VDT, fe, ru)).multiplyScalar(W.scale_m);
    const C = W.corr;
    if (C && C.bones[bone]) {
      const x = fidx(fe, C.fe), y = fidx(ru, C.ru);
      const i = Math.min(Math.floor(x), C.fe.length - 2), j = Math.min(Math.floor(y), C.ru.length - 2), u = x - i, v = y - j;
      const bil = (g) => [0, 1, 2].map((k) => g[i][j][k] * (1 - u) * (1 - v) + g[i + 1][j][k] * u * (1 - v) +
        g[i][j + 1][k] * (1 - u) * v + g[i + 1][j + 1][k] * u * v);
      t.add(V(bil(C.bones[bone])));
      if (C.rot && C.rot[bone]) R = rotvec(V(bil(C.rot[bone]))).multiply(R);
    }
    return { R, t };
  }
  carpalMatrix(bone, fe, ru) {
    const { R, t } = this.carpalRT(bone, fe, ru), r0 = this.restCarpal[bone];
    const M = R.clone().multiply(r0.R.clone().transpose());
    const c = V(this.rig.wrist.bones[bone].centroid);
    return M.setPosition(c.clone().sub(r0.t).add(t).sub(c.clone().applyMatrix4(M)));
  }
  jointMatrix(j, pose) {
    if (j.type === 'carpal') return this.carpalMatrix(j.child, this.value(pose, 'wrist_fe'), this.value(pose, 'wrist_ru'));
    const M = new THREE.Matrix4();
    for (const ax of j.axes) {                  // first axis fixed in the parent, later ones ride on the child
      const ang = (this.value(pose, ax.dof) - this.rest[ax.dof]) * (ax.sign ?? 1) * D2R;
      M.multiply(about(ax.c, new THREE.Matrix4().makeRotationAxis(ax.a, ang)));
    }
    return M;
  }
  // -> { bone name: Matrix4 } and an array in skin-joint order
  solve(pose) {
    const W = {};
    for (const b of this.rig.fixed) W[b] = new THREE.Matrix4();
    for (const j of this.joints) W[j.child] = W[j.parent].clone().multiply(this.jointMatrix(j, pose));
    const ang = (this.value(pose, 'pronation') - this.rest.pronation) * this.fa.sign * D2R;
    for (const t of this.cab.twist)
      W[t.joint] = W.Ulna.clone().multiply(about(this.fa.c, new THREE.Matrix4().makeRotationAxis(this.fa.a, t.fraction * ang)));
    return { W, list: this.names.map((n) => W[n]) };
  }
}
