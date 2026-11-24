// Direct manipulation (Phase E3):
// - Grab: point at a bone, squeeze (VR) or press (desktop, Move tab open), drag -> rotates the joint just proximal
//   to it; grabbing a distal phalanx bends the whole finger / thumb (CCD IK). Always within the joint ranges.
// - Mirror my hand (WebXR hand tracking): the user's right-hand finger and thumb angles drive the model.
import * as THREE from 'three';

const ORD = { second: 2, third: 3, fourth: 4, fifth: 5, first: 1 };
const D2R = Math.PI / 180;
const XRF = { 2: 'index-finger', 3: 'middle-finger', 4: 'ring-finger', 5: 'pinky-finger' };

// dofs moved by grabbing a bone
function dofsOf(bone, link) {
  if (bone === 'Radius') return ['pronation'];
  if (/^(Scaphoid|Lunate|Triquetrum|Pisiform|Capitate|Hamate|Trapezoid|Trapezium) bone$|^(Second|Third) metacarpal/.test(bone)) return ['wrist_fe', 'wrist_ru'];
  if (bone === 'Fourth metacarpal bone') return ['cmc4'];
  if (bone === 'Fifth metacarpal bone') return ['cmc5'];
  if (bone === 'First metacarpal bone') return ['cmc1_fe', 'cmc1_aa'];
  const m = /^(Proximal|Middle|Distal) phalanx of (\w+) finger/.exec(bone);
  if (!m) return [];
  const f = ORD[m[2]];
  if (f === 1) return m[1] === 'Proximal' ? ['mcp1'] : ['cmc1_fe', 'cmc1_aa', 'mcp1', 'ip1'];
  if (m[1] === 'Proximal') return [`mcp${f}_flex`, `mcp${f}_abd`];
  if (m[1] === 'Middle') return [`pip${f}`];
  return [`mcp${f}_flex`, `mcp${f}_abd`, `pip${f}`].concat(link ? [] : [`dip${f}`]);
}

export class Grab {
  constructor(app, move) {
    this.app = app; this.move = move; this.g = null; this.mirror = false; this.mirrorStatus = '';
    this.ray = new THREE.Raycaster();
    const mat = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide });
    this.proxies = [];          // rigid copies of the bones (no CPU skinning needed to raycast them)
    for (const s of app.anatomy.structs) if (s.layer === 'bone') for (const m of s.meshes) {
      if (!m.isSkinnedMesh) continue;
      const p = new THREE.Mesh(m.geometry, mat); p.matrixAutoUpdate = false;
      p.userData = { bone: s.name, struct: s, Q: new THREE.Matrix4().multiplyMatrices(m.skeleton.boneInverses[0], m.bindMatrix) };
      this.proxies.push(p);
    }
    this.attachDom();
  }
  get model() { return this.app.anatomy.center.children[0]; }    // glTF scene = parent of the skin joints

  // ---- picking + IK ----------------------------------------------------------------------------------------
  pickBone(ray) {
    const P = this.model.matrixWorld, M = this.move.rig.solve(this.move.pose).W;
    for (const p of this.proxies) if (M[p.userData.bone]) p.matrixWorld.multiplyMatrices(P, M[p.userData.bone]).multiply(p.userData.Q);
    const h = ray.intersectObjects(this.proxies, false).find((x) => this.app.anatomy.kept(x.point));
    return h ? { bone: h.object.userData.bone, struct: h.object.userData.struct, point: h.point, distance: h.distance } : null;
  }
  begin(ray) {
    const h = this.pickBone(ray); if (!h) return false;
    const dofs = dofsOf(h.bone, this.move.link); if (!dofs.length) return false;
    const P = this.model.matrixWorld, M = this.move.rig.solve(this.move.pose).W[h.bone];
    const local = h.point.clone().applyMatrix4(new THREE.Matrix4().multiplyMatrices(P, M).invert());   // rest coords
    this.g = { bone: h.bone, dofs, local, dist: h.distance };
    this.move.stop(); this.app.anatomy.select(h.struct); this.app.menus.refresh();
    return true;
  }
  drag(ray) {
    const g = this.g; if (!g) return;
    const t = ray.ray.at(g.dist, new THREE.Vector3()).applyMatrix4(this.model.matrixWorld.clone().invert());
    const v = new THREE.Vector3(), rig = this.move.rig;
    this.move.set(this.ik(g.dofs, (pose) => v.copy(g.local).applyMatrix4(rig.solve(pose).W[g.bone]).distanceToSquared(t)));
  }
  end() { const was = !!this.g; this.g = null; return was; }

  // CCD: one dof at a time (distal first), 1-D Newton on err() with numeric derivatives, clamped to range
  // -> partial pose for Move.set()
  ik(dofs, err, iters = 4) {
    const M = this.move, h = 1;
    let pose = { ...M.pose };
    const withV = (p, d, x) => {
      const q = { ...p, [d]: x };
      if (M.link && d.startsWith('pip')) q['dip' + d.slice(3)] = M.ratio * x;
      return M.rig.clamp(q);
    };
    let e0 = err(pose);
    for (let it = 0; it < iters; it++) for (const d of [...dofs].reverse()) {
      const x = pose[d], ep = err(withV(pose, d, x + h)), em = err(withV(pose, d, x - h));
      const g = (ep - em) / (2 * h), H = (ep - 2 * e0 + em) / (h * h);
      let step = H > 1e-12 ? -g / H : -Math.sign(g) * 5;
      step = Math.max(-25, Math.min(25, step));
      for (let k = 0; k < 4; k++) {
        const q = withV(pose, d, x + step), e = err(q);
        if (e < e0) { pose = q; e0 = e; break; }
        step /= 2;
      }
    }
    const part = Object.fromEntries(dofs.map((d) => [d, pose[d]]));
    for (const d of dofs) if (d.startsWith('pip') && M.link) part['dip' + d.slice(3)] = pose['dip' + d.slice(3)];
    return part;
  }

  // ---- desktop / phone: press on a bone while the Move tab is open ------------------------------------------
  attachDom() {
    const el = this.app.renderer.domElement, ndc = new THREE.Vector2();
    const cast = (e) => { ndc.set(e.clientX / innerWidth * 2 - 1, -e.clientY / innerHeight * 2 + 1); this.ray.setFromCamera(ndc, this.app.camera); return this.ray; };
    addEventListener('pointerdown', (e) => {
      if (e.target !== el || this.app.inXR() || !this.app.menus || this.app.menus.tab !== 'move' || !this.move.ui.grabOn) return;
      if (!this.begin(cast(e))) return;
      e.stopImmediatePropagation(); this.pid = e.pointerId;
    }, { capture: true });
    addEventListener('pointermove', (e) => { if (this.g && e.pointerId === this.pid) { this.drag(cast(e)); e.stopImmediatePropagation(); } }, { capture: true });
    addEventListener('pointerup', (e) => { if (this.g && e.pointerId === this.pid) { this.end(); e.stopImmediatePropagation(); } }, { capture: true });
  }
  // ---- VR: right-hand squeeze (xr.js asks first; true = handled here) -----------------------------------------
  squeeze(h, down) {
    if (!down) { if (this.xrHand === h) { this.xrHand = null; this.end(); return true; } return false; }
    if (h.hand !== 'right' || this.app.menus.tab !== 'move' || !this.move.ui.grabOn) return false;
    this.ray.setFromXRController(h.ctrl);
    if (!this.begin(this.ray)) return false;
    this.xrHand = h; return true;
  }

  update(dt) {
    if (this.xrHand) { this.ray.setFromXRController(this.xrHand.ctrl); this.drag(this.ray); }
    if (this.mirror) this.mirrorStep(dt);
  }

  // ---- Mirror my hand: WebXR hand joints -> joint angles ------------------------------------------------------
  mirrorStep(dt) {
    const r = this.app.renderer, frame = r.xr.getFrame && r.xr.getFrame(), ref = r.xr.getReferenceSpace();
    const src = frame && [...frame.session.inputSources].find((s) => s.hand && s.handedness === 'right');
    if (!src) { this.mirrorStatus = 'put the controllers down'; return; }
    const P = (n) => { const j = src.hand.get(n), p = j && frame.getJointPose(j, ref); return p && new THREE.Vector3().copy(p.transform.position); };
    const w = P('wrist'), i2 = P('index-finger-phalanx-proximal'), i5 = P('pinky-finger-phalanx-proximal');
    if (!w || !i2 || !i5) { this.mirrorStatus = 'hand not tracked'; return; }
    this.mirrorStatus = 'tracking';
    const palm = i2.clone().sub(w).cross(i5.clone().sub(w)).normalize();      // palmar (right hand)
    const tgt = {};
    const ang = (a, b, k) => {             // signed angle a -> b about k (projected), degrees; + = flexion
      const x = a.clone().addScaledVector(k, -a.dot(k)), y = b.clone().addScaledVector(k, -b.dot(k));
      return Math.atan2(x.clone().cross(y).dot(k), x.dot(y)) / D2R;
    };
    for (const f of [2, 3, 4, 5]) {
      const n = XRF[f], J = ['metacarpal', 'phalanx-proximal', 'phalanx-intermediate', 'phalanx-distal', 'tip'].map((s) => P(n + '-' + s));
      if (J.some((x) => !x)) continue;
      const [a, b, c, d] = [0, 1, 2, 3].map((i) => J[i + 1].clone().sub(J[i]).normalize());
      const k = a.clone().cross(palm).normalize();                         // lateral axis, points radially
      Object.assign(tgt, { [`mcp${f}_flex`]: ang(a, b, k), [`pip${f}`]: ang(b, c, k), [`dip${f}`]: ang(c, d, k),
        [`mcp${f}_abd`]: Math.asin(Math.max(-1, Math.min(1, b.dot(k)))) / D2R });
    }
    const T = ['thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip'].map(P);
    if (T.every(Boolean)) {
      const [a, b, c] = [0, 1, 2].map((i) => T[i + 1].clone().sub(T[i]).normalize());
      tgt.mcp1 = Math.acos(Math.min(1, a.dot(b))) / D2R; tgt.ip1 = Math.acos(Math.min(1, b.dot(c))) / D2R;
      Object.assign(tgt, this.thumbCMC(a, w, i2, i5, P('middle-finger-phalanx-proximal')));
    }
    const s = 1 - Math.exp(-dt / 0.1), cur = this.move.pose, part = {};   // ~100 ms smoothing
    for (const [d, v] of Object.entries(tgt)) if (isFinite(v)) part[d] = cur[d] + (v - cur[d]) * s;
    const link = this.move.link; this.move.link = false; this.move.set(part); this.move.link = link;
  }
  // tracked first-metacarpal direction, expressed in the palm frame, matched by the model's CMC1 (2 dofs, IK)
  thumbCMC(a, w, i2, i5, m3) {
    if (!m3) return {};
    const frame = (d, r) => { const e1 = d.clone().normalize(), e2 = r.clone().addScaledVector(e1, -r.dot(e1)).normalize(); return [e1, e2, e1.clone().cross(e2)]; };
    const Ft = frame(m3.clone().sub(w), i2.clone().sub(i5));
    const rig = this.move.rig, W = rig.solve(this.move.pose).W, jc = (id) => new THREE.Vector3(...rig.rig.joints.find((j) => j.id === id).axes[0].center);
    const cap = new THREE.Vector3(...rig.rig.wrist.bones['Capitate bone'].centroid).applyMatrix4(W['Capitate bone']);
    const mc = (f) => jc('mcp' + f).applyMatrix4(W[['', '', 'Second', 'Third', '', 'Fifth'][f] + ' metacarpal bone']);
    const Fm = frame(mc(3).sub(cap), mc(2).sub(mc(5)));
    const t = new THREE.Vector3(); Ft.forEach((e, i) => t.addScaledVector(Fm[i], a.dot(e)));
    const c0 = jc('cmc1'), c1 = jc('mcp1'), v = new THREE.Vector3(), u = new THREE.Vector3();
    return this.ik(['cmc1_fe', 'cmc1_aa'], (pose) => { const M1 = rig.solve(pose).W['First metacarpal bone'];
      return v.copy(c1).applyMatrix4(M1).sub(u.copy(c0).applyMatrix4(M1)).normalize().distanceToSquared(t); }, 3);
  }
}
