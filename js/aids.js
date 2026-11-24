// Teaching aids (Phase E3): glide markers (rings every 5 mm on the tendons and the median/ulnar nerves, with live
// excursion in the info panel), ligament strain colour (end-to-end length vs neutral), ghost of the neutral pose.
import * as THREE from 'three';

const STEP = 0.005;                     // ring spacing on the material (m)
const NERVES = ['Median nerve', 'Ulnar nerve'];
const SUMMARY = ['FDS3', 'FDP3', 'FPL1', 'EDC3', 'Median nerve', 'Ulnar nerve'];
const STRAIN = 0.10;                    // full colour at +/- 10 % end-to-end length change
const GREEN = new THREE.Color(0x2ecc40), YELLOW = new THREE.Color(0xf1c40f), RED = new THREE.Color(0xe74c3c);

export class Aids {
  constructor(app, move) {
    this.app = app; this.move = move; this.rings = false; this.strain = false; this.ghost = null;
    this.model = app.anatomy.center.children[0];
    this.buildRings(); this.buildLigaments();
  }

  // ---- glide markers --------------------------------------------------------------------------------------
  buildRings() {
    const C = this.move.cables, list = [];
    for (const c of C.list) {
      const ok = (c.kind === 'tendon' && !c.common) || NERVES.includes(c.name); if (!ok) continue;
      const u0 = c.kind === 'tendon' ? (c.u_mtj || 0) : 0, L = c.sr[c.n - 1];
      for (let u = Math.ceil(u0 / STEP) * STEP; u < L - 0.002; u += STEP) list.push({ c, u, r: rad(c, u) });
    }
    this.ringList = list;
    const geo = new THREE.TorusGeometry(1, 0.09, 4, 18);
    const mat = new THREE.MeshBasicMaterial({ color: 0xffffff, clippingPlanes: this.app.anatomy.clipPlanes });
    this.ringMesh = new THREE.InstancedMesh(geo, mat, list.length);
    list.forEach((x, i) => this.ringMesh.setColorAt(i, new THREE.Color(x.c.kind === 'tendon' ? 0x1d4ed8 : 0x111111)));
    this.ringMesh.frustumCulled = false; this.ringMesh.visible = false;
    this.model.add(this.ringMesh);
  }
  updateRings() {
    const C = this.move.cables, fr = new Float64Array(12), m = new THREE.Matrix4(), s = this.ringMesh;
    const t = new THREE.Vector3(), n = new THREE.Vector3(), b = new THREE.Vector3();
    this.ringList.forEach((x, i) => {
      C.frame(x.c, C.sigma(x.c, x.u), fr);
      t.set(fr[3], fr[4], fr[5]); n.set(fr[6], fr[7], fr[8]); b.set(fr[9], fr[10], fr[11]);
      const r = x.r * 1.12;
      m.makeBasis(n.multiplyScalar(r), b.multiplyScalar(r), t.multiplyScalar(r)).setPosition(fr[0], fr[1], fr[2]);
      s.setMatrixAt(i, m);
    });
    s.instanceMatrix.needsUpdate = true;
  }
  setRings(on) { this.rings = on; this.ringMesh.visible = on; if (on) this.updateRings(); this.app.menus.refresh(); }

  // live excursion lines for the info panel: the selected structure's cables, else a summary while rings are on
  lines(s) {
    const M = this.move, C = M.cables, out = [];
    const fmt = (c) => {
      const mm = M.excursion(c), what = c.kind === 'tendon' ? (c.common ? 'sheet' : 'excursion') : 'glide at wrist';
      return `${c.name.replace(' nerve', ' n.')}: ${what} ${Math.abs(mm).toFixed(1)} mm ${mm < -0.05 ? 'proximal' : mm > 0.05 ? 'distal' : ''}`;
    };
    const ids = s && this.move.tissue.structCables.get(s.name);
    const byStruct = C.list.filter((c) => (ids && ids.has(c.id)) || (s && c.structure === s.name));
    if (byStruct.length) byStruct.slice(0, 3).forEach((c) => out.push(fmt(c)));
    else if (this.rings && !s) for (const n of SUMMARY) { const c = C.list.find((x) => x.name === n); if (c) out.push(fmt(c)); }
    return out;
  }

  // ---- ligament strain colour (per vertex: strain of the nearest bone pair the ligament joins) ----------------
  // ends = centroids of the vertices fixed (weight >= 0.98) to each bone; pairs only within one digit ray, within
  // the carpus / forearm, or metacarpal-metacarpal (multi-finger meshes must not pair two fingers)
  buildLigaments() {
    const rig = this.move.rig, neutral = rig.solve(rig.clamp(rig.neutral)).list, v = new THREE.Vector3(), names = rig.names;
    const ray = (j) => { const n = names[j], m = /(first|second|third|fourth|fifth) (finger|metacarpal)|^(First|Second|Third|Fourth|Fifth) metacarpal/.exec(n);
      return m ? ['first', 'second', 'third', 'fourth', 'fifth'].indexOf((m[1] || m[3]).toLowerCase()) + 1 : 0; };
    const isMC = (j) => / metacarpal bone$/.test(names[j]);
    const ok = (a, b) => ray(a) === ray(b) || (ray(a) === 0 && (ray(b) === 0 || isMC(b))) || (ray(b) === 0 && isMC(a)) || (isMC(a) && isMC(b));
    this.ligs = [];
    for (const s of this.app.anatomy.structs) if (s.layer === 'ligament_capsule') for (const m of s.meshes) {
      if (!m.isSkinnedMesh) continue;
      const a = m.geometry.attributes, n = a.position.count, Q = new THREE.Matrix4().multiplyMatrices(m.skeleton.boneInverses[0], m.bindMatrix);
      const P = new Float64Array(3 * n), acc = new Map();
      for (let i = 0; i < n; i++) {
        v.fromBufferAttribute(a.position, i).applyMatrix4(Q); P[3 * i] = v.x; P[3 * i + 1] = v.y; P[3 * i + 2] = v.z;
        if (a.skinWeight.getComponent(i, 0) < 0.98) continue;
        const j = a.skinIndex.getComponent(i, 0), e = acc.get(j) || { n: 0, p: new THREE.Vector3() };
        e.n++; e.p.add(v); acc.set(j, e);
      }
      const ends = [...acc.entries()].filter(([, e]) => e.n >= 8).map(([j, e]) => ({ j, p: e.p.divideScalar(e.n) }));
      const pairs = [];
      for (let x = 0; x < ends.length; x++) for (let y = x + 1; y < ends.length; y++) if (ok(ends[x].j, ends[y].j)) {
        const pr = [ends[x], ends[y]], L0 = this.len(pr, neutral);
        if (L0 > 1e-3 && L0 < 0.04) pairs.push({ ends: pr, L0, e: 0 });
      }
      if (!pairs.length) continue;
      const pv = new Uint8Array(n), A0 = new THREE.Vector3(), B0 = new THREE.Vector3(), seg = new THREE.Line3();
      for (let i = 0; i < n; i++) {                   // vertex -> nearest pair (segment between its rest ends)
        v.set(P[3 * i], P[3 * i + 1], P[3 * i + 2]); let best = 1e9;
        pairs.forEach((q, k) => { seg.set(A0.copy(q.ends[0].p), B0.copy(q.ends[1].p)); const d = seg.closestPointToPoint(v, true, A0).distanceTo(v); if (d < best) { best = d; pv[i] = k; } });
      }
      const col = new THREE.BufferAttribute(new Float32Array(3 * n).fill(1), 3);
      this.ligs.push({ m, pairs, pv, col, name: s.name });
    }
  }
  len(ends, M) { return ends[0].p.clone().applyMatrix4(M[ends[0].j]).distanceTo(ends[1].p.clone().applyMatrix4(M[ends[1].j])); }
  updateStrain() {
    const M = this.move.mats, c = new THREE.Color();
    for (const L of this.ligs) {
      const cs = L.pairs.map((q) => {
        q.e = this.len(q.ends, M) / q.L0 - 1; const t = Math.max(-1, Math.min(1, q.e / STRAIN));
        return t < 0 ? c.lerpColors(YELLOW, GREEN, -t).clone() : c.lerpColors(YELLOW, RED, t).clone();
      });
      for (const mat of Object.values(L.m.userData.mats)) {          // also materials made later (realistic view)
        if (!mat.vertexColors) { mat.vertexColors = true; mat.needsUpdate = true; }
        if (mat.color.getHex() !== 0xffffff) { mat.userData.strainBase = mat.color.clone(); mat.color.set(0xffffff); }
      }
      const A = L.col.array;
      for (let i = 0; i < L.pv.length; i++) { const k = cs[L.pv[i]]; A[3 * i] = k.r; A[3 * i + 1] = k.g; A[3 * i + 2] = k.b; }
      L.col.needsUpdate = true;
    }
  }
  setStrain(on) {
    this.strain = on;
    for (const L of this.ligs) {
      if (on) L.m.geometry.setAttribute('color', L.col); else L.m.geometry.deleteAttribute('color');
      for (const mat of Object.values(L.m.userData.mats)) {
        if (on && !mat.userData.strainBase) { mat.userData.strainBase = mat.color.clone(); mat.color.set(0xffffff); }
        if (!on && mat.userData.strainBase) { mat.color.copy(mat.userData.strainBase); delete mat.userData.strainBase; }
        mat.vertexColors = on; mat.needsUpdate = true;
      }
    }
    if (on) this.updateStrain();
    const R = this.app.realism; if (!on && R && R.ready) R.applyPalette(R.palette);
    this.app.menus.refresh();
  }

  // ---- ghost of the neutral pose (bones + skin, translucent) -----------------------------------------------
  setGhost(on) {
    if (on && !this.ghost) this.buildGhost();
    if (this.ghost) this.ghost.visible = on;
    this.app.menus.refresh();
  }
  buildGhost() {
    const rig = this.move.rig, g = new THREE.Group(), { list } = rig.solve(rig.clamp(rig.neutral));
    const bones = list.map((M) => { const b = new THREE.Bone(); b.matrixAutoUpdate = false; b.matrix.copy(M); g.add(b); return b; });
    const orig = this.move.bones;
    const mk = (color, opacity) => new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false });
    const mats = { bone: mk(0x9ad0ff, 0.28), skin: mk(0xbfe3ff, 0.10) };
    for (const s of this.app.anatomy.structs) if (mats[s.layer] && s.name !== 'Nail plate') for (const m of s.meshes) {
      if (!m.isSkinnedMesh) continue;
      const sm = new THREE.SkinnedMesh(m.geometry, mats[s.layer]);
      sm.bind(new THREE.Skeleton(m.skeleton.bones.map((b) => bones[orig.indexOf(b)]), m.skeleton.boneInverses), m.bindMatrix);
      sm.frustumCulled = false; sm.renderOrder = 2; g.add(sm);
    }
    this.ghost = g; this.model.add(g);
  }

  update() {
    if (this.rings) this.updateRings();
    if (this.strain) this.updateStrain();
  }
}

function rad(c, u) {   // cable radius at rest arc length u
  const s = c.sr; let i = 0;
  while (i < c.n - 2 && s[i + 1] < u) i++;
  return c.r[i] || 0.001;
}
