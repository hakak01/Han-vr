// Soft tissues that do more than follow the skin (hand_rig.glb attributes, see tools/bind_tissues.py):
//   _cid/_cable: vertex rides on a cable (tendon material point, or sheath following its tendon)
//   _slide: muscle belly slides with its tendons and bulges (constant volume); lumbricals move with FDP
//   _shift: lateral bands move volarly with PIP flexion
// Each pose: target position on the CPU -> per-vertex offset 'aDelta' added BEFORE GPU skinning (bind space), so the
// procedural / fibre textures (which use bind-pose positions) stay attached to the tissue.
import * as THREE from 'three';
import { seg, SEGF } from './cables.js';

// every skinned shader: transformed += aDelta (after begin_vertex, where the texture coordinates are taken)
THREE.ShaderChunk.skinning_pars_vertex = '#ifdef USE_SKINNING\nattribute vec3 aDelta;\n#endif\n' + THREE.ShaderChunk.skinning_pars_vertex;
THREE.ShaderChunk.skinning_vertex = '#ifdef USE_SKINNING\ntransformed += aDelta;\n#endif\n' + THREE.ShaderChunk.skinning_vertex;

const _s = new THREE.Sphere();
const geomZero = (n) => new THREE.BufferAttribute(new Int8Array(3 * n), 3);

export class TissueMove {
  constructor(anatomy, cables, rig) {
    this.cables = cables; this.rig = rig; this.cab = cables.cab;
    this.items = []; this.skinned = []; this.structCables = new Map();
    const lat = this.cab.lateral_bands;
    for (const s of anatomy.structs) for (const m of s.meshes) {
      if (!m.isSkinnedMesh) continue;
      m.frustumCulled = false; this.skinned.push(m); this.restSphere(m);
      const a = m.geometry.attributes, L = lat[s.name];
      if (!(a._cid || a._slide || (a._shift && L))) { m.geometry.setAttribute('aDelta', geomZero(a.position.count)); continue; }
      this.items.push(this.prepare(m, s, L));
    }
  }

  // rest bounding sphere (model space) and the skin joints used: posed sphere = union of the rest sphere carried by
  // each joint (a skinned vertex is a blend of those) + the largest soft-tissue offset of the mesh
  restSphere(m) {
    const a = m.geometry.attributes, Q = new THREE.Matrix4().multiplyMatrices(m.skeleton.boneInverses[0], m.bindMatrix);
    m.geometry.computeBoundingSphere();
    const sp = m.geometry.boundingSphere.clone().applyMatrix4(Q), js = new Set();
    for (let i = 0; i < a.position.count; i++) for (let j = 0; j < 4; j++) if (a.skinWeight.getComponent(i, j) > 0) js.add(a.skinIndex.getComponent(i, j));
    m.userData.rs = { c: sp.center, r: sp.radius, js: [...js], margin: 0.001 };
    m.boundingSphere = new THREE.Sphere(sp.center.clone(), sp.radius);
    const tm = this;
    m.raycast = function (ray, hits) { tm.raycast(this, ray, hits); };
  }
  // exact ray test on the posed mesh: positions skinned once per pose (cached), then a plain Mesh raycast
  raycast(m, ray, hits) {
    _s.copy(m.boundingSphere).applyMatrix4(m.matrixWorld);
    if (!ray.ray.intersectsSphere(_s)) return;
    const R = m.userData.rs, v = new THREE.Vector3();
    if (R.ver !== this.version) {
      const a = m.geometry.attributes, n = a.position.count;
      if (!R.geo) { R.geo = new THREE.BufferGeometry(); R.geo.setIndex(m.geometry.index); R.geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * n), 3)); R.proxy = new THREE.Mesh(R.geo); }
      const P = R.geo.attributes.position.array;
      for (let i = 0; i < n; i++) { m.getVertexPosition(i, v); P[3 * i] = v.x; P[3 * i + 1] = v.y; P[3 * i + 2] = v.z; }
      R.geo.attributes.position.needsUpdate = true; R.geo.computeBoundingSphere(); R.geo.boundingBox = null; R.ver = this.version;
    }
    R.proxy.material = m.material; R.proxy.matrixWorld.copy(m.matrixWorld);
    const out = []; R.proxy.raycast(ray, out);
    for (const h of out) { h.object = m; hits.push(h); }
  }
  spheres(M) {
    this.version = (this.version || 0) + 1;
    const c = new THREE.Vector3(), p = new THREE.Vector3();
    for (const m of this.skinned) {
      const R = m.userData.rs, S = m.boundingSphere || (m.boundingSphere = new THREE.Sphere());
      c.set(0, 0, 0);
      for (const j of R.js) c.add(p.copy(R.c).applyMatrix4(M[j]));
      c.divideScalar(R.js.length);
      let r = 0;
      for (const j of R.js) r = Math.max(r, p.copy(R.c).applyMatrix4(M[j]).distanceTo(c));
      S.center.copy(c); S.radius = r + R.r + R.margin; m.boundingBox = null;
    }
  }

  prepare(m, s, L) {
    const a = m.geometry.attributes, n = a.position.count, sk = m.skeleton;
    const Q = new THREE.Matrix4().multiplyMatrices(sk.boneInverses[0], m.bindMatrix);
    if (sk.boneInverses.some((b) => !b.equals(sk.boneInverses[0]))) console.warn('unequal inverse binds', s.name);
    const Qi = new THREE.Matrix3().setFromMatrix4(Q).invert().elements;
    const idx = [], v = new THREE.Vector3();
    for (let i = 0; i < n; i++) {
      const c = a._cid ? Math.round(a._cid.getX(i)) : -1, sl = a._slide ? Math.round(a._slide.getX(i)) : -1;
      const sh = a._shift && L ? a._shift.getX(i) : 0;
      if (c !== -1 || sl !== -1 || sh > 1e-4) idx.push(i);
    }
    const k = idx.length, it = { m, s, k, idx: Int32Array.from(idx), Qi, L,
      rest: new Float64Array(3 * k), J: new Int32Array(4 * k), W: new Float64Array(4 * k),
      cid: new Int32Array(k), cb: new Float64Array(4 * k), sl: new Float64Array(4 * k), sh: new Float64Array(k) };
    const lum = new Map(), ids = this.structCables.get(s.name) || new Set();
    idx.forEach((i, q) => {
      v.fromBufferAttribute(a.position, i).applyMatrix4(Q);
      it.rest[3 * q] = v.x; it.rest[3 * q + 1] = v.y; it.rest[3 * q + 2] = v.z;
      for (let j = 0; j < 4; j++) { it.J[4 * q + j] = a.skinIndex.getComponent(i, j); it.W[4 * q + j] = a.skinWeight.getComponent(i, j); }
      it.cid[q] = a._cid ? Math.round(a._cid.getX(i)) : -1;
      if (a._cable) for (let j = 0; j < 4; j++) it.cb[4 * q + j] = a._cable.getComponent(i, j);
      if (a._slide) for (let j = 0; j < 4; j++) it.sl[4 * q + j] = a._slide.getComponent(i, j);
      it.sl[4 * q] = a._slide ? Math.round(it.sl[4 * q]) : -1;
      if (it.sl[4 * q] >= 0) lum.set(it.sl[4 * q], Math.min(lum.get(it.sl[4 * q]) ?? 1, it.sl[4 * q + 3]));
      it.sh[q] = a._shift && L ? a._shift.getX(i) : 0;
      const c = it.cid[q] >= 0 ? it.cid[q] : it.cid[q] <= -2 ? -it.cid[q] - 2 : it.sl[4 * q];
      if (c >= 0) ids.add(c);
    });
    it.lum = lum;                                    // slide group is a lumbrical when its min bulge weight < 0
    // cable-bound vertices sharing (cable, u) share one posed frame: computed once per group
    const gm = new Map(); it.gid = new Int32Array(k).fill(-1); it.gC = []; it.gU = []; it.gF = [];
    for (let q = 0; q < k; q++) {
      const c = it.cid[q]; if (c === -1) continue;
      const key = c + ':' + it.cb[4 * q];
      if (!gm.has(key)) { gm.set(key, it.gC.length); it.gC.push(this.cables.byId.get(c >= 0 ? c : -c - 2)); it.gU.push(it.cb[4 * q]); it.gF.push(c < 0); }
      it.gid[q] = gm.get(key);
    }
    it.frames = new Float64Array(12 * it.gC.length);
    const bones = new Set(); for (let i = 0; i < 4 * k; i++) if (it.W[i]) bones.add(it.J[i]);
    if (L) bones.add(this.rig.index[L.bone]);
    it.bones = [...bones]; it.cabs = new Set(it.gC);
    // slide-only vertices: kind 1 = belly (straight muscle axis + bulge), 2 = lumbrical (rides on its FDP); the
    // offset is in bind space, so no skin matrix is needed for them
    it.kind = new Uint8Array(k); it.ax = new Float64Array(3 * k); it.perp = new Float64Array(3 * k); it.ref = new Array(k);
    for (let q = 0; q < k; q++) {
      const id = it.sl[4 * q]; if (it.cid[q] !== -1 || id < 0) continue;
      const c = this.cables.byId.get(id), x = it.rest.subarray(3 * q, 3 * q + 3);
      if (lum.get(id) < 0) {
        const i = seg(c.sr, it.sl[4 * q + 1]), g = SEGF[0], T0 = c.T0, t = [0, 1, 2].map((r) => T0[3 * i + r] * (1 - g) + T0[3 * i + 3 + r] * g);
        const l = Math.hypot(...t) || 1; it.cabs.add(c); it.kind[q] = 2; it.ref[q] = c; it.ax.set(t.map((v) => v / l), 3 * q);
      } else {
        const mu = this.cab.muscles[c.muscle], O = mu.origin, t = [0, 1, 2].map((r) => mu.junction[r] - O[r]), l = Math.hypot(...t);
        const d = [0, 1, 2].map((r) => x[r] - O[r]), sd = (d[0] * t[0] + d[1] * t[1] + d[2] * t[2]) / (l * l);
        for (const id2 of mu.cables) it.cabs.add(this.cables.byId.get(id2));
        it.kind[q] = 1; it.ref[q] = c.muscle; it.ax.set(t.map((v) => v / l), 3 * q); it.perp.set(d.map((v, r) => v - sd * t[r]), 3 * q);
      }
    }
    this.structCables.set(s.name, ids);
    it.delta = new Float32Array(3 * n);
    const attr = new THREE.BufferAttribute(it.delta, 3); attr.setUsage(THREE.DynamicDrawUsage);
    m.geometry.setAttribute('aDelta', attr);
    const D = it.delta;
    m.getVertexPosition = function (i, t) {          // raycast / bounds see the same position as the GPU
      THREE.Mesh.prototype.getVertexPosition.call(this, i, t);
      t.x += D[3 * i]; t.y += D[3 * i + 1]; t.z += D[3 * i + 2];
      return this.applyBoneTransform(i, t);
    };
    return it;
  }

  // M: skin-joint matrices (Rig.solve().list); cables already updated for this pose
  update(M, pose) {
    const E = M.map((x) => x.elements), C = this.cables, fr = new Float64Array(12), B = new Float64Array(12), cab = this.cab;
    const bul = {};                                  // constant-volume bulge per muscle: r x 1/sqrt(axial scale)
    for (const [k, mu] of Object.entries(cab.muscles)) bul[k] = 1 / Math.sqrt(Math.max((mu.L + (C.muscle[k] || 0)) / mu.L, 0.3)) - 1;
    const ch = C.changed;
    for (const it of this.items) {
      if (!it.bones.some((j) => ch[j]) && ![...it.cabs].some((c) => c.fresh)) continue;   // nothing it depends on moved
      const { k, idx, rest, J, W, cid, cb, sl, sh, Qi, delta, L, kind, ax, perp, ref } = it;
      let sv = null;
      if (L) { const e = E[this.rig.index[L.bone]], d = L.dir, f = L.max_m * Math.min(Math.max(this.rig.value(pose, L.dof) / 90, 0), 1);
        sv = [0, 1, 2].map((r) => f * (e[r] * d[0] + e[r + 4] * d[1] + e[r + 8] * d[2])); }
      const FR = it.frames; let mx = 0;
      for (let g = 0; g < it.gC.length; g++) {
        const cc = it.gC[g], u = it.gU[g];
        C.frame(cc, it.gF[g] ? C.follow(cc, u) : C.sigma(cc, u), fr); FR.set(fr, 12 * g);
      }
      for (let q = 0; q < k; q++) {
        let px = 0, py = 0, pz = 0, bx = 0, by = 0, bz = 0;   // posed-space offset (needs B^-1) / bind-space offset
        const c = cid[q], kd = kind[q];
        if (kd) {
          const f = sl[4 * q + 2], a = 3 * q;
          if (kd === 2) { const m = f * C.slide(ref[q], sl[4 * q + 1]); bx = m * ax[a]; by = m * ax[a + 1]; bz = m * ax[a + 2]; }
          else { const m = f * C.muscle[ref[q]], b = bul[ref[q]] * sl[4 * q + 3];
            bx = m * ax[a] + b * perp[a]; by = m * ax[a + 1] + b * perp[a + 1]; bz = m * ax[a + 2] + b * perp[a + 2]; }
          if (!(sv && sh[q])) {
            const o = 3 * idx[q], d2 = bx * bx + by * by + bz * bz; if (d2 > mx) mx = d2;
            delta[o] = Qi[0] * bx + Qi[3] * by + Qi[6] * bz; delta[o + 1] = Qi[1] * bx + Qi[4] * by + Qi[7] * bz;
            delta[o + 2] = Qi[2] * bx + Qi[5] * by + Qi[8] * bz;
            continue;
          }
        }
        B.fill(0);                                // blended affine (column-major 3x4)
        for (let j = 0; j < 4; j++) {
          const w = W[4 * q + j]; if (!w) continue;
          const e = E[J[4 * q + j]];
          for (let r = 0; r < 3; r++) { B[r] += w * e[r]; B[r + 3] += w * e[r + 4]; B[r + 6] += w * e[r + 8]; B[r + 9] += w * e[r + 12]; }
        }
        const x = rest[3 * q], y = rest[3 * q + 1], z = rest[3 * q + 2];
        if (c !== -1) {
          const o = 12 * it.gid[q], ox = cb[4 * q + 1], oy = cb[4 * q + 2], oz = cb[4 * q + 3];
          px = FR[o] + FR[o + 6] * ox + FR[o + 9] * oy + FR[o + 3] * oz - (B[0] * x + B[3] * y + B[6] * z + B[9]);
          py = FR[o + 1] + FR[o + 7] * ox + FR[o + 10] * oy + FR[o + 4] * oz - (B[1] * x + B[4] * y + B[7] * z + B[10]);
          pz = FR[o + 2] + FR[o + 8] * ox + FR[o + 11] * oy + FR[o + 5] * oz - (B[2] * x + B[5] * y + B[8] * z + B[11]);
        }
        if (sv && sh[q]) { px += sh[q] * sv[0]; py += sh[q] * sv[1]; pz += sh[q] * sv[2]; }
        if (px || py || pz) {                      // B_lin^-1 * posed offset
          const a0 = B[0], a1 = B[1], a2 = B[2], a3 = B[3], a4 = B[4], a5 = B[5], a6 = B[6], a7 = B[7], a8 = B[8];
          const c0 = a4 * a8 - a5 * a7, c1 = a5 * a6 - a3 * a8, c2 = a3 * a7 - a4 * a6, det = a0 * c0 + a1 * c1 + a2 * c2;
          bx += (c0 * px + c1 * py + c2 * pz) / det;
          by += ((a7 * a2 - a1 * a8) * px + (a0 * a8 - a6 * a2) * py + (a6 * a1 - a0 * a7) * pz) / det;
          bz += ((a1 * a5 - a4 * a2) * px + (a3 * a2 - a0 * a5) * py + (a0 * a4 - a3 * a1) * pz) / det;
        }
        const d2 = bx * bx + by * by + bz * bz; if (d2 > mx) mx = d2;
        const o = 3 * idx[q];                      // bind (model) space -> quantised mesh space
        delta[o] = Qi[0] * bx + Qi[3] * by + Qi[6] * bz; delta[o + 1] = Qi[1] * bx + Qi[4] * by + Qi[7] * bz;
        delta[o + 2] = Qi[2] * bx + Qi[5] * by + Qi[8] * bz;
      }
      it.m.geometry.attributes.aDelta.needsUpdate = true;
      it.m.userData.rs.margin = Math.max(it.m.userData.rs.margin, Math.sqrt(mx) + 0.002);
    }
  }
}
