// Movement controller (Phase E3): pose state -> skeleton + cables + soft tissues; animated presets, DIP link,
// tenodesis. Grab / IK / hand mirroring: grab.js. Teaching aids: aids.js. Menu tab: movemenu.js.
import { Rig } from './rig.js';
import { Cables } from './cables.js';
import { TissueMove } from './tissuemove.js';
import { Grab } from './grab.js';
import { Aids } from './aids.js';
import { MoveMenu } from './movemenu.js';

const FINGERS = [2, 3, 4, 5];
const ease = (t) => t * t * (3 - 2 * t);
const SEG = 1.6;              // seconds per preset segment at speed 1
// tenodesis: resting cascade at wrist 0 (more flexion towards the little finger), thumb
const CASCADE = { 2: 0.25, 3: 0.3, 4: 0.35, 5: 0.4, 1: 0.25 };

export class Move {
  constructor(app, gltfScene, data) {
    this.app = app; this.data = data;
    const rig = this.rig = new Rig(data.rig, data.cm, data.joints, data.cab);
    this.cables = new Cables(data.cab, rig);
    this.tissue = new TissueMove(app.anatomy, this.cables, rig);
    this.bones = rig.names.map((n) => {
      let b = null; gltfScene.traverse((o) => { if (o.isBone && (o.userData.name || o.name) === n) b = o; });
      if (b) b.matrixAutoUpdate = false; else console.warn('no skin joint', n);
      return b;
    });
    this.presets = data.presets.presets || [];
    this.ratio = data.joints.active_dip_per_pip ?? 0.667; this.link = true;
    this.speed = 1; this.loop = true; this.anim = null; this.tenodesis = false;
    this.pose = rig.clamp({}); this.target = null; this.dirty = true; this.version = 0;
    this.mats = null;
    this.ref = this.tenoRef();
    this.aids = new Aids(app, this);
    this.grab = new Grab(app, this);
    this.ui = new MoveMenu(app, this);
    this.neutralInfo = this.measure(rig.clamp(rig.neutral));   // excursions are shown relative to neutral
    this.apply();
  }

  // ---- pose state ------------------------------------------------------------------------------------------
  // set some dofs (degrees); 'link' couples DIP = ratio x PIP for the fingers whose PIP was set
  set(part, { stopAnim = true } = {}) {
    if (stopAnim) this.anim = null;
    const p = { ...this.pose, ...part };
    if (this.link) for (const f of FINGERS) if (`pip${f}` in part && !(`dip${f}` in part)) p[`dip${f}`] = this.ratio * p[`pip${f}`];
    this.pose = this.rig.clamp(this.tenodesis ? this.teno(p) : p); this.dirty = true;
  }
  reset() { this.anim = null; this.pose = this.rig.clamp({}); this.dirty = true; }
  value(d) { return this.pose[d]; }

  // ---- presets ---------------------------------------------------------------------------------------------
  // keys: partial poses; fingers/thumb not listed -> 0, forearm/wrist not listed -> unchanged; dipN "link" -> ratio x PIP
  full(key, base) {
    const p = { ...base };
    for (const d of this.rig.dofs) if (!['pronation', 'wrist_fe', 'wrist_ru'].includes(d)) p[d] = 0;
    for (const [d, v] of Object.entries(key)) if (v !== 'link') p[d] = v;
    for (const [d, v] of Object.entries(key)) if (v === 'link') p[d] = this.ratio * p[d.replace('dip', 'pip')];
    return this.rig.clamp(p);
  }
  play(pr) {
    const base = { ...this.pose }, keys = pr.keys.map((k) => this.full(k, base));
    const from = this.full(pr.from || {}, base);              // loop partner of the keys (default: open hand)
    this.anim = { pr, path: [{ ...this.pose }, ...keys], from, keys, i: 0, t: 0, hold: 0 };
    this.ui.refresh();
  }
  stepAnim(dt) {
    const A = this.anim; if (!A) return;
    if (A.hold > 0) { A.hold -= dt; return; }
    A.t += dt * this.speed / (A.pr.seconds || SEG);
    const a = A.path[A.i], b = A.path[A.i + 1], t = ease(Math.min(A.t, 1)), p = {};
    for (const d of this.rig.dofs) p[d] = a[d] + (b[d] - a[d]) * t;
    this.pose = this.rig.clamp(this.tenodesis && !A.pr.keepTeno ? this.teno(p) : p); this.dirty = true;
    if (A.t < 1) return;
    A.t = 0; A.i++; A.hold = 0.35;
    if (A.i < A.path.length - 1) return;
    if (!this.loop) { this.anim = null; this.ui.refresh(); return; }
    // loop: back and forth between the 'from' pose and the keys
    A.path = A.pr.cycle ? [A.keys[A.keys.length - 1], ...A.keys] : [b, ...[...A.keys].reverse().slice(1), A.from, ...A.keys];
    A.i = 0;
  }
  stop() { this.anim = null; this.ui.refresh(); }

  // ---- tenodesis: flexor (FDP, FPL) musculotendinous length kept constant -> fingers follow the wrist --------
  tendon(name) { return this.cables.list.find((c) => c.name === name); }
  shortening(c, p) {                     // sum of the ARMS excursion tables (m), larger = shorter path
    let E = 0;
    for (const [d, t] of Object.entries(c.mech || {})) {
      const x = Math.min(Math.max((this.rig.value(p, d) - t.deg0) / t.step, 0), t.F.length - 1), i = Math.min(Math.floor(x), t.F.length - 2);
      E += t.F[i] + (x - i) * (t.F[i + 1] - t.F[i]);
    }
    return E;
  }
  cascade(f, s, p) {
    if (f === 1) { p.mcp1 = 50 * s; p.ip1 = 80 * s; return p; }
    p[`mcp${f}_flex`] = 90 * s; p[`pip${f}`] = 100 * s; p[`dip${f}`] = this.ratio * 100 * s; return p;
  }
  tenoRef() {
    const ref = {}, p0 = this.rig.clamp({ ...this.rig.neutral });
    for (const f of [1, ...FINGERS]) {
      const c = this.tendon(f === 1 ? 'FPL1' : 'FDP' + f); if (!c) continue;
      ref[f] = { c, E: this.shortening(c, this.cascade(f, CASCADE[f], { ...p0 })) };
    }
    return ref;
  }
  teno(p) {
    for (const [f, r] of Object.entries(this.ref)) {
      let lo = 0, hi = 1;
      const E = (s) => this.shortening(r.c, this.cascade(+f, s, { ...p }));
      if (E(0) >= r.E) hi = 0; else if (E(1) <= r.E) lo = 1;
      for (let k = 0; k < 18 && hi > lo; k++) { const m = (lo + hi) / 2; if (E(m) < r.E) lo = m; else hi = m; }
      this.cascade(+f, (lo + hi) / 2, p);
      if (+f > 1) p[`mcp${f}_abd`] = 0;
    }
    return p;
  }
  setTenodesis(on) { this.tenodesis = on; if (on) this.set({}, { stopAnim: false }); this.ui.refresh(); }

  // ---- apply -----------------------------------------------------------------------------------------------
  // bones always; soft tissue (cables, vertex offsets, aids) unless soft = false. The skin carries the soft tissue
  // with the bones every frame, so offsets one or two frames old are not visible while animating.
  apply(soft = true) {
    const { list } = this.rig.solve(this.pose);
    this.mats = list;
    this.bones.forEach((b, i) => { if (b) { b.matrix.copy(list[i]); b.matrixWorldNeedsUpdate = true; } });
    this.tissue.spheres(list);
    this.dirty = false; this.version++; this.softDirty = !soft;
    if (!soft) return;
    const t = performance.now();
    this.cables.update(list, this.pose);
    this.tissue.update(list, this.pose);
    this.aids.update();
    this.cost = 0.8 * (this.cost || 0) + 0.2 * (performance.now() - t);   // ms, smoothed
  }
  // tendon excursion (dT, < 0 = proximal) and nerve glide at the wrist for a pose, without moving anything
  measure(pose) {
    const { list } = this.rig.solve(pose);
    this.cables.update(list, pose);
    const out = {};
    for (const c of this.cables.list) out[c.id] = this.glide(c);
    return out;
  }
  // glide (m) of the cable's material, + = distal: tendon whole-unit slide; nerve/vessel at the wrist level
  glide(c) {
    if (c.mode === 'tendon') return c.dT;
    if (c.mode === 'common') return c.d;
    if (c.uWrist === undefined) {           // marker: 20 mm proximal (along the cable) to its point nearest the lunate
      const ce = this.data.rig.wrist.bones['Lunate bone'].centroid; let best = 1e9;
      for (let i = 0; i < c.n; i++) {
        const d = Math.hypot(c.P[3 * i] - ce[0], c.P[3 * i + 1] - ce[1], c.P[3 * i + 2] - ce[2]);
        if (d < best) { best = d; c.uWrist = Math.max(c.sr[i] - 0.02, 0); }
      }
    }
    // material at the marker relative to its bed point (proximal to the wrist bend = relative to the radius)
    return this.cables.sigma(c, c.uWrist) - this.cables.follow(c, c.uWrist);
  }
  excursion(c) { return (this.glide(c) - (this.neutralInfo ? this.neutralInfo[c.id] : 0)) * 1e3; }   // mm vs neutral

  update(dt) {
    this.stepAnim(dt);
    this.grab.update(dt);
    if (!this.dirty && !this.softDirty) return;
    const busy = !!(this.anim || this.grab.g || this.grab.mirror), every = busy ? Math.min(3, 1 + Math.floor((this.cost || 0) / 5)) : 1;
    this.tick = (this.tick || 0) + 1;
    if (this.dirty) this.apply(this.tick % every === 0);
    else if (!busy || this.tick % every === 0) this.apply(true);
    const now = performance.now();
    if (now - (this.lastUi || 0) > 120) { this.lastUi = now; this.ui.live(); }
  }
}
