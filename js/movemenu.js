// "Move" tab of the wrist menu (same canvas panel on desktop, phone and VR): joint sliders with degree readout,
// animated presets (speed, loop), tenodesis, mirror my hand, teaching aids, reset.
import { GROUPS } from './rig.js';

const PAGES = [['presets', 'Presets'], ['wrist', 'Wrist'], ['thumb', 'Thumb'], ['f23', 'Index·Mid'], ['f45', 'Ring·Little'], ['aids', 'Aids']];

export class MoveMenu {
  constructor(app, move) { this.app = app; this.move = move; this.page = 'presets'; this.grabOn = true; }
  refresh() { if (this.app.menus) this.app.menus.refresh(); }
  // throttled redraw while the pose changes (sliders follow animations, info panel shows live excursions)
  live() {
    const M = this.app.menus; if (!M) return;
    if (M.tab === 'move' && !M.menu.drag && this.page !== 'presets' && this.page !== 'aids') M.buildMenu();
    if (M.A.selected || this.move.aids.rings) M.buildInfo();
  }

  build(p) {
    const w = 96;
    PAGES.forEach(([k, l], i) => p.button(15 + i * (w + 7), 80, w, 46, l, () => { this.page = k; this.refresh(); },
      { active: this.page === k, size: 18 }));
    if (GROUPS[this.page]) this.sliders(p, GROUPS[this.page]);
    else this[this.page](p);
    p.text(625, 785, this.app.fps ? this.app.fps + ' fps' : '', { size: 18, align: 'right', color: '#9aa' });
  }

  sliders(p, list) {
    const M = this.move; let y = 140;
    for (const [d, label] of list) {
      const [lo, hi] = M.rig.limits(d, M.pose), val = M.pose[d];
      p.text(20, y + 23, label, { size: 19, color: '#dde' });
      p.text(625, y + 23, Math.round(val) + '°', { size: 21, align: 'right', bold: true });
      const ro = p.items[p.items.length - 1];
      p.slider(300, y, 255, 46, (val - lo) / Math.max(hi - lo, 1e-6), (v) => {
        const [a, b] = M.rig.limits(d, M.pose);
        M.set({ [d]: a + v * (b - a) }); ro.label = Math.round(M.pose[d]) + '°';
      });
      y += 50;
    }
    p.text(20, y + 16, M.tenodesis ? 'Tenodesis is on: fingers follow the wrist.' : (M.link ? 'DIP follows PIP (×' + M.ratio.toFixed(2) + ')' : ''),
      { size: 17, color: '#9aa' });
    this.bottom(p);
  }

  presets(p) {
    const M = this.move, pr = M.presets, run = M.anim && M.anim.pr;
    const main = pr.filter((x) => !/^Kapandji/.test(x.name)), kap = pr.filter((x) => /^Kapandji/.test(x.name));
    let y = 138;
    main.forEach((x, i) => {
      const c = i % 3, r = Math.floor(i / 3);
      p.button(15 + c * 205, y + r * 52, 198, 46, x.name, () => M.play(x), { size: 18, active: run === x });
    });
    y += Math.ceil(main.length / 3) * 52 + 8;
    if (kap.length) {
      p.text(20, y + 10, 'Kapandji opposition score', { size: 18, color: '#9aa' }); y += 24;
      kap.forEach((x, i) => p.button(15 + i * 61, y, 56, 46, x.name.replace(/\D+/g, ''), () => M.play(x), { size: 20, active: run === x }));
      y += 58;
    }
    p.text(20, y + 23, 'Speed ' + M.speed.toFixed(2) + '×', { size: 20 });
    const sp = p.items[p.items.length - 1];
    p.slider(190, y, 435, 46, (M.speed - 0.25) / 2.25, (v) => { M.speed = 0.25 + 2.25 * v; sp.label = 'Speed ' + M.speed.toFixed(2) + '×'; });
    y += 56;
    p.button(15, y, 198, 50, (M.loop ? '✓ ' : '') + 'Loop', () => { M.loop = !M.loop; this.refresh(); }, { active: M.loop });
    p.button(220, y, 198, 50, 'Stop', () => M.stop(), { active: !!run, color: '#b03030' });
    p.button(425, y, 200, 50, (M.tenodesis ? '✓ ' : '') + 'Tenodesis', () => M.setTenodesis(!M.tenodesis), { active: M.tenodesis });
    this.bottom(p);
  }

  aids(p) {
    const M = this.move, A = M.aids, G = M.grab; let y = 140;
    const tog = (label, on, f, note) => {
      p.button(15, y, 300, 50, (on ? '✓ ' : '') + label, f, { active: on, size: 20 });
      if (note) p.text(330, y + 25, note, { size: 16, color: '#9aa', wrap: 290, maxLines: 2 });
      y += 58;
    };
    tog('Tenodesis', M.tenodesis, () => M.setTenodesis(!M.tenodesis), 'wrist extension flexes the fingers (constant FDP/FPL length)');
    tog('Mirror my hand', G.mirror, () => { G.mirror = !G.mirror; this.refresh(); },
      this.app.inXR() ? (G.mirror ? G.mirrorStatus || 'starting…' : 'put the controllers down; fingers + thumb') : 'VR headset with hand tracking');
    tog('Glide markers', A.rings, () => A.setRings(!A.rings), 'rings every 5 mm on tendons, median + ulnar nerves');
    tog('Ligament strain', A.strain, () => A.setStrain(!A.strain), 'green slack → red taut (length vs neutral)');
    tog('Ghost of neutral', !!(A.ghost && A.ghost.visible), () => A.setGhost(!(A.ghost && A.ghost.visible)), 'bones + skin in the neutral pose');
    tog('Grab bones', this.grabOn, () => { this.grabOn = !this.grabOn; this.refresh(); },
      this.app.inXR() ? 'point at a bone, squeeze, drag' : 'press on a bone and drag');
    tog('Link DIP to PIP', M.link, () => { M.link = !M.link; this.refresh(); }, 'active flexion');
    p.text(20, y + 23, 'DIP = ' + M.ratio.toFixed(2) + ' × PIP', { size: 20 });
    const rt = p.items[p.items.length - 1];
    p.slider(250, y, 375, 46, M.ratio, (v) => { M.ratio = v; rt.label = 'DIP = ' + v.toFixed(2) + ' × PIP'; });
    this.bottom(p);
  }

  bottom(p) {
    const M = this.move;
    p.button(15, 712, 300, 56, 'Reset pose', () => { M.reset(); this.refresh(); });
    p.button(325, 712, 300, 56, 'Neutral (all 0°)', () => { M.set(M.rig.neutral); this.refresh(); });
  }
}
