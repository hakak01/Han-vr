// Phase E4 accuracy test (headless): node tools/check_motion.mjs [--no-perf]
// Serves the repo, opens the real viewer in headless Chromium (Playwright), runs tools/motion_tests.js inside the
// page and writes the section "Phase E4" of docs/MOTION_CHECK.md: test | published | model | pass.
// three.js 0.170.0 is served from a local copy (CDN may be blocked): THREE_DIR, ./node_modules/three or
// data/raw/*/node_modules/three (npm i --prefix data/raw/npm three@0.170.0).
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';

const ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..');
const { chromium } = createRequire(import.meta.url)('playwright');
const THREE_DIR = [process.env.THREE_DIR, 'node_modules/three', 'data/raw/npm/node_modules/three', 'data/raw/enc/node_modules/three']
  .filter(Boolean).map((p) => path.resolve(ROOT, p)).find((p) => fs.existsSync(path.join(p, 'build/three.module.js')));
if (!THREE_DIR) { console.error('three@0.170.0 not found: npm i --prefix data/raw/npm three@0.170.0'); process.exit(1); }
const TYPES = { '.js': 'text/javascript', '.mjs': 'text/javascript', '.json': 'application/json', '.html': 'text/html', '.glb': 'model/gltf-binary',
  '.ktx2': 'image/ktx2', '.png': 'image/png', '.jpg': 'image/jpeg', '.hdr': 'application/octet-stream', '.css': 'text/css', '.wasm': 'application/wasm' };

const server = http.createServer((q, r) => {
  const f = path.join(ROOT, decodeURIComponent(new URL(q.url, 'http://x').pathname));
  const p = fs.existsSync(f) && fs.statSync(f).isDirectory() ? path.join(f, 'index.html') : f;
  if (!p.startsWith(ROOT) || !fs.existsSync(p)) { r.writeHead(404); return r.end(); }
  r.writeHead(200, { 'Content-Type': TYPES[path.extname(p)] || 'application/octet-stream', 'Content-Length': fs.statSync(p).size });
  fs.createReadStream(p).pipe(r);
});
await new Promise((res) => server.listen(0, res));
const URL0 = `http://localhost:${server.address().port}/`;

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
await page.route('https://cdn.jsdelivr.net/npm/three@0.170.0/**', (r) => {
  const u = new URL(r.request().url()); r.fulfill({ path: path.join(THREE_DIR, u.pathname.split('three@0.170.0/')[1]), contentType: 'text/javascript' });
});
page.on('console', (m) => console.log('  [page]', m.text().slice(0, 200)));
page.on('pageerror', (e) => console.log('  [page error]', e.message.slice(0, 300)));
const t0 = Date.now();
await page.goto(URL0, { waitUntil: 'load' });
await page.waitForFunction(() => window.app && window.app.move && window.app.menus, null, { timeout: 180000 });
console.log('viewer loaded in', ((Date.now() - t0) / 1000).toFixed(1), 's');
const R = await page.evaluate(async () => (await import('/tools/motion_tests.js')).run(window.app, (s) => console.log(s)));
const perf = [];
if (!process.argv.includes('--no-perf')) for (const v of ['present', 'realistic', 'cadaver'])
  perf.push(await page.evaluate(async (v) => (await import('/tools/motion_tests.js')).perf(window.app, v), v));
await browser.close(); server.close();

// ---- ARMS reference (tools/arms_ma.json, scaled to our hand like cables.json mech tables) -------------------
const A = JSON.parse(fs.readFileSync(path.join(ROOT, 'tools/arms_ma.json'))), SC = JSON.parse(fs.readFileSync(path.join(ROOT, 'assets/cables.json'))).mech_scale;
const interp = (x, rows) => x <= rows[0][0] ? rows[0][1] : x >= rows.at(-1)[0] ? rows.at(-1)[1]
  : (() => { const i = rows.findIndex((r) => r[0] >= x); const [a, b] = [rows[i - 1], rows[i]]; return a[1] + (x - a[0]) / (b[0] - a[0]) * (b[1] - a[1]); })();
function arms(tendon, from, to) {             // mm, + = pulled proximally; dofs missing from a pose are 0
  let E = 0;
  for (const d of new Set([...Object.keys(from), ...Object.keys(to)])) {
    const rows = A.moment_arms[tendon][d]; if (!rows) continue;
    const a = from[d] || 0, b = to[d] || 0, n = Math.max(1, Math.ceil(Math.abs(b - a))), s = d.startsWith('wrist') ? SC.wrist : /1$/.test(d) ? SC.thumb : SC['digit' + d.match(/\d/)[0]];
    for (let k = 0; k < n; k++) E += interp(a + (k + 0.5) * (b - a) / n, rows) * (b - a) / n * Math.PI / 180 * s;
  }
  return E;
}
const { fist, thumb, wExt, wFlex } = R.poses, X = R.excursion;
const fing = (f) => Object.fromEntries(Object.entries(fist).filter(([d]) => d.includes(String(f))));

// ---- table ---------------------------------------------------------------------------------------------------
const L = [], fails = [], f1 = (x) => x.toFixed(1), pc = (m, r) => { const x = 100 * (m - r) / r; return (x >= 0 ? '+' : '') + x.toFixed(0) + ' %'; };
const row = (test, pub, model, ok) => { L.push(`| ${test} | ${pub} | ${model} | ${ok ? 'PASS' : 'FAIL'} |`); if (!ok) fails.push(test); };
const within = (m, r) => Math.abs(m - r) <= 0.25 * Math.abs(r);
L.push('\n## Phase E4: accuracy test (tools/check_motion.mjs, headless Chromium on the live viewer code)\n');
L.push(`Run ${new Date().toISOString().slice(0, 10)}. Pass = within 25 % unless stated. Excursion reference = ARMS model ` +
  '(McFarland 2023) scaled to our hand (user decision 2026-11-24); Wehbe & Hunter 1985 (in vivo) shown with the % difference.\n');
L.push('| test | published | model | pass |\n|---|---|---|---|');
const wf = `wrist ext ${-wExt.wrist_fe} -> flex ${wFlex.wrist_fe}`;
for (const [k, t, w, from, to] of [
  ['FDS3_fist', 'FDS3, wrist 0, fingers 0 -> full fist (MCP 90, PIP 110, DIP 80)', 24, {}, fing(3)],
  ['FDP3_fist', 'FDP3, same', 32, {}, fing(3)],
  ['FPL1_fist', `FPL, wrist 0, thumb 0 -> MCP ${thumb.mcp1}, IP ${thumb.ip1}`, 27, {}, thumb],
  ['FDS3_total', `FDS3 total: ${wf} + fist`, 49, wExt, { ...wFlex, ...fing(3) }],
  ['FDP3_total', 'FDP3 total, same', 50, wExt, { ...wFlex, ...fing(3) }],
  ['FPL1_total', `FPL total: ${wf} + thumb flexion`, 35, wExt, { ...wFlex, ...thumb }]]) {
  const ref = arms(k.split('_')[0], from, to);
  row(`Excursion ${t} (mm)`, `ARMS ${f1(ref)}; Wehbe ${w} (model ${pc(X[k], w)})`, f1(X[k]), within(X[k], ref));
}
const M = R.median;
row('Median nerve glide at the wrist (marker 20 mm proximal to the lunate, vs radius), wrist ext 60 -> flex 65 (mm, - = proximal)', 'Wright 1996: 19.6', f1(M.wrist), within(Math.abs(M.wrist), 19.6));
row('Median nerve glide at the wrist, all digits hyperextension -> full flexion (mm, - = proximal)', 'Wright 1996: 9.7', f1(M.fingers), within(Math.abs(M.fingers), 9.7));
for (const c of R.collateral)
  row(`MCP collateral ligament ${c.digit} ${c.side === '+' ? 'side A' : 'side B'}: length MCP 0 -> ${c.flexDeg} (mm)`, 'longer in flexion',
    `${f1(c.L0)} -> ${f1(c.L90)}`, c.L90 > c.L0);
for (const [n, m0, m1, p0, p1] of R.tenodesis)
  row(`Tenodesis on, wrist 0 -> 60 ext: ${n} ${n === 'thumb' ? 'MCP / IP' : 'MCP / PIP'} (deg)`, 'fingers flex',
    `${f1(m0)} -> ${f1(m1)} / ${f1(p0)} -> ${f1(p1)}`, m1 > m0 + 5 && p1 > p0 + 5);
L.push('\nPresets (every key pose; worst key shown). Tissue: soft-tissue vertices newly > 1 mm inside a bone vs neutral ' +
  `(${R.softCount} vertices; pass < 0.2 %, as Phase E2). Pulley: largest growth of pulley-centre -> flexor-centreline distance vs neutral (pass < 1.5 mm).\n`);
L.push('| preset | new depth > 1 mm (vertices, %, max mm) | worst structures | pulley growth (mm) | pass |\n|---|---|---|---|---|');
for (const p of R.presets) {
  const ok = p.frac < 0.002 && p.pulley.d < 1.5; if (!ok) fails.push('preset ' + p.preset);
  L.push(`| ${p.preset} | ${p.bad} (${(100 * p.frac).toFixed(2)} %, ${f1(p.maxNew)}) | ${p.top.join('; ') || '-'} | ${f1(p.pulley.d)} ${p.pulley.what} | ${ok ? 'PASS' : 'FAIL'} |`);
}
if (perf.length) {
  L.push('\nfps estimate (desktop CPU of this run; I test on the Quest): "Full fist" animated at 72 Hz, JS work per frame of the ' +
    'viewer loop (pose, cables, tissue offsets, scene matrices). Pass = mean <= 5 ms and 95th percentile <= 13.9 ms ' +
    '(one frame at 72 fps; the GPU is not measured here).\n');
  L.push('| view | JS ms per frame: mean / 95th pct / max | draw calls | triangles | pass |\n|---|---|---|---|---|');
  for (const p of perf) {
    const ok = p.mean <= 5 && p.p95 <= 13.9; if (!ok) fails.push('fps ' + p.view);
    L.push(`| ${p.view} | ${p.mean.toFixed(2)} / ${p.p95.toFixed(2)} / ${p.max.toFixed(1)} | ${p.calls} | ${p.tris} | ${ok ? 'PASS' : 'FAIL'} |`);
  }
}
L.push('\nE4 failed: ' + (fails.length ? fails.join('; ') : 'none'));
const file = path.join(ROOT, 'docs/MOTION_CHECK.md'), txt = fs.readFileSync(file, 'utf8').split('\n## Phase E4')[0].trimEnd();
fs.writeFileSync(file, txt + '\n' + L.join('\n') + '\n');
console.log(L.join('\n'));
