// Isometri: dua robot forklift membuat jalur Skenario 1 di peta Gudang dengan
// perencana dqfd-murni. Semua angka dibaca dari data uji nyata:
//   Forklift Robot Program/uji_nyata/20261024_073221_Gudang/hasil.jsonl
//   Camera/peta_constraint/Gudang.json
// Geometri robot dari "Obsidian Vault/03 Model Robot ID 01.md" (sama dengan konfig.py).
//
//   node isometri/build.mjs           -> isometri/s1_gudang_dqfd.html (gelap)
//   node isometri/build.mjs --light   -> isometri/s1_gudang_dqfd-light.html
import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { homedir } from "node:os";

const ANATOMY = join(homedir(), ".claude/skills/synced/45072784-2a89-4635-a3b8-1ba58ecdf7e9_baae30c7-fd03-492b-ab56-fba199477359/anatomy");
const k = await import(join(ANATOMY, "kit/iso-kit.mjs"));
const G = await import(join(ANATOMY, "kit/lathe.mjs"));
const { kitScript } = await import(join(ANATOMY, "scripts/inline-kit.mjs"));
import { makeRobot } from "./robot.mjs";
import { PARAM as PURSUIT, simulateMulti } from "./pursuit.mjs";

const HERE = dirname(fileURLToPath(import.meta.url));
const REPO = join(HERE, "..");
const theme = process.argv.includes("--light") ? "light" : "dark";

// ---------------------------------------------------------------- data asli
const RUN_DIR = join(REPO, "Forklift Robot Program/uji_nyata/20261024_073221_Gudang");
const MAP = JSON.parse(readFileSync(join(REPO, "Camera/peta_constraint/Gudang.json"), "utf8"));
const RUNS = readFileSync(join(RUN_DIR, "hasil.jsonl"), "utf8").trim().split("\n").map((line) => JSON.parse(line));
const UJI = RUNS.find((r) => r.skenario === "S1" && r.perencana === "dqfd-murni" && r.ulang === 0);

// Geometri robot (cm), acuan poros roda depan, u ke depan, v ke samping.
const BODY_FRONT = 4.0, BODY_REAR = 23.0, HALF_W = 17.5 / 2, HEIGHT = 16.5, WHEELBASE = 22.5;
const TAG = 10.0, TAG_TO_FRONT = 9.5;
const PRIM_CM = 7.3;                                   // panjang satu primitif
const KAPPA_MAX = Math.tan((41.3 * Math.PI) / 180) / WHEELBASE;
const MARGIN_CM = UJI.robot["1"].rencana[0].margin_cm; // 8 cm

function planOf(id) {
  const r = UJI.robot[id];
  const plan = r.rencana[0];
  const pts = plan.jalur_cm;
  // Model jauh menaruh 4 titik per primitif (7,3 / 4 = 1,825 cm). Titik pertama
  // yang jaraknya lain adalah awal kata RS dari model dekat: itulah pose serah.
  let serah = pts.length - 1;
  for (let i = 1; i < pts.length; i++) {
    if (Math.abs(Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) - PRIM_CM / 4) > 0.02) { serah = i - 1; break; }
  }
  if (serah % 4) throw new Error(`pose serah ID ${id} tidak jatuh di batas primitif`);
  const goal = r.tujuan;
  return {
    id: Number(id),
    pts,
    serah,
    prims: serah / 4,
    goal,
    msFar: plan.rincian_ms.dqfd_jauh,
    msNear: plan.rincian_ms.dqfd_dekat,
    msAll: plan.ms,
    serahDist: Math.hypot(pts[serah][0] - goal[0], pts[serah][1] - goal[1]),
    cusps: pts.map((p, i) => (i && p[3] !== pts[i - 1][3] ? i - 1 : -1)).filter((i) => i >= 0),
    galat: r.galat_cm,
  };
}
const PLANS = [planOf("1"), planOf("2")];
for (const p of PLANS) console.log(`ID 0${p.id}: ${p.prims} primitif, serah ${p.serahDist.toFixed(1)} cm, ${p.msAll.toFixed(1)} ms (jauh ${p.msFar.toFixed(1)}, dekat ${p.msNear.toFixed(1)}), ${p.cusps.length} pembalikan`);

// ---------------------------------------------------------------- dunia
// 1 cm = S satuan dunia. x dan y sama dengan citra kamera (y ke bawah peta),
// yang di kit ini memang arah lantai yang benar bila dilihat dari atas.
const S = 2.5;
const W = (x, y, z = 0) => [x * S, y * S, z * S];
const WIDTH = 1040, HEIGHT_VB = 700;
const PLATE = { x: -22 * S, y: -26 * S, w: 286 * S, d: 266 * S, r: 5 * S };
const PLATE_T = 3.2 * S, FOOT_H = 2.2 * S;
const WALL_H = 6;            // cm, tinggi dinding hanya ilustrasi

let P = k.fitProjection(
  [...k.boxCorners(PLATE, -PLATE_T - FOOT_H, 0), W(0, 0, HEIGHT + 2), W(250, 0, WALL_H)],
  WIDTH, HEIGHT_VB, { pad: 26, azimuth: 60, elevation: 40 },
);
const at = (p) => k.iso(p, P);
const S_ = (paths, style) => k.solidSvg(paths, style);
const L = (d, style) => k.lineSvg(d, style);
const D = (points, style) => k.dotsSvg(points, style);
const depth = (p) => k.depthOf(p, P);

// ---------------------------------------------------------------- lantai
function floor() {
  const out = [`<path class="iso-halo" d="${k.haloOf(PLATE, -PLATE_T - FOOT_H, PLATE_T, P)}"/>`];
  const feet = k.corners(PLATE, 16 * S).sort((a, b) => depth([...a, 0]) - depth([...b, 0]));
  for (const [x, y] of feet) {
    out.push(S_(k.cylinder(x, y, 5.2 * S, -PLATE_T - FOOT_H, 0.7 * S, P, 32), { tone: "lo" }));
    out.push(S_(k.cylinder(x, y, 4.0 * S, -PLATE_T - FOOT_H + 0.7 * S, FOOT_H - 0.7 * S, P, 32), { tone: "mid" }));
  }
  out.push(S_(k.slabOf(PLATE, -PLATE_T, PLATE_T, P, 8, 1.4), { tone: "mid" }));
  // Sambungan papan lantai: tiga papan memanjang x, garis dari tepi ke tepi.
  const seams = [61, 148].map((y) => k.lineOnTop([PLATE.x, y * S], [PLATE.x + PLATE.w, y * S], 0, P)).join("");
  out.push(L(seams, { tone: "lo" }));
  // Kisi 50 cm, tepi ke tepi.
  const grid = [];
  for (let x = 0; x <= 250; x += 50) grid.push(k.lineOnTop([x * S, PLATE.y], [x * S, PLATE.y + PLATE.d], 0, P));
  for (let y = 0; y <= 200; y += 50) if (y !== 0) grid.push(k.lineOnTop([PLATE.x, y * S], [PLATE.x + PLATE.w, y * S], 0, P));
  out.push(L(grid.join(""), { tone: "faint" }));
  // Sekrup sudut papan.
  const screws = k.corners(PLATE, 7 * S);
  out.push(L(screws.map(([x, y]) => k.ring(x, y, 1.6 * S, 0, P, 20)).join(""), { tone: "lo" }));
  out.push(D(screws.map(([x, y]) => at([x, y, 0])), { size: 0.7 }));
  // Penggaris cm di dua tepi depan: minor 5 cm, mayor 50 cm.
  const sx = { x: -4 * S, y: 224 * S, w: 258 * S, d: 6 * S, r: 0.6 * S };
  out.push(S_(k.slabOf(sx, 0, 0.35 * S, P, 3), { tone: "lo", crease: "none" }));
  const tx = k.topTicks(0, 250 * S, 5 * S, 10, sx.y + 0.15 * S, 0.35 * S, [1.6 * S, 3.4 * S], P, "x");
  out.push(L(tx.minor, { tone: "lo" }), L(tx.major, { tone: "mid" }));
  out.push(D([at([sx.x + 1.6 * S, sx.y + 3 * S, 0.35 * S]), at([sx.x + sx.w - 1.6 * S, sx.y + 3 * S, 0.35 * S])], { size: 0.6 }));
  const sy = { x: 254 * S, y: -4 * S, w: 6 * S, d: 222 * S, r: 0.6 * S };
  out.push(S_(k.slabOf(sy, 0, 0.35 * S, P, 3), { tone: "lo", crease: "none" }));
  const ty = k.topTicks(0, 210 * S, 5 * S, 10, sy.x + 0.15 * S, 0.35 * S, [1.6 * S, 3.4 * S], P, "y");
  out.push(L(ty.minor, { tone: "lo" }), L(ty.major, { tone: "mid" }));
  out.push(D([at([sy.x + 3 * S, sy.y + 1.6 * S, 0.35 * S]), at([sy.x + 3 * S, sy.y + sy.d - 1.6 * S, 0.35 * S])], { size: 0.6 }));
  // Pelat nama di sudut kiri depan: tiga garis cetak.
  const lp = { x: -18 * S, y: 222 * S, w: 11 * S, d: 8 * S, r: 0.8 * S };
  out.push(S_(k.slabOf(lp, 0, 0.4 * S, P, 3), { tone: "mid" }));
  out.push(L(k.planOutline(k.insetPlan(lp, 0.7 * S), 0.4 * S, P, 3), { tone: "faint" }));
  out.push(L([[1.8, 7], [2.8, 5.5], [3.8, 6.4]].map(([dy, len]) => k.lineOnTop([lp.x + 2 * S, lp.y + dy * S], [lp.x + (2 + len) * S, lp.y + dy * S], 0.4 * S, P)).join(""), { tone: "lo", free: true }));
  return out.join("");
}

// Bak tujuan: telapak badan robot di pose tujuan, dicat di lantai.
function local(pose, u, v, z = 0) {
  const t = (pose[2] * Math.PI) / 180, c = Math.cos(t), s = Math.sin(t);
  return [(pose[0] + u * c - v * s) * S, (pose[1] + u * s + v * c) * S, z * S];
}
const footRing = (pose, grow = 0) => [[BODY_FRONT + grow, HALF_W + grow], [-BODY_REAR - grow, HALF_W + grow], [-BODY_REAR - grow, -HALF_W - grow], [BODY_FRONT + grow, -HALF_W - grow]].map(([u, v]) => local(pose, u, v));
function goalBay(plan) {
  const g = plan.goal;
  const ring = footRing(g, 1.2);
  const inner = footRing(g, -0.6);
  const flat = k.extrude(ring.map(([x, y]) => [x, y]), 0, 0.12 * S, P);
  const out = [S_(flat, { tone: "mid", crease: "none" })];
  out.push(L(k.onTop(inner.map(([x, y]) => [x, y]), 0.12 * S, P, true), { tone: "faint" }));
  // Panah arah hadap tujuan: batang dari tepi dalam belakang, kepala segitiga
  // tertutup di depan, titik di poros roda depan (titik acuan pose tujuan).
  const flatPts = (list) => list.map(([u, v]) => { const p = local(g, u, v); return [p[0], p[1]]; });
  const zf = 0.12 * S;
  out.push(L(k.onTop(flatPts([[-BODY_REAR + 0.6, 0], [BODY_FRONT - 4.6, 0]]), zf, P), { tone: "mid" }));
  out.push(L(k.onTop(flatPts([[BODY_FRONT - 0.6, 0], [BODY_FRONT - 4.6, 2.6], [BODY_FRONT - 4.6, -2.6]]), zf, P, true), { tone: "mid" }));
  out.push(D([at(local(g, 0, 0, 0.12))], { size: 1.1, tone: "hi" }));
  return out.join("");
}

// ---------------------------------------------------------------- dinding
// Zona Z1-Z8 diekstrusi dan dipotong per panel 30 cm supaya urutan lukis benar.
// Potong poligon cembung ke persegi [x0, x1] x [y0, y1] (Sutherland-Hodgman).
function clipRect(poly, [x0, x1, y0, y1]) {
  const edges = [[(p) => p[0] >= x0, (a, b) => [x0, a[1] + (b[1] - a[1]) * (x0 - a[0]) / (b[0] - a[0])]],
    [(p) => p[0] <= x1, (a, b) => [x1, a[1] + (b[1] - a[1]) * (x1 - a[0]) / (b[0] - a[0])]],
    [(p) => p[1] >= y0, (a, b) => [a[0] + (b[0] - a[0]) * (y0 - a[1]) / (b[1] - a[1]), y0]],
    [(p) => p[1] <= y1, (a, b) => [a[0] + (b[0] - a[0]) * (y1 - a[1]) / (b[1] - a[1]), y1]]];
  let out = poly;
  for (const [inside, cross] of edges) {
    const src = out; out = [];
    src.forEach((cur, i) => {
      const prev = src[(i + src.length - 1) % src.length];
      if (inside(cur)) { if (!inside(prev)) out.push(cross(prev, cur)); out.push(cur); }
      else if (inside(prev)) out.push(cross(prev, cur));
    });
    if (!out.length) return out;
  }
  return out;
}
function wallPanels(clip = null) {
  const items = [];
  for (const zone of MAP.zona) {
    const [a, b, c, d] = zone.titik;
    const len = (p, q) => Math.hypot(q[0] - p[0], q[1] - p[1]);
    let [p0, p1, q1, q0] = len(a, b) + len(d, c) >= len(b, c) + len(a, d) ? [a, b, c, d] : [b, c, d, a];
    const n = Math.max(1, Math.round(len(p0, p1) / 30));
    const lerp = (p, q, t) => [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t];
    for (let i = 0; i < n; i++) {
      let cm = [lerp(p0, p1, i / n), lerp(p0, p1, (i + 1) / n), lerp(q0, q1, (i + 1) / n), lerp(q0, q1, i / n)];
      if (clip) { cm = clipRect(cm, clip); if (cm.length < 3) continue; }
      const ring = cm.map(([x, y]) => [x * S, y * S]);
      const cx = ring.reduce((s, p) => s + p[0], 0) / ring.length, cy = ring.reduce((s, p) => s + p[1], 0) / ring.length;
      const top = WALL_H * S;
      const mid = lerp(cm[0], cm[1], 0.5), mid2 = lerp(cm[cm.length - 1], cm[2], 0.5);
      const screws = clip ? [] : [lerp(ring[0], ring[2], 0.22), lerp(ring[0], ring[2], 0.78)];
      const svg = S_(k.extrude(ring, 0, top, P, { bevel: 0.35 * S }), { tone: "mid" })
        + (screws.length ? D(screws.map(([x, y]) => at([x, y, top])), { size: 0.55 }) : "");
      items.push({ key: depth([cx, cy, top / 2]), svg });
    }
  }
  return items;
}

// ---------------------------------------------------------------- robot
const C_ROBOT = { S, BODY_FRONT, BODY_REAR, HALF_W, WHEELBASE, TAG, TAG_TO_FRONT };
function robot(plan) {
  const r = makeRobot(k, G, P, C_ROBOT)(plan.pts[0], plan.id);
  return { key: r.key, svg: `<g data-part="robot-${plan.id}" data-key="${r.key.toFixed(2)}">${r.svg}</g>` };
}


// ---------------------------------------------------------------- Gbr 1: peta penuh
const P1 = P;
const svg1 = k.figureSvg({
  width: WIDTH,
  height: HEIGHT_VB,
  label: "Isometri peta Gudang dengan dua robot forklift. Model jauh DQfD menyusun jalur primitif demi primitif dari pose awal, lalu di pose serah model dekat memilih satu kata Reeds-Shepp sampai tujuan.",
  body: [
    floor(),
    `<g data-part="goals"></g>`,
    `<g data-part="live"></g>`,
    `<g data-part="drive"></g>`,
    `<g data-part="solids">`,
    ...[...wallPanels().map((w) => ({ ...w, svg: `<g data-key="${w.key.toFixed(2)}">${w.svg}</g>` })), ...PLANS.map(robot)].sort((a, b) => a.key - b.key).map((s) => s.svg),
    `</g>`,
  ],
});

// ---------------------------------------------------------------- Gbr 2: dekat robot ID 01
// Dilihat dari depan robot (arah hadapnya -y), supaya tiang garpu dan ruang di
// depan poros, tempat 14 aksi kandidat bercabang, tidak tertutup badan.
const CLIP = [-12, 76, 100, 178];                 // cm, dinding dipotong di sini
const PATCH = { x: -12 * S, y: 100 * S, w: 88 * S, d: 82 * S, r: 2.5 * S };
const W2 = 1040, H2 = 620;
P = k.fitProjection([...k.boxCorners(PATCH, -PLATE_T - FOOT_H, 0), W(24, 158, HEIGHT + 1)], W2, H2, { pad: 30, azimuth: -120, elevation: 36 });
const P2 = P;
function patchFloor() {
  const out = [`<path class="iso-halo" d="${k.haloOf(PATCH, -PLATE_T - FOOT_H, PLATE_T, P)}"/>`];
  const feet = k.corners(PATCH, 9 * S).sort((a, b) => depth([...a, 0]) - depth([...b, 0]));
  for (const [x, y] of feet) {
    out.push(S_(k.cylinder(x, y, 4.2 * S, -PLATE_T - FOOT_H, 0.6 * S, P, 40), { tone: "lo" }));
    out.push(S_(k.cylinder(x, y, 3.2 * S, -PLATE_T - FOOT_H + 0.6 * S, FOOT_H - 0.6 * S, P, 40), { tone: "mid" }));
  }
  out.push(S_(k.slabOf(PATCH, -PLATE_T, PLATE_T, P, 8, 1.2), { tone: "mid" }));
  const grid = [];
  for (let x = -10; x <= 70; x += 10) grid.push(k.lineOnTop([x * S, PATCH.y], [x * S, PATCH.y + PATCH.d], 0, P));
  for (let y = 110; y <= 180; y += 10) grid.push(k.lineOnTop([PATCH.x, y * S], [PATCH.x + PATCH.w, y * S], 0, P));
  out.push(L(grid.join(""), { tone: "faint" }));
  out.push(L(k.lineOnTop([PATCH.x, 148 * S], [PATCH.x + PATCH.w, 148 * S], 0, P), { tone: "lo" }));
  const screws = k.corners(PATCH, 4 * S);
  out.push(L(screws.map(([x, y]) => k.ring(x, y, 1.1 * S, 0, P, 20)).join(""), { tone: "lo" }));
  out.push(D(screws.map(([x, y]) => at([x, y, 0])), { size: 0.8 }));
  // Penggaris 1 cm / 10 cm di tepi y kecil (dekat penonton dari sisi ini).
  const sx = { x: -11 * S, y: 100.8 * S, w: 86 * S, d: 3.4 * S, r: 0.4 * S };
  out.push(S_(k.slabOf(sx, 0, 0.3 * S, P, 3), { tone: "lo", crease: "none" }));
  const tx = k.topTicks(-10 * S, 70 * S, 1 * S, 10, sx.y + sx.d - 0.1 * S, 0.3 * S, [-0.9 * S, -2.0 * S], P, "x");
  out.push(L(tx.minor, { tone: "lo" }), L(tx.major, { tone: "mid" }));
  return out.join("");
}
const svg2 = k.figureSvg({
  width: W2,
  height: H2,
  label: "Robot forklift ID 01 di pose awal S1, dilihat dari depan. Di depan poros roda depannya bercabang 14 aksi kandidat sepanjang 7,3 cm; model jauh memilih satu, menambahkannya ke jalur, lalu bercabang lagi dari ujung yang baru, sementara robot masih diam.",
  body: [
    patchFloor(),
    `<clipPath id="patch-clip"><path d="${k.onTop(k.roundedPlan(k.insetPlan(PATCH, 0.6 * S), 6), 0, P, true)}"/></clipPath>`,
    `<g data-part="live" clip-path="url(#patch-clip)"></g>`,
    ...[...wallPanels(CLIP), robot(PLANS[0])].sort((a, b) => a.key - b.key).map((s) => s.svg),
  ],
});
P = P1;

// ---------------------------------------------------------------- hidup
const SLOW1 = 0.04, SLOW2 = 0.16;   // detik tayangan per ms perencanaan
const N2 = 8;                       // primitif yang masih di dalam pelat Gbr 2
const live = `
const S = ${S};
const PLANS = ${JSON.stringify(PLANS.map((p) => ({ id: p.id, pts: p.pts.map((q) => q.map((v) => +v.toFixed(2))), serah: p.serah, prims: p.prims, msFar: p.msFar, msNear: p.msNear, msAll: p.msAll, cusps: p.cusps })))};
const ZONES = ${JSON.stringify(MAP.zona.map((z) => z.titik))};
const GEOM = { front: ${BODY_FRONT}, rear: ${BODY_REAR}, half: ${HALF_W}, prim: ${PRIM_CM}, kmax: ${KAPPA_MAX}, margin: ${MARGIN_CM} };
const LEVELS = [-3, -2, -1, 0, 1, 2, 3].map((i) => i / 3 * GEOM.kmax);
const still = matchMedia("(prefers-reduced-motion: reduce)");
const ns = "http://www.w3.org/2000/svg";
const fmt = (v, d = 1) => v.toFixed(d).replace(".", ",");

function arc(pose, kappa, dir) {
  let [x, y, t] = [pose[0], pose[1], pose[2] * Math.PI / 180];
  const out = [[x, y]], n = 10, ds = GEOM.prim / n;
  for (let i = 0; i < n; i++) { x += dir * Math.cos(t) * ds; y += dir * Math.sin(t) * ds; t += dir * kappa * ds; out.push([x, y]); }
  return { pts: out, end: [x, y, t * 180 / Math.PI] };
}
function inside(px, py, poly) {
  let sign = 0;
  for (let i = 0; i < poly.length; i++) {
    const [ax, ay] = poly[i], [bx, by] = poly[(i + 1) % poly.length];
    const c = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    if (c !== 0) { if (sign && Math.sign(c) !== sign) return false; sign = Math.sign(c); }
  }
  return true;
}
function bodyRing(pose, grow = 0) {
  const t = pose[2] * Math.PI / 180, c = Math.cos(t), s = Math.sin(t);
  return [[GEOM.front + grow, GEOM.half + grow], [-GEOM.rear - grow, GEOM.half + grow], [-GEOM.rear - grow, -GEOM.half - grow], [GEOM.front + grow, -GEOM.half - grow]]
    .map(([u, v]) => [pose[0] + u * c - v * s, pose[1] + u * s + v * c]);
}
function hits(pose) {
  const r = bodyRing(pose, GEOM.margin), samples = [];
  for (let i = 0; i < 4; i++) for (let j = 0; j < 6; j++) { const a = r[i], b = r[(i + 1) % 4]; samples.push([a[0] + (b[0] - a[0]) * j / 6, a[1] + (b[1] - a[1]) * j / 6]); }
  return ZONES.some((z) => samples.some(([x, y]) => inside(x, y, z)));
}

function mount(stage, cfg) {
  const P = cfg.P;
  const layer = stage.querySelector('[data-part="live"]');
  const readout = stage.closest(".iso-figure").querySelector("[data-readout]");
  const el = (tag, attrs) => { const e = document.createElementNS(ns, tag); for (const [a, v] of Object.entries(attrs)) e.setAttribute(a, v); layer.appendChild(e); return e; };
  const flat = (pts) => onTop(pts.map(([x, y]) => [x * S, y * S]), 0.15 * S, P);
  const ringAt = (x, y, r) => ring(x * S, y * S, r * S, 0.15 * S, P, 28);
  const plans = cfg.ids.map((id) => ({ ...PLANS.find((p) => p.id === id) }));
  for (const p of plans) {
    p.until = Math.min(p.prims, cfg.prims ?? p.prims);
    p.tFar = p.msFar * cfg.slow;
    p.tNear = Math.max(0.9, p.msNear * cfg.slow);
    p.tEnd = p.until < p.prims ? p.tFar * p.until / p.prims : p.tFar + p.tNear;
    p.el = {
      foot: el("path", { class: "iso-line", "data-tone": "mid", "data-dotted": "", d: "" }),
      far: el("path", { class: "iso-line", "data-tone": "mid", d: "", style: "stroke-width:" + cfg.width }),
      knots: el("g", { class: "iso-dots", "data-tone": "hi" }),
      fan: cfg.fan ? LEVELS.flatMap(() => [1, -1]).map(() => el("path", { class: "iso-line", "data-tone": "lo", d: "" })) : [],
      tips: el("g", { class: "iso-dots", "data-tone": "hi" }),
      near: el("path", { class: "iso-line", "data-tone": "lit", d: "", style: "stroke-width:" + cfg.width }),
      serah: el("path", { class: "iso-line", "data-tone": "hi", d: "" }),
      cusps: el("path", { class: "iso-line", "data-tone": "hi", d: "" }),
      pick: el("path", { class: "iso-line", "data-tone": "lit", d: "", style: "stroke-width:" + (cfg.fan ? cfg.width * 0.8 : cfg.width * 1.7) }),
    };
  }
  const T_END = Math.max(...plans.map((p) => p.tEnd));
  const HOLD = 3.2, FADE = 0.7, LOOP = T_END + HOLD + FADE;
  const dot = ([x, y], r) => { const [sx, sy] = iso([x * S, y * S, 0.15 * S], P); return '<circle cx="' + sx.toFixed(1) + '" cy="' + sy.toFixed(1) + '" r="' + r + '"/>'; };

  function show(tau, alpha) {
    layer.style.opacity = alpha;
    const parts = [];
    for (const p of plans) {
      const e = p.el;
      let n, share = 0, ms;
      if (tau < p.tFar) { n = Math.min(p.until, Math.floor(tau / p.tFar * p.prims)); ms = Math.min(tau, p.tEnd) / cfg.slow; }
      else { n = p.prims; share = Math.min(1, (tau - p.tFar) / p.tNear); ms = p.msFar + share * (p.msAll - p.msFar); }
      const farPhase = n < p.prims, done = share >= 1;
      e.far.setAttribute("d", n ? flat(p.pts.slice(0, 4 * n + 1)) : "");
      if (e.n !== n) { e.knots.innerHTML = p.pts.slice(0, 4 * n + 1).filter((_, i) => i % 4 === 0).map((q) => dot(q, cfg.knot)).join(""); e.n = n; }
      const pose = p.pts[Math.min(4 * n, p.serah)];
      e.foot.setAttribute("d", !done ? onTop(bodyRing(farPhase ? pose : p.pts[p.serah]).map(([x, y]) => [x * S, y * S]), 0.15 * S, P, true) : "");
      let bad = 0, f = 0;
      const tips = [];
      for (const kappa of LEVELS) for (const dir of [1, -1]) {
        const path = e.fan[f++];
        if (!path) continue;
        if (!farPhase) { path.setAttribute("d", ""); continue; }
        const a = arc(pose, kappa, dir), hit = hits(a.end);
        bad += hit;
        path.setAttribute("d", flat(a.pts));
        path.setAttribute("data-tone", hit ? "lo" : "mid");
        path.style.strokeWidth = hit ? "" : 0.8;
        if (hit) path.setAttribute("data-dotted", ""); else { path.removeAttribute("data-dotted"); tips.push(dot(a.pts[a.pts.length - 1], cfg.knot * 0.8)); }
      }
      e.tips.innerHTML = tips.join("");
      e.pick.setAttribute("d", farPhase ? flat(p.pts.slice(4 * n, 4 * n + 5)) : "");
      e.serah.setAttribute("d", farPhase ? "" : ringAt(p.pts[p.serah][0], p.pts[p.serah][1], 2.2));
      const word = p.pts.slice(p.serah);
      const upto = Math.max(1, Math.round(share * (word.length - 1)));
      e.near.setAttribute("d", farPhase ? "" : flat(word.slice(0, upto + 1)));
      e.near.setAttribute("data-tone", done ? "hi" : "lit");
      e.cusps.setAttribute("d", farPhase ? "" : p.cusps.filter((i) => i - p.serah <= upto).map((i) => ringAt(p.pts[i][0], p.pts[i][1], 1.4)).join(""));
      parts.push(cfg.say(p, { n, farPhase, done, ms, bad }));
    }
    const text = parts.join(" · ");
    if (readout.textContent !== text) readout.textContent = text;
    stage.setAttribute("aria-valuetext", text);
  }

  let tau = 0, target = null, last = 0, frame = 0, visible = false, idle = 0, touring = true;
  function tick(now) {
    const dt = last ? Math.min((now - last) / 1000, 1 / 20) : 1 / 60;
    last = now;
    if (touring) tau = (tau + dt) % LOOP;
    else tau += (target - tau) * (1 - Math.exp(-dt / 0.12));
    const alpha = touring && tau > T_END + HOLD ? Math.max(0, 1 - (tau - T_END - HOLD) / FADE) : 1;
    show(Math.min(tau, T_END), alpha);
    frame = visible && (touring || Math.abs(target - tau) > 0.002) ? requestAnimationFrame(tick) : 0;
  }
  const run = () => { if (!frame && visible) { last = 0; frame = requestAnimationFrame(tick); } };
  const steer = (to) => { if (touring) tau = Math.min(tau, T_END); touring = false; target = Math.max(0, Math.min(T_END, to)); clearTimeout(idle); idle = setTimeout(() => { touring = !still.matches; run(); }, 3600); run(); };
  new IntersectionObserver((en) => { visible = en[en.length - 1].isIntersecting; if (visible) run(); }, { rootMargin: "120px 0px" }).observe(stage);
  let locked = false;
  stage.addEventListener("pointermove", (ev) => { if (locked) return; const b = stage.getBoundingClientRect(); steer(((ev.clientX - b.left) / b.width - 0.1) / 0.8 * T_END); });
  stage.addEventListener("keydown", (ev) => {
    if (locked) return;
    const step = cfg.key * cfg.slow, base = touring ? Math.min(tau, T_END) : target;
    if (ev.key === "ArrowRight" || ev.key === "ArrowUp") steer(base + step);
    else if (ev.key === "ArrowLeft" || ev.key === "ArrowDown") steer(base - step);
    else if (ev.key === "Home") steer(0);
    else if (ev.key === "End") steer(T_END);
    else return;
    ev.preventDefault();
  });
  const q = new URLSearchParams(location.search);
  if (still.matches || q.has("t")) { touring = false; tau = target = q.has("t") ? Number(q.get("t")) * T_END : T_END; }
  show(Math.min(tau, T_END), 1);
  return {
    lock() { locked = true; touring = false; clearTimeout(idle); tau = target = T_END; show(T_END, 1); layer.style.opacity = 0.35; },
    unlock() { locked = false; touring = !still.matches; run(); },
  };
}

const stages = document.querySelectorAll(".iso-stage");
mount(stages[1], {
  P: ${JSON.stringify(P2)}, ids: [1], slow: ${SLOW2}, fan: true, prims: ${N2}, width: 2.2, knot: 1.6, key: 2,
  say: (p, s) => "primitif " + s.n + "/" + p.prims + " · " + fmt(s.ms) + " ms · " + (14 - s.bad) + " dari 14 aksi bebas zona",
});
`;

// ---------------------------------------------------------------- jalan (pure pursuit)
const strip = (src) => src.replace(/^import .*$/gm, "").replace(/^export /gm, "");
const LATHE_SRC = strip(readFileSync(join(ANATOMY, "kit/lathe.mjs"), "utf8"));
const ROBOT_SRC = strip(readFileSync(join(HERE, "robot.mjs"), "utf8"));
const PURSUIT_SRC = strip(readFileSync(join(HERE, "pursuit.mjs"), "utf8"));
const FIG1_SRC = readFileSync(join(HERE, "fig1.js"), "utf8");
// Semua run di folder ini: kunci "S1|0|dqfd-murni" (ulangan mulai 0).
const DATA = {};
for (const run of RUNS) {
  if (run.status !== "selesai") continue;
  const robots = Object.keys(run.robot).sort().map((id) => {
    const r = run.robot[id], plan = r.rencana[0];
    const pts = plan.jalur_cm.map((q) => q.map((v) => +v.toFixed(2)));
    let serah = pts.length - 1;
    for (let i = 1; i < pts.length; i++) if (Math.abs(Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) - PRIM_CM / 4) > 0.03) { serah = i - 1; break; }
    return {
      id: Number(id), pts, goal: r.tujuan, ms: plan.ms, parts: plan.rincian_ms, serah, prims: Math.floor(serah / 4),
      cusps: pts.map((p, i) => (i && p[3] !== pts[i - 1][3] ? i - 1 : -1)).filter((i) => i >= 0),
      measured: { t: r.t_tempuh_s, wait: r.t_menunggu_s, galat: r.galat_cm, n: r.n_rencana },
    };
  });
  DATA[`${run.skenario}|${run.ulang}|${run.perencana}`] = { scen: run.skenario, ulang: run.ulang, planner: run.perencana, robots };
}
const CFG = { P: P1, S, C: C_ROBOT, DATA, DEFAULT: "S1|0|dqfd-murni", SLOW: SLOW1, NEAR_MIN: 0.9 };
console.log("run tersedia:", Object.keys(DATA).length);
const MEASURED = PLANS.map((p) => { const r = UJI.robot[String(p.id)]; return { t: r.t_tempuh_s, wait: r.t_menunggu_s, galat: r.galat_cm, plans: r.n_rencana }; });
const drive = `
const G = (() => {
${LATHE_SRC}
return { frameAlong, disc, ribsOf, circleOf, dotsOf, arcOf };
})();
const RB = (() => {
${ROBOT_SRC}
return { makeRobot, makeGoal };
})();
const makeRobot = RB.makeRobot, makeGoal = RB.makeGoal;
const PP = (() => {
${PURSUIT_SRC}
return { simulateMulti, PARAM };
})();
const CFG = ${JSON.stringify(CFG)};
${FIG1_SRC}
`;

// Simulasi yang sama dijalankan sekali di sini untuk angka keterangan gambar.
const SIM = simulateMulti(PLANS.map((p) => p.pts.map((q) => q.map((v) => +v.toFixed(2)))));
const SIM_ERR = SIM.robots.map((r, i) => { const q = r.samples[r.samples.length - 1]; return Math.hypot(q[1] - PLANS[i].goal[0], q[2] - PLANS[i].goal[1]); });
console.log("sim:", SIM.robots.map((r, i) => `ID 0${i + 1} ${r.doneAt.toFixed(2)} s tunggu ${r.waited.toFixed(2)} galat ${SIM_ERR[i].toFixed(2)}`).join(" | "), "celah min", SIM.minGap.toFixed(2));

// ---------------------------------------------------------------- halaman
const [a, b] = PLANS;
const ms = (v) => v.toFixed(1).replace(".", ",");
const cm = (v) => v.toFixed(1).replace(".", ",");
const seg = (g, label, items) => `<div class="fk-seg" role="group" aria-label="${label}"><span class="fk-lab">${label}</span>${items.map(([v, t]) => `<button type="button" data-g="${g}" data-v="${v}" aria-pressed="false">${t}</button>`).join("")}</div>`;
const slider = (name, label, min, max, step, value, fmt) => `<label class="fk-param"><span class="fk-lab">${label}</span><input type="range" name="${name}" min="${min}" max="${max}" step="${step}" value="${value}" data-fmt="${fmt}"><output></output></label>`;
const controls = `<div class="fk-panel">
<div class="fk-row">${seg("scen", "Skenario", [["S1", "S1"], ["S2", "S2"]])}${seg("ulang", "Ulangan", [["0", "1"], ["1", "2"], ["2", "3"]])}${seg("planner", "Perencana", [["dqfd-murni", "DQfD murni"], ["astar", "Hybrid A*"]])}</div>
<div class="fk-row fk-play"><button type="button" class="fk-btn" data-act="start" aria-pressed="false">▶ Mulai jalan</button><button type="button" class="fk-btn" data-act="speed" aria-label="Kecepatan tayangan">1×</button><button type="button" class="fk-btn" data-act="reset">Kembali ke perencanaan</button>
<div class="fk-timeline"><input type="range" class="fk-time" min="0" max="1" step="0.05" value="0" aria-label="Waktu jalan (detik)"><svg class="fk-marks" viewBox="0 0 1000 20" preserveAspectRatio="none" aria-hidden="true"></svg></div><output class="fk-time-out">0 s</output></div>
<div class="fk-row fk-params"><span class="fk-lab fk-head">Pelacak pure pursuit</span>
${slider("lookahead", "Lookahead", 10, 50, 1, PURSUIT.lookahead, "cm")}
${slider("forwardSps", "Laju maju", 2000, 14000, 500, PURSUIT.forwardSps, "sps")}
${slider("reverseSps", "Laju mundur", 1000, 8000, 500, PURSUIT.reverseSps, "sps")}
${slider("steerDead", "Tunda kemudi", 0, 0.5, 0.01, PURSUIT.steerDead, "s")}
<button type="button" class="fk-btn fk-small" data-act="defaults">Nilai konfig.py</button></div>
<p class="fk-result" aria-live="polite"></p>
<section class="fk-detail"><h3 class="fk-detail-h"></h3><div class="fk-stats"></div><div class="fk-charts"></div><p class="fk-detail-note">Klik robot di gambar untuk memilih. Arahkan kursor ke grafik untuk membaca nilai, klik grafik untuk melompat ke waktu itu. Pita abu = robot menunggu (wasit).</p></section>
</div>`;
const plate1 = k.plateHtml({
  fig: "Gbr 1",
  title: "S1 · Gudang · dqfd-murni · ulangan 1",
  hint: "Geser kursor: waktu perencanaan · klik robot: detail",
  readout: `ID 01 ${ms(a.msAll)} ms · ID 02 ${ms(b.msAll)} ms`,
  keys: [
    { mark: "edge", label: `Jalur rencana: rantai primitif ${cm(PRIM_CM)} cm (titik di tiap sambungan); redup saat robot berjalan` },
    { mark: "lit", label: "Primitif yang sedang dipilih / kata RS dari pose serah (lingkaran); saat berjalan: lintasan yang ditempuh dan target lookahead" },
    { mark: "dotted", label: `Telapak badan di ujung jalur (margin ${MARGIN_CM} cm); di sekitar robot = robot yang dipilih` },
    { mark: "flat", label: "Bak tujuan: telapak badan di pose tujuan, panah = arah hadap" },
  ],
  caption:
    `Jalur asli dari uji robot nyata 24-10-2026 di peta Gudang (rencana pertama tiap robot; hasil.jsonl). Tayangan perencanaan ${Math.round(SLOW1 * 1000)}× lebih lambat: untuk DQfD primitif yang disimpan ditata rata sepanjang waktu model jauh (pencarian sebenarnya mencoba lebih banyak langkah, sekitar 1 ms per langkah); untuk Hybrid A* jalur muncul utuh setelah waktu A* habis, karena urutan ekspansinya tidak tercatat. ` +
    `Mulai jalan menjalankan simulasi, bukan rekaman: pelacak pure pursuit (port PathTracker dengan PARAM_PELACAK konfig.py), model aktuator simulator (tunda kemudi, τ ${ms(PURSUIT.steerTau)} s, servo ${PURSUIT.slewDegS}°/s, kemudi maks ${PURSUIT.maxSteerDeg}°, ramp 12000 sps/s²) dan versi ringkas wasit.py; faktor sempit, ikut_badan, manuver minggir, serta derau dan tunda kamera tidak dimodelkan. Angka uji nyata ada di panel detail untuk pembanding. Denah dinding dari zona Z1–Z8; tinggi dinding hanya ilustrasi.`,
  body: `<div class="iso-stage" tabindex="0" role="group" aria-label="Peta Gudang. Panah kiri/kanan menggeser waktu, spasi memulai atau menjeda jalan.">${svg1}</div>${controls}`,
});
const plate2 = k.plateHtml({
  fig: "Gbr 2",
  title: "Robot ID 01 · pose awal S1",
  hint: "Geser kursor untuk menggulir primitif",
  readout: `primitif ${N2}/${a.prims}`,
  keys: [
    { mark: "raised", label: "Robot forklift ID 01: 27 × 17,5 × 16,5 cm, poros roda 22,5 cm, roda kemudi di belakang, AprilTag 10 cm di dek atas" },
    { mark: "edge", label: `14 aksi kandidat dari ujung jalur: 7 tingkat kemudi (±41,3°) × maju/mundur, masing-masing ${cm(PRIM_CM)} cm` },
    { mark: "lit", label: "Aksi yang dipilih model jauh (Q tertinggi), lalu disambung ke jalur" },
    { mark: "dotted", label: "Telapak badan di ujung jalur; aksi yang badannya menyentuh zona digambar samar bertitik" },
  ],
  caption:
    `Dilihat dari depan robot. Robot tetap diam selama perencanaan: yang maju hanya ujung jalur. Poros roda depan adalah titik acuan pose, jadi semua aksi bercabang dari situ dan kipasnya sempit (beda arah paling besar ${(PRIM_CM * KAPPA_MAX * 180 / Math.PI).toFixed(1).replace(".", ",")}° per primitif). ` +
    `Ditampilkan ${N2} primitif pertama dari ${a.prims}, ${Math.round(SLOW2 * 1000)}× lebih lambat. Komponen di antara dek (stepper NEMA17, TMC2209, ESP32-S3, servo MG995) dari catatan firmware; letak dan baterai hanya ilustrasi.`,
  body: `<div class="iso-stage" tabindex="0" role="slider" aria-label="Primitif Gbr 2: panah kiri dan kanan menggeser 2 ms" aria-valuemin="0" aria-valuemax="${N2}">${svg2}</div>`,
});

const css = `<style>.fk-ink{fill:var(--anatomy-hi);opacity:.82;stroke:none}
.fk-panel{margin-top:14px;font-family:var(--anatomy-mono);font-size:11px;color:var(--anatomy-text);display:grid;gap:12px}
.fk-row{display:flex;flex-wrap:wrap;align-items:center;gap:8px 18px}
.fk-lab{color:var(--anatomy-muted);letter-spacing:.06em;text-transform:uppercase;font-size:10px}
.fk-seg{display:flex;align-items:center;gap:0}
.fk-seg .fk-lab{margin-right:8px}
.fk-seg button,.fk-btn{font:inherit;color:var(--anatomy-strong);background:var(--anatomy-card);border:1px solid var(--anatomy-mid);border-radius:0;padding:6px 11px;cursor:pointer;letter-spacing:.04em;min-height:30px}
.fk-seg button+button{margin-left:-1px}
.fk-seg button[aria-pressed="true"],.fk-btn[aria-pressed="true"]{background:var(--anatomy-strong);color:var(--anatomy-card);border-color:var(--anatomy-strong)}
.fk-seg button:disabled{opacity:.35;cursor:not-allowed}
.fk-seg button:hover:not(:disabled),.fk-btn:hover{border-color:var(--anatomy-hi)}
.fk-seg button:focus-visible,.fk-btn:focus-visible,.fk-panel input:focus-visible{outline:2px solid var(--anatomy-hi);outline-offset:2px}
.fk-small{padding:4px 9px;min-height:26px}
.fk-timeline{flex:1 1 260px;display:grid;gap:2px;min-width:200px}
.fk-panel input[type=range]{width:100%;accent-color:var(--anatomy-strong);margin:0}
.fk-marks{width:100%;height:20px;display:block}
.fk-mark-l{font:7px var(--anatomy-mono);fill:var(--anatomy-muted)}
.fk-mark-base{stroke:var(--anatomy-faint);stroke-width:1;vector-effect:non-scaling-stroke}
.fk-mark-wait{fill:var(--anatomy-hi);opacity:.75}
.fk-mark-cusp{fill:var(--anatomy-mid);opacity:.6}
.fk-mark-done{fill:var(--anatomy-strong)}
.fk-time-out{min-width:110px;text-align:right;color:var(--anatomy-strong)}
.fk-params{gap:8px 20px}
.fk-head{width:100%}
.fk-param{display:grid;grid-template-columns:auto;gap:3px;flex:1 1 180px;min-width:160px}
.fk-param output{color:var(--anatomy-strong)}
.fk-result{margin:0;color:var(--anatomy-strong);line-height:1.5}
.fk-detail{border-top:1px solid var(--anatomy-faint);padding-top:10px}
.fk-detail h3{margin:0 0 8px;font:inherit;font-size:11px;color:var(--anatomy-strong);letter-spacing:.06em;text-transform:uppercase}
.fk-stats{display:flex;flex-wrap:wrap;gap:6px 18px;margin-bottom:8px}
.fk-stats b{display:block;font-weight:400;color:var(--anatomy-muted);font-size:10px;letter-spacing:.05em;text-transform:uppercase}
.fk-stats span{color:var(--anatomy-strong)}
.fk-charts{display:grid;gap:6px}
.fk-ch{margin:0}
.fk-ch figcaption{display:flex;justify-content:space-between;color:var(--anatomy-muted);font-size:10px;letter-spacing:.04em}
.fk-ch-val{color:var(--anatomy-strong)}
.fk-ch svg{width:100%;height:64px;display:block;overflow:visible}
.fk-ch-line{fill:none;stroke:var(--anatomy-hi);stroke-width:2;vector-effect:non-scaling-stroke;stroke-linejoin:round}
.fk-ch-zero{stroke:var(--anatomy-faint);stroke-width:1;vector-effect:non-scaling-stroke}
.fk-ch-ax{font:9px var(--anatomy-mono);fill:var(--anatomy-muted)}
.fk-ch-wait{fill:var(--anatomy-mid);opacity:.18}
.fk-ch-cur{stroke:var(--anatomy-strong);stroke-width:1;vector-effect:non-scaling-stroke}
.fk-ch-hov{stroke:var(--anatomy-mid);stroke-width:1;stroke-dasharray:3 3;vector-effect:non-scaling-stroke}
.fk-ch-dot{fill:var(--anatomy-strong);stroke:var(--anatomy-card);stroke-width:2;vector-effect:non-scaling-stroke}
.fk-ch-hit{fill:transparent;cursor:crosshair}
.fk-detail-note{margin:8px 0 0;color:var(--anatomy-muted);line-height:1.5}
[data-part^="robot-"]{cursor:pointer}
@media (max-width:600px){.iso-plate-corner[data-corner="hint"]{display:none}.fk-time-out{min-width:0}.fk-seg .fk-lab{width:100%;margin:0 0 4px}.fk-seg{flex-wrap:wrap}}</style>`;
const name = theme === "light" ? "s1_gudang_dqfd-light.html" : "s1_gudang_dqfd.html";
writeFileSync(join(HERE, name), k.pageHtml({ title: "S1 Gudang dqfd-murni", theme, body: css + plate1 + plate2, script: kitScript() + live + drive, width: 1100 }));
console.log("wrote", name);
