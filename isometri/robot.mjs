// Pembuat robot forklift, dipakai saat build (Node) dan di halaman (browser).
// Tidak boleh memakai apa pun dari luar selain argumennya: sumbernya disalin
// apa adanya ke dalam halaman.
export function makeRobot(k, G, P, C) {
  const { S, BODY_FRONT, BODY_REAR, HALF_W, WHEELBASE, TAG, TAG_TO_FRONT } = C;
  const S_ = (paths, style) => k.solidSvg(paths, style);
  const L = (d, style) => k.lineSvg(d, style);
  const D = (points, style) => k.dotsSvg(points, style);
  const at = (p) => k.iso(p, P);
  const depth = (p) => k.depthOf(p, P);
  const local = (pose, u, v, z = 0) => {
    const t = (pose[2] * Math.PI) / 180, c = Math.cos(t), s = Math.sin(t);
    return [(pose[0] + u * c - v * s) * S, (pose[1] + u * s + v * c) * S, z * S];
  };
  return function robot(pose, id, { steer = 0, roll = 0 } = {}) {
  const th = (pose[2] * Math.PI) / 180;
  const fwd = [Math.cos(th), Math.sin(th), 0];
  const side = [-Math.sin(th), Math.cos(th), 0];
  const L3 = (u, v, z) => local(pose, u, v, z);
  const ringL = (u0, u1, v0, v1, r = 0, steps = 3) => k.roundedPlan({ x: u0, y: v0, w: u1 - u0, d: v1 - v0, r }, steps).map(([u, v]) => { const p = L3(u, v); return [p[0], p[1]]; });
  const box = (u0, u1, v0, v1, z0, h, { r = 0.3, bevel = 0, steps = 3 } = {}) => k.extrude(ringL(u0, u1, v0, v1, r, steps), z0 * S, h * S, P, { bevel: bevel * S });
  const dz = (u, v, z) => depth(L3(u, v, z));
  // Rangka roda kemudi: diputar -steer di sekitar poros belakang (kemudi belakang
  // membelok berlawanan dengan arah belok badan pada acuan poros depan).
  const sr = (-steer * Math.PI) / 180, cs = Math.cos(sr), sn = Math.sin(sr);
  const Lr = (u, v, z = 0) => { const du = u + WHEELBASE; return L3(-WHEELBASE + du * cs - v * sn, du * sn + v * cs, z); };
  const ringR = (u0, u1, v0, v1, r = 0, steps = 3) => k.roundedPlan({ x: u0, y: v0, w: u1 - u0, d: v1 - v0, r }, steps).map(([u, v]) => { const p = Lr(u, v); return [p[0], p[1]]; });
  const boxR = (u0, u1, v0, v1, z0, h, { r = 0.3 } = {}) => k.extrude(ringR(u0, u1, v0, v1, r, 3), z0 * S, h * S, P);
  const sideR = [-Math.sin(th + sr), Math.cos(th + sr), 0];
  const dots = (list, size = 0.5, tone = "mid") => D(list.map(([u, v, z]) => at(L3(u, v, z))), { size, tone });
  const topLine = (pts, z) => k.onTop(pts.map(([u, v]) => { const p = L3(u, v); return [p[0], p[1]]; }), z * S, P);

  const DECK0 = 5.5, DECK_T = 0.6, DECK1 = 15.5, TOP = DECK1 + 0.5;
  const below = [], mid = [], over = [];
  const add = (list, key, svg) => list.push({ key, svg });

  // Roda depan penggerak: jarak terluar 13 cm, lebar 2,5 cm.
  const RW = 2.75;
  for (const sgn of [1, -1]) {
    const vc = sgn * (6.5 - 1.25);
    const F = G.frameAlong(L3(0, vc, RW), side);
    let svg = S_(G.disc(-1.25 * S, 1.25 * S, RW * S, F, P), { tone: "mid" });
    svg += k.fadedSvg(G.ribsOf(F, -1.25 * S, 1.25 * S, RW * S, 30, P, { fade: [0.1, 0.55], phase: sgn * roll / RW }), { tone: "lo" });
    const out = sgn * 1.25 * S;
    svg += S_(G.disc(Math.min(out, out + sgn * 0.35 * S), Math.max(out, out + sgn * 0.35 * S), 1.3 * S, F, P), { tone: "mid" });
    svg += L(G.circleOf(F, out + sgn * 0.35 * S, 0.45 * S, P), { tone: "lo" });
    svg += D(G.dotsOf(F, out + sgn * 0.35 * S, 0.9 * S, 5, P, { all: true }), { size: 0.45 });
    add(below, dz(0, vc, RW), svg);
  }
  // Poros depan dan rumah bantalan di bawah dek.
  add(below, dz(0, 0, RW) - 0.01, S_(G.disc(-4 * S, 4 * S, 0.3 * S, G.frameAlong(L3(0, 0, RW), side), P), { tone: "mid" }));
  for (const sgn of [1, -1]) add(below, dz(0, sgn * 3, 4.2) + 0.02, S_(box(-1.2, 1.2, sgn * 2.6 - 0.6, sgn * 2.6 + 0.6, RW - 0.9, DECK0 - (RW - 0.9)), { tone: "mid" }));
  // Roda kemudi belakang dalam garpu (yoke), lurus.
  const RR = 2.0;
  const Fr = G.frameAlong(Lr(-WHEELBASE, 0, RR), sideR);
  let rear = S_(G.disc(-0.7 * S, 0.7 * S, RR * S, Fr, P), { tone: "mid" });
  rear += k.fadedSvg(G.ribsOf(Fr, -0.7 * S, 0.7 * S, RR * S, 24, P, { fade: [0.1, 0.55], phase: roll / RR }), { tone: "lo" });
  add(below, dz(-WHEELBASE, 0, RR), rear);
  for (const sgn of [1, -1]) add(below, dz(-WHEELBASE, sgn * 1.05, 2.5) + sgn * 0.01, S_(boxR(-WHEELBASE - 0.8, -WHEELBASE + 0.8, sgn * 0.85 - 0.2, sgn * 0.85 + 0.2, RR - 0.4, 4.6 - (RR - 0.4)), { tone: "mid" }));
  add(below, dz(-WHEELBASE, 0, 4.7) + 0.05, S_(boxR(-WHEELBASE - 0.8, -WHEELBASE + 0.8, -1.05, 1.05, 4.6, 0.3, { r: 0.4 }), { tone: "mid" }));
  add(below, dz(-WHEELBASE, 0, 5.2) + 0.06, S_(k.cylinder(...L3(-WHEELBASE, 0).slice(0, 2), 0.35 * S, 4.9 * S, (DECK0 - 4.9) * S, P, 16), { tone: "mid" }));

  // Dek bawah.
  const deck0 = S_(box(-BODY_REAR, 2.6, -HALF_W, HALF_W, DECK0, DECK_T, { r: 1.2, bevel: 0.25, steps: 6 }), { tone: "mid" });
  const deck0Holes = L(topLine([[-BODY_REAR, -HALF_W + 1.2], [-BODY_REAR, HALF_W - 1.2]], DECK0 + DECK_T), { tone: "faint" });

  const Z = DECK0 + DECK_T;
  // Motor stepper NEMA17 (42 mm), berbaring dengan poros ke samping.
  add(mid, dz(-2.3, -4.5, Z + 2.1), [
    S_(box(-4.4, -0.2, -6.6, -5.9, Z, 4.2, { r: 0.35 }), { tone: "mid" }),
    S_(box(-4.3, -0.3, -5.9, -3.2, Z + 0.05, 4.1, { r: 0.2 }), { tone: "hi", crease: "lo" }),
    S_(box(-4.4, -0.2, -3.2, -2.5, Z, 4.2, { r: 0.35 }), { tone: "mid" }),
    L([-5.2, -4.55, -3.9].map((v) => topLine([[-4.3, v], [-0.3, v]], Z + 4.15)).join(""), { tone: "faint" }),
    dots([[-3.9, -6.25, Z + 4.2], [-0.7, -6.25, Z + 4.2], [-3.9, -2.85, Z + 4.2], [-0.7, -2.85, Z + 4.2]], 0.45),
  ].join(""));
  // Driver TMC2209 dengan heatsink sirip.
  add(mid, dz(-8, -5, Z + 1), [
    S_(box(-9.3, -6.7, -6.3, -3.9, Z, 0.25), { tone: "mid" }),
    S_(box(-8.6, -7.4, -5.7, -4.5, Z + 0.25, 0.9, { r: 0.1 }), { tone: "hi", crease: "lo" }),
    L([-5.4, -5.1, -4.8].map((v) => topLine([[-8.6, v], [-7.4, v]], Z + 1.15)).join(""), { tone: "lo" }),
    dots([[-9.1, -6.1, Z + 0.25], [-9.1, -4.1, Z + 0.25], [-6.9, -6.1, Z + 0.25], [-6.9, -4.1, Z + 0.25]], 0.4),
  ].join(""));
  // ESP32-S3-Nano dengan dua baris header.
  add(mid, dz(-10.6, 4.2, Z + 0.6), [
    S_(box(-13.6, -7.6, 3.3, 5.1, Z, 0.35), { tone: "mid" }),
    S_(box(-12.2, -10.0, 3.7, 4.7, Z + 0.35, 0.3, { r: 0.05 }), { tone: "hi", crease: "lo" }),
    S_(box(-8.2, -7.5, 3.85, 4.55, Z + 0.35, 0.3, { r: 0.12 }), { tone: "mid" }),
    D([...Array(9)].flatMap((_, i) => [3.45, 4.95].map((v) => at(L3(-13.3 + i * 0.6, v, Z + 0.35)))), { size: 0.35, tone: "hi" }),
  ].join(""));
  // Baterai 2 x 18650 dalam dudukan.
  const BZ = Z + 0.2;
  add(mid, dz(-16.2, -1, BZ + 1), [
    S_(box(-19.8, -12.6, -3.2, 1.2, Z, 1.0, { r: 0.3 }), { tone: "mid" }),
    ...[-2.1, 0.1].map((v) => {
      const F = G.frameAlong(L3(-19.4, v, BZ + 0.95), fwd);
      return S_(G.disc(0, 6.5 * S, 0.9 * S, F, P), { tone: "hi", crease: "lo" }) + L(G.arcOf(F, 0.6 * S, 0.9 * S, P), { tone: "faint" });
    }),
  ].join(""));
  // Servo kemudi MG995 terbalik di atas roda belakang.
  add(mid, dz(-20.6, 0, Z + 2), [
    S_(box(-22.6, -18.6, -1.0, 1.0, Z, 2.6, { r: 0.2 }), { tone: "hi", crease: "lo" }),
    S_(box(-23.0, -18.2, -1.0, 1.0, Z + 2.6, 0.3, { r: 0.2 }), { tone: "mid" }),
    S_(box(-22.6, -18.6, -1.0, 1.0, Z + 2.9, 0.9, { r: 0.2 }), { tone: "hi", crease: "lo" }),
    dots([[-22.75, -0.5, Z + 2.9], [-22.75, 0.5, Z + 2.9], [-18.45, -0.5, Z + 2.9], [-18.45, 0.5, Z + 2.9]], 0.4),
  ].join(""));
  // Tiang kuningan dek atas.
  const posts = [[-21.6, 7.6], [-21.6, -7.6], [1.4, 7.6], [1.4, -7.6]];
  for (const [u, v] of posts) {
    const [x, y] = L3(u, v);
    add(mid, dz(u, v, (Z + DECK1) / 2), S_(k.cylinder(x, y, 0.42 * S, Z * S, (DECK1 - Z) * S, P, 16), { tone: "mid", crease: "none" }) + L(k.sideArc(x, y, 0.42 * S, (Z + 0.6) * S, P, 10) + k.sideArc(x, y, 0.42 * S, (DECK1 - 0.6) * S, P, 10), { tone: "faint" }));
  }

  // Dek atas, AprilTag 10 x 10 cm (pusat 9,5 cm di belakang ujung depan).
  const deck1 = [
    S_(box(-BODY_REAR, 2.6, -HALF_W, HALF_W, DECK1, 0.5, { r: 1.2, bevel: 0.25, steps: 6 }), { tone: "hi", crease: "lo" }),
    L(posts.map(([u, v]) => { const [x, y] = L3(u, v); return k.ring(x, y, 0.7 * S, (DECK1 + 0.5) * S, P, 14); }).join(""), { tone: "lo" }),
    dots(posts.map(([u, v]) => [u, v, DECK1 + 0.5]), 0.5),
  ];
  const TU = BODY_FRONT - TAG_TO_FRONT; // pusat tag (u)
  deck1.push(S_(box(TU - TAG / 2 - 0.4, TU + TAG / 2 + 0.4, -TAG / 2 - 0.4, TAG / 2 + 0.4, DECK1 + 0.5, 0.15, { r: 0.2 }), { tone: "mid", crease: "none" }));
  const cell = TAG / 10, T0 = TU - TAG / 2, V0 = -TAG / 2, TZ = (DECK1 + 0.65) * S;
  const bits = (id) => [...Array(36)].map((_, i) => (((i * 7 + id * 13) * 2654435761) >>> 0) % 5 < 2);
  const ink = [];
  const quad = (u0, v0, du, dv) => k.onTop([[u0, v0], [u0 + du, v0], [u0 + du, v0 + dv], [u0, v0 + dv]].map(([u, v]) => { const p = L3(u, v); return [p[0], p[1]]; }), TZ, P, true);
  // Bingkai hitam satu sel, lalu 6 x 6 bit data.
  for (let i = 1; i < 9; i++) for (const j of [1, 8]) { ink.push(quad(T0 + i * cell, V0 + j * cell, cell, cell)); if (i > 1 && i < 8) ink.push(quad(T0 + j * cell, V0 + i * cell, cell, cell)); }
  bits(id).forEach((on, n) => { if (on) ink.push(quad(T0 + (2 + (n % 6)) * cell, V0 + (2 + Math.floor(n / 6)) * cell, cell, cell)); });
  deck1.push(`<path class="fk-ink" d="${ink.join("")}"/>`);
  // Label dibaca dari arah penonton: pilih sumbu badan yang paling ke kanan di layar.
  const readAlong = [0, 90, 180, 270].map((d) => pose[2] + d).sort((p1, p2) => {
    const sx = (deg) => { const r = (deg * Math.PI) / 180; return at([Math.cos(r), Math.sin(r), 0])[0] - at([0, 0, 0])[0]; };
    return sx(p2) - sx(p1);
  })[0];
  const label = `<g transform="${k.topMatrix(L3(-16.8, 0).slice(0, 2), (DECK1 + 0.5) * S, P, "x")}"><g transform="rotate(${readAlong.toFixed(1)})">${k.faceTextSvg("", `ID 0${id}`, { size: 2.6 * S, tone: "lo", anchor: "middle" }).replace("<text ", '<text dy="0.35em" ')}</g></g>`;
  deck1.push(label);

  // Tiang garpu di depan: dua kanal, kereta garpu, dua garpu terlipat.
  const mast = [
    ...[1, -1].map((sgn) => S_(box(2.6, 3.3, sgn * 5.6 - (sgn > 0 ? 0 : 1.4), sgn * 5.6 + (sgn > 0 ? 1.4 : 0), 0.8, DECK1 - 0.8), { tone: "mid" })),
    S_(box(2.6, 3.3, -7.0, 7.0, DECK1 - 1.2, 1.2), { tone: "mid" }),
    S_(box(3.3, 3.6, -6.4, 6.4, 1.6, 4.4), { tone: "mid" }),
    ...[1, -1].map((sgn) => S_(box(3.6, 4.0, sgn * 3.8 - 0.6, sgn * 3.8 + 0.6, 1.2, 9.6, { r: 0.1 }), { tone: "hi", crease: "lo" })),
  ].join("");
  const mastKey = dz(3.3, 0, 8);

  const centre = dz(-9.5, 0, 8);
  const order = (list) => list.sort((a, b) => a.key - b.key).map((x) => x.svg).join("");
  const parts = [
    `<path class="iso-halo" d="${k.extrude(ringL(-BODY_REAR - 1, BODY_FRONT + 1, -HALF_W - 1, HALF_W + 1, 2, 4), 0, 0.1, P).fill}" style="filter:blur(5px)"/>`,
    mastKey < centre ? mast : "",
    order(below),
    deck0, deck0Holes,
    order(mid),
    deck1.join(""),
    mastKey >= centre ? mast : "",
  ];
  return { key: centre, svg: parts.join("") };
  };
}

// Bak tujuan: telapak badan di pose tujuan, dicat di lantai, panah = arah hadap.
export function makeGoal(k, P, C) {
  const { S, BODY_FRONT, BODY_REAR, HALF_W } = C;
  const local = (pose, u, v, z = 0) => {
    const t = (pose[2] * Math.PI) / 180, c = Math.cos(t), s = Math.sin(t);
    return [(pose[0] + u * c - v * s) * S, (pose[1] + u * s + v * c) * S, z * S];
  };
  const foot = (g, grow) => [[BODY_FRONT + grow, HALF_W + grow], [-BODY_REAR - grow, HALF_W + grow], [-BODY_REAR - grow, -HALF_W - grow], [BODY_FRONT + grow, -HALF_W - grow]].map(([u, v]) => local(g, u, v).slice(0, 2));
  return function goal(g) {
    const zf = 0.12 * S;
    const flatPts = (list) => list.map(([u, v]) => local(g, u, v).slice(0, 2));
    return [
      k.solidSvg(k.extrude(foot(g, 1.2), 0, zf, P), { tone: "mid", crease: "none" }),
      k.lineSvg(k.onTop(foot(g, -0.6), zf, P, true), { tone: "faint" }),
      k.lineSvg(k.onTop(flatPts([[-BODY_REAR + 0.6, 0], [BODY_FRONT - 4.6, 0]]), zf, P), { tone: "mid" }),
      k.lineSvg(k.onTop(flatPts([[BODY_FRONT - 0.6, 0], [BODY_FRONT - 4.6, 2.6], [BODY_FRONT - 4.6, -2.6]]), zf, P, true), { tone: "mid" }),
      k.dotsSvg([k.iso(local(g, 0, 0, 0.12), P)], { size: 1.1, tone: "hi" }),
    ].join("");
  };
}
