// Gbr 1 interaktif: pilih skenario/ulangan/perencana, tayang ulang perencanaan,
// jalankan simulasi pure pursuit, gulir waktu, ubah parameter pelacak, klik robot.
// Disalin ke halaman oleh build.mjs; memakai global dari kit (iso, onTop, ring,
// solidSvg, ...), G, makeRobot, makeGoal, PP, dan CFG yang disisipkan build.
(() => {
  const { P, S, C, DATA, DEFAULT, SLOW, NEAR_MIN } = CFG;
  const kNS = { solidSvg, lineSvg, dotsSvg, iso, depthOf, roundedPlan, extrude, cylinder, sideArc, ring, onTop, fadedSvg, faceTextSvg, topMatrix };
  const buildRobot = makeRobot(kNS, G, P, C);
  const buildGoal = makeGoal(kNS, P, C);
  const stage = document.querySelectorAll(".iso-stage")[0];
  const fig = stage.closest(".iso-figure");
  const svg = stage.querySelector("svg");
  const $ = (sel) => fig.querySelector(sel);
  const $$ = (sel) => [...fig.querySelectorAll(sel)];
  const goalsG = svg.querySelector('[data-part="goals"]');
  const planG = svg.querySelector('[data-part="live"]');
  const driveG = svg.querySelector('[data-part="drive"]');
  const solidsG = svg.querySelector('[data-part="solids"]');
  const readout = fig.querySelector("[data-readout]");
  const robotsEl = [1, 2].map((id) => solidsG.querySelector('[data-part="robot-' + id + '"]'));
  const still = matchMedia("(prefers-reduced-motion: reduce)");
  const ns = "http://www.w3.org/2000/svg";
  const z = 0.15 * S;
  const fmt = (v, d = 1) => (v ?? 0).toFixed(d).replace(".", ",").replace("-", "−");
  const mk = (parent, tag, attrs) => { const e = document.createElementNS(ns, tag); for (const [a, v] of Object.entries(attrs)) e.setAttribute(a, v); parent.appendChild(e); return e; };
  const flat = (pts) => onTop(pts.map(([x, y]) => [x * S, y * S]), z, P);
  const ringAt = (x, y, r) => ring(x * S, y * S, r * S, z, P, 28);
  const scr = (x, y) => iso([x * S, y * S, z], P);
  const bodyRing = (pose, grow = 0) => {
    const t = (pose[2] * Math.PI) / 180, c = Math.cos(t), s = Math.sin(t);
    return [[C.BODY_FRONT + grow, C.HALF_W + grow], [-C.BODY_REAR - grow, C.HALF_W + grow], [-C.BODY_REAR - grow, -C.HALF_W - grow], [C.BODY_FRONT + grow, -C.HALF_W - grow]]
      .map(([u, v]) => [(pose[0] + u * c - v * s) * S, (pose[1] + u * s + v * c) * S]);
  };

  // ------------------------------------------------------------ keadaan
  let cur = null, sim = null, END = 1, key = DEFAULT;
  let mode = "plan", tau = 0, touring = true, idle = 0, t = 0, playing = false, rate = 1, sel = null;
  const params = { lookahead: PP.PARAM.lookahead, forwardSps: PP.PARAM.forwardSps, reverseSps: PP.PARAM.reverseSps, steerDead: PP.PARAM.steerDead };
  const DEFAULTS = { ...params };

  // ------------------------------------------------------------ lapisan rencana
  const planEl = [0, 1].map(() => ({
    foot: mk(planG, "path", { class: "iso-line", "data-tone": "mid", "data-dotted": "", d: "" }),
    far: mk(planG, "path", { class: "iso-line", "data-tone": "mid", d: "", style: "stroke-width:1.4" }),
    knots: mk(planG, "g", { class: "iso-dots", "data-tone": "hi" }),
    near: mk(planG, "path", { class: "iso-line", "data-tone": "lit", d: "", style: "stroke-width:1.4" }),
    serah: mk(planG, "path", { class: "iso-line", "data-tone": "hi", d: "" }),
    cusps: mk(planG, "path", { class: "iso-line", "data-tone": "hi", d: "" }),
    pick: mk(planG, "path", { class: "iso-line", "data-tone": "lit", d: "", style: "stroke-width:2.4" }),
  }));
  const selRing = mk(driveG, "path", { class: "iso-line", "data-tone": "lit", "data-dotted": "", d: "" });
  const trail = [0, 1].map(() => mk(driveG, "path", { class: "iso-line", "data-tone": "lit", d: "", style: "stroke-width:1.7" }));
  const aim = [0, 1].map(() => mk(driveG, "path", { class: "iso-line", "data-tone": "lit", d: "" }));
  const aimDot = [0, 1].map(() => mk(driveG, "circle", { class: "iso-wire-end", r: "1.6", cx: "-99", cy: "-99", style: "stroke:var(--anatomy-lit)" }));
  const dot = ([x, y], r) => { const [sx, sy] = scr(x, y); return '<circle cx="' + sx.toFixed(1) + '" cy="' + sy.toFixed(1) + '" r="' + r + '"/>'; };

  const isDqfd = () => cur.planner === "dqfd-murni";
  function timings(r) {
    const far = isDqfd() ? r.parts.dqfd_jauh : r.parts.astar;
    const near = isDqfd() ? r.parts.dqfd_dekat : r.parts.rs;
    r.tFar = far * SLOW; r.tNear = Math.max(NEAR_MIN, near * SLOW); r.tEnd = r.tFar + r.tNear;
  }
  const PLAN_END = () => Math.max(...cur.robots.map((r) => r.tEnd));
  const HOLD = 3.2, FADE = 0.7;

  function showPlan(tt, alpha = 1) {
    planG.style.opacity = mode === "drive" ? 0.35 : alpha;
    const parts = [];
    cur.robots.forEach((r, i) => {
      const e = planEl[i];
      let n, share = 0, ms;
      if (tt < r.tFar) { n = isDqfd() ? Math.floor((tt / r.tFar) * r.prims) : 0; ms = tt / SLOW; }
      else { n = r.prims; share = Math.min(1, (tt - r.tFar) / r.tNear); ms = r.far + share * (r.ms - r.far); }
      const farPhase = tt < r.tFar, done = share >= 1;
      e.far.setAttribute("d", n ? flat(r.pts.slice(0, 4 * n + 1)) : "");
      if (e.n !== n || e.key !== key) { e.knots.innerHTML = r.pts.slice(0, 4 * n + 1).filter((_, j) => j % 4 === 0).map((q) => dot(q, 0.9)).join(""); e.n = n; e.key = key; }
      const pose = r.pts[Math.min(4 * n, r.serah)];
      e.foot.setAttribute("d", !done && mode === "plan" ? onTop(bodyRing(farPhase ? pose : r.pts[r.serah]), z, P, true) : "");
      e.pick.setAttribute("d", farPhase && isDqfd() ? flat(r.pts.slice(4 * n, 4 * n + 5)) : "");
      e.serah.setAttribute("d", farPhase ? "" : ringAt(r.pts[r.serah][0], r.pts[r.serah][1], 2.2));
      const word = r.pts.slice(r.serah);
      const upto = Math.max(1, Math.round(share * (word.length - 1)));
      e.near.setAttribute("d", farPhase ? "" : flat(word.slice(0, upto + 1)));
      e.near.setAttribute("data-tone", done ? "hi" : "lit");
      e.cusps.setAttribute("d", farPhase ? "" : r.cusps.filter((j) => j - r.serah <= upto).map((j) => ringAt(r.pts[j][0], r.pts[j][1], 1.4)).join(""));
      const name = "ID 0" + r.id + " ";
      if (done) parts.push(name + fmt(r.ms) + " ms");
      else if (farPhase) parts.push(name + (isDqfd() ? "jauh " + n + "/" + r.prims : "A* mencari") + " " + fmt(ms, 0) + " ms");
      else parts.push(name + (isDqfd() ? "dekat" : "RS") + " " + fmt(ms, 0) + " ms");
    });
    if (mode === "plan") say(parts.join(" · "));
  }
  function say(text) { if (readout.textContent !== text) readout.textContent = text; stage.setAttribute("aria-valuetext", text); }

  // ------------------------------------------------------------ robot
  function place(i, pose, opts) {
    const el = robotsEl[i];
    const r = buildRobot(pose, cur.robots[i].id, opts);
    el.innerHTML = r.svg;
    el.dataset.key = r.key.toFixed(2);
    el.remove();
    const next = [...solidsG.children].find((c) => Number(c.dataset.key) > r.key);
    solidsG.insertBefore(el, next || null);
  }
  function sampleAt(r, tt) {
    const s = r.samples;
    if (tt >= s[s.length - 1][0]) return s[s.length - 1];
    let lo = 0, hi = s.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (s[m][0] <= tt) lo = m; else hi = m; }
    const a = s[lo], b = s[hi], f = (tt - a[0]) / (b[0] - a[0] || 1);
    const out = a.map((v, j) => (v === null || b[j] === null ? v : v + (b[j] - v) * f));
    out[3] = a[3] + ((((b[3] - a[3]) % 360) + 540) % 360 - 180) * f;
    out[10] = a[10]; out[11] = a[11];
    return out;
  }
  const poseNow = (i) => {
    if (mode === "drive") { const s = sampleAt(sim.robots[i], t); return [s[1], s[2], s[3]]; }
    return cur.robots[i].pts[0];
  };
  function drawSel() {
    selRing.setAttribute("d", sel === null ? "" : onTop(bodyRing(poseNow(sel), 2.2), z, P, true));
    robotsEl.forEach((el, i) => el.toggleAttribute("data-sel", sel === i));
  }

  // ------------------------------------------------------------ jalan
  function drawDrive() {
    const parts = [];
    sim.robots.forEach((r, i) => {
      const [, x, y, yaw, steer, v, tx, ty, dist, , paused, waiting] = sampleAt(r, t);
      place(i, [x, y, yaw], { steer, roll: dist * S });
      const upto = [];
      for (const q of r.samples) { if (q[0] > t) break; upto.push([q[1] * S, q[2] * S]); }
      upto.push([x * S, y * S]);
      trail[i].setAttribute("d", onTop(upto, z, P));
      const done = r.doneAt !== null && t >= r.doneAt;
      if (tx !== null && !done) {
        const [ax, ay] = scr(x, y), [bx, by] = scr(tx, ty);
        aim[i].setAttribute("d", "M" + ax.toFixed(1) + " " + ay.toFixed(1) + "L" + bx.toFixed(1) + " " + by.toFixed(1));
        aimDot[i].setAttribute("cx", bx.toFixed(1)); aimDot[i].setAttribute("cy", by.toFixed(1));
      } else { aim[i].setAttribute("d", ""); aimDot[i].setAttribute("cx", "-99"); }
      const g = cur.robots[i].goal, name = "ID 0" + cur.robots[i].id + " ";
      if (done) parts.push(name + "tiba " + fmt(r.doneAt) + " s, galat " + fmt(Math.hypot(x - g[0], y - g[1])) + " cm");
      else if (waiting >= 0.5) parts.push(name + "menunggu");
      else if (paused >= 0.5) parts.push(name + "jeda ganti arah");
      else parts.push(name + fmt(v, 0) + " cm/s δ " + fmt(steer, 0) + "°");
    });
    say((t >= END ? "selesai · " : "t " + fmt(t) + " s · ") + parts.join(" · "));
    timeEl.value = t;
    timeOut.textContent = fmt(t) + " / " + fmt(END) + " s";
    drawSel();
    cursorCharts();
  }
  function clearDrive() {
    [...trail, ...aim].forEach((e) => e.setAttribute("d", ""));
    aimDot.forEach((e) => e.setAttribute("cx", "-99"));
  }
  function enterDrive() {
    if (mode === "drive") return;
    mode = "drive"; touring = false; clearTimeout(idle);
    tau = PLAN_END(); showPlan(tau);
  }
  function toPlan() {
    mode = "plan"; playing = false; t = 0;
    clearDrive();
    cur.robots.forEach((r, i) => place(i, r.pts[0], {}));
    timeEl.value = 0; timeOut.textContent = fmt(0) + " / " + fmt(END) + " s";
    tau = 0; touring = !still.matches;
    showPlan(still.matches ? PLAN_END() : 0);
    drawSel(); cursorCharts(); label(); run();
  }

  // ------------------------------------------------------------ simulasi
  function resim() {
    sim = PP.simulateMulti(cur.robots.map((r) => r.pts), { ...PP.PARAM, ...params });
    END = Math.max(...sim.robots.map((r) => r.samples[r.samples.length - 1][0]));
    t = Math.min(t, END);
    timeEl.max = END.toFixed(2);
    drawMarkers();
    const res = sim.robots.map((r, i) => {
      const q = r.samples[r.samples.length - 1], g = cur.robots[i].goal;
      return "ID 0" + cur.robots[i].id + " tiba " + fmt(r.doneAt) + " s" + (r.waited > 0.05 ? " (menunggu " + fmt(r.waited) + " s)" : "") + ", galat " + fmt(Math.hypot(q[1] - g[0], q[2] - g[1])) + " cm";
    });
    $(".fk-result").textContent = "Simulasi: " + res.join(" · ") + " · celah badan min " + fmt(sim.minGap) + " cm" + (sim.minGap <= 0 ? " — BERSENTUHAN" : "");
    drawCharts();
    if (mode === "drive") drawDrive();
  }

  // ------------------------------------------------------------ bilah waktu
  const timeEl = $(".fk-time"), timeOut = $(".fk-time-out"), marks = $(".fk-marks");
  function drawMarkers() {
    // Dua baris: ID 01 dan ID 02. Pekat = menunggu (wasit), samar = jeda ganti arah.
    const W = 1000, rows = [];
    sim.robots.forEach((r, i) => {
      const y = 2 + i * 9;
      rows.push('<text x="0" y="' + (y + 6) + '" class="fk-mark-l">0' + cur.robots[i].id + '</text>');
      rows.push('<line x1="28" x2="' + W + '" y1="' + (y + 3.5) + '" y2="' + (y + 3.5) + '" class="fk-mark-base"/>');
      for (const [col, cls] of [[11, "fk-mark-wait"], [10, "fk-mark-cusp"]]) {
        let start = null;
        r.samples.forEach((q, j) => {
          const on = q[col] >= 0.5, last = j === r.samples.length - 1;
          if (on && start === null) start = q[0];
          if ((!on || last) && start !== null) {
            const x0 = 28 + (start / END) * (W - 28), x1 = 28 + (q[0] / END) * (W - 28);
            rows.push('<rect x="' + x0.toFixed(1) + '" y="' + y + '" width="' + Math.max(2, x1 - x0).toFixed(1) + '" height="7" rx="1.5" class="' + cls + '"><title>ID 0' + cur.robots[i].id + (cls === "fk-mark-wait" ? " menunggu " : " jeda ganti arah ") + fmt(start) + "–" + fmt(q[0]) + ' s</title></rect>');
            start = null;
          }
        });
      }
      if (r.doneAt !== null) { const x = 28 + (r.doneAt / END) * (W - 28); rows.push('<rect x="' + (x - 1).toFixed(1) + '" y="' + y + '" width="2" height="7" class="fk-mark-done"><title>ID 0' + cur.robots[i].id + " tiba " + fmt(r.doneAt) + ' s</title></rect>'); }
    });
    marks.innerHTML = rows.join("");
  }
  timeEl.addEventListener("input", () => { enterDrive(); playing = false; t = Number(timeEl.value); label(); drawDrive(); });

  // ------------------------------------------------------------ panel detail
  const panel = $(".fk-detail");
  const CH = [
    { key: 5, name: "Kecepatan", unit: "cm/s", d: 0 },
    { key: 4, name: "Sudut kemudi", unit: "°", d: 0 },
    { key: 9, name: "Simpangan dari jalur", unit: "cm", d: 1 },
  ];
  const CW = 600, CHH = 64, PADL = 34, PADR = 8, PADT = 6, PADB = 14;
  const xs = (tt) => PADL + (tt / END) * (CW - PADL - PADR);
  function drawCharts() {
    const i = sel ?? 0;
    const r = sim.robots[i], meta = cur.robots[i];
    const q = r.samples[r.samples.length - 1];
    const maxDev = Math.max(...r.samples.map((s) => s[9] || 0));
    const dist = q[8];
    panel.querySelector(".fk-detail-h").textContent = "Robot ID 0" + meta.id + (sel === null ? " (klik robot lain di gambar untuk berganti)" : "");
    panel.querySelector(".fk-stats").innerHTML = [
      ["Tiba (sim)", fmt(r.doneAt) + " s"], ["Menunggu", fmt(r.waited) + " s"], ["Jarak tempuh", fmt(dist, 0) + " cm"],
      ["Simpangan maks", fmt(maxDev) + " cm"], ["Galat akhir", fmt(Math.hypot(q[1] - meta.goal[0], q[2] - meta.goal[1])) + " cm"],
      ["Waktu rencana", fmt(meta.ms) + " ms"], ["Uji nyata", fmt(meta.measured.t) + " s · galat " + fmt(meta.measured.galat) + " cm" + (meta.measured.n > 1 ? " · " + meta.measured.n + " rencana" : "")],
    ].map(([a, b]) => "<span><b>" + a + "</b>" + b + "</span>").join("");
    panel.querySelector(".fk-charts").innerHTML = CH.map((c, ci) => {
      const vals = r.samples.map((s) => s[c.key] || 0);
      let lo = Math.min(0, ...vals), hi = Math.max(0, ...vals);
      if (hi - lo < 1e-6) hi = lo + 1;
      const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
      const ys = (v) => PADT + (1 - (v - lo) / (hi - lo)) * (CHH - PADT - PADB);
      const path = r.samples.map((s, j) => (j ? "L" : "M") + xs(s[0]).toFixed(1) + " " + ys(s[c.key] || 0).toFixed(1)).join("");
      const waits = [];
      let st = null;
      r.samples.forEach((s, j) => { const on = s[11] >= 0.5; if (on && st === null) st = s[0]; if ((!on || j === r.samples.length - 1) && st !== null) { waits.push('<rect x="' + xs(st).toFixed(1) + '" y="' + PADT + '" width="' + Math.max(1, xs(s[0]) - xs(st)).toFixed(1) + '" height="' + (CHH - PADT - PADB) + '" class="fk-ch-wait"/>'); st = null; } });
      const ticks = [];
      for (let k = 0; k <= END; k += END > 30 ? 10 : 5) ticks.push('<text x="' + xs(k).toFixed(1) + '" y="' + (CHH - 2) + '" class="fk-ch-ax" text-anchor="middle">' + k + "</text>");
      return '<figure class="fk-ch" data-ch="' + ci + '"><figcaption><span>' + c.name + ' (' + c.unit + ')</span><output class="fk-ch-val"></output></figcaption>' +
        '<svg viewBox="0 0 ' + CW + " " + CHH + '" preserveAspectRatio="none" role="img" aria-label="' + c.name + ' terhadap waktu, ID 0' + meta.id + '">' +
        waits.join("") +
        '<line x1="' + PADL + '" x2="' + (CW - PADR) + '" y1="' + ys(0).toFixed(1) + '" y2="' + ys(0).toFixed(1) + '" class="fk-ch-zero"/>' +
        '<text x="' + (PADL - 4) + '" y="' + (ys(hi - pad) + 3).toFixed(1) + '" class="fk-ch-ax" text-anchor="end">' + fmt(hi - pad, 0) + "</text>" +
        '<text x="' + (PADL - 4) + '" y="' + (ys(lo + pad) + 3).toFixed(1) + '" class="fk-ch-ax" text-anchor="end">' + fmt(lo + pad, 0) + "</text>" +
        ticks.join("") +
        '<path d="' + path + '" class="fk-ch-line"/>' +
        '<line class="fk-ch-cur" y1="' + PADT + '" y2="' + (CHH - PADB) + '" x1="-9" x2="-9"/>' +
        '<line class="fk-ch-hov" y1="' + PADT + '" y2="' + (CHH - PADB) + '" x1="0" x2="0" display="none"/>' +
        '<circle class="fk-ch-dot" r="3" cx="0" cy="0" display="none"/>' +
        '<rect class="fk-ch-hit" x="' + PADL + '" y="0" width="' + (CW - PADL - PADR) + '" height="' + CHH + '"/></svg></figure>';
    }).join("");
    panel.querySelectorAll(".fk-ch").forEach((f) => {
      const ci = Number(f.dataset.ch), c = CH[ci], s2 = f.querySelector("svg");
      const tAt = (ev) => { const b = s2.getBoundingClientRect(); return Math.max(0, Math.min(END, (((ev.clientX - b.left) / b.width) * CW - PADL) / (CW - PADL - PADR) * END)); };
      s2.addEventListener("pointermove", (ev) => hoverChart(ci, tAt(ev)));
      s2.addEventListener("pointerleave", () => hoverChart(ci, null));
      s2.addEventListener("click", (ev) => { enterDrive(); playing = false; t = tAt(ev); label(); drawDrive(); });
    });
    cursorCharts();
  }
  function valueAt(ci, tt) { return sampleAt(sim.robots[sel ?? 0], tt)[CH[ci].key] || 0; }
  function hoverChart(ci, tt) {
    const f = panel.querySelector('.fk-ch[data-ch="' + ci + '"]');
    const hov = f.querySelector(".fk-ch-hov"), dotEl = f.querySelector(".fk-ch-dot"), out = f.querySelector(".fk-ch-val");
    if (tt === null) { hov.setAttribute("display", "none"); dotEl.setAttribute("display", "none"); cursorCharts(); return; }
    hov.removeAttribute("display"); dotEl.removeAttribute("display");
    const path = f.querySelector(".fk-ch-line"), x = xs(tt);
    hov.setAttribute("x1", x); hov.setAttribute("x2", x);
    // Nilai y dari path: cari titik dengan x terdekat.
    const len = path.getTotalLength(); let a = 0, b = len;
    for (let k = 0; k < 22; k++) { const m = (a + b) / 2; if (path.getPointAtLength(m).x < x) a = m; else b = m; }
    const p = path.getPointAtLength(a);
    dotEl.setAttribute("cx", p.x); dotEl.setAttribute("cy", p.y);
    out.textContent = "t " + fmt(tt) + " s: " + fmt(valueAt(ci, tt), CH[ci].d + 1) + " " + CH[ci].unit;
  }
  function cursorCharts() {
    const tt = mode === "drive" ? t : 0;
    panel.querySelectorAll(".fk-ch").forEach((f) => {
      const ci = Number(f.dataset.ch), x = xs(tt), cl = f.querySelector(".fk-ch-cur");
      cl.setAttribute("x1", x); cl.setAttribute("x2", x);
      if (f.querySelector(".fk-ch-hov").getAttribute("display") === "none") f.querySelector(".fk-ch-val").textContent = (mode === "drive" ? "t " + fmt(tt) + " s: " : "") + fmt(valueAt(ci, tt), CH[ci].d + 1) + " " + CH[ci].unit;
    });
  }

  // ------------------------------------------------------------ pilihan skenario
  function load(next) {
    key = next; cur = DATA[key];
    cur.robots.forEach((r) => { r.far = isDqfd() ? r.parts.dqfd_jauh : r.parts.astar; timings(r); });
    goalsG.innerHTML = cur.robots.map((r) => buildGoal(r.goal)).join("");
    const title = fig.querySelector('[data-corner="title"]');
    if (title) title.textContent = cur.scen + " · Gudang · " + (isDqfd() ? "dqfd-murni" : "Hybrid A*") + " · ulangan " + (cur.ulang + 1);
    planEl.forEach((e) => { e.n = -1; });
    $$(".fk-seg button").forEach((b) => {
      const [g, v] = [b.dataset.g, b.dataset.v];
      const parts = key.split("|"), idx = { scen: 0, ulang: 1, planner: 2 }[g];
      b.setAttribute("aria-pressed", String(parts[idx] === v));
      const trial = [...parts]; trial[idx] = v;
      b.disabled = !DATA[trial.join("|")];
    });
    t = 0;
    resim();
    toPlan();
  }
  $$(".fk-seg button").forEach((b) => b.addEventListener("click", () => {
    const parts = key.split("|"), idx = { scen: 0, ulang: 1, planner: 2 }[b.dataset.g];
    parts[idx] = b.dataset.v;
    if (DATA[parts.join("|")]) load(parts.join("|"));
  }));

  // ------------------------------------------------------------ parameter
  let simTimer = 0;
  $$(".fk-param input").forEach((inp) => {
    const out = inp.parentElement.querySelector("output");
    const show = () => { out.textContent = inp.dataset.fmt === "s" ? fmt(Number(inp.value), 2) + " s" : inp.dataset.fmt === "sps" ? inp.value + " sps · " + fmt(Number(inp.value) * PP.PARAM.cmPerStep) + " cm/s" : inp.value + " cm"; };
    show();
    inp.addEventListener("input", () => {
      params[inp.name] = Number(inp.value); show();
      clearTimeout(simTimer); simTimer = setTimeout(resim, 60);
    });
  });
  $('[data-act="defaults"]').addEventListener("click", () => {
    Object.assign(params, DEFAULTS);
    $$(".fk-param input").forEach((inp) => { inp.value = params[inp.name]; inp.dispatchEvent(new Event("input")); });
  });

  // ------------------------------------------------------------ tombol jalan
  const btn = $('[data-act="start"]'), spd = $('[data-act="speed"]');
  function label() {
    btn.textContent = playing ? "❚❚ Jeda" : mode === "drive" && t >= END ? "↺ Ulang dari awal" : mode === "drive" && t > 0 ? "▶ Lanjut" : "▶ Mulai jalan";
    btn.setAttribute("aria-pressed", String(playing));
  }
  btn.addEventListener("click", () => {
    if (playing) playing = false;
    else { enterDrive(); if (t >= END) t = 0; playing = true; }
    label(); drawDrive(); run();
  });
  $('[data-act="reset"]').addEventListener("click", () => toPlan());
  spd.addEventListener("click", () => { rate = rate === 1 ? 2 : rate === 2 ? 4 : rate === 4 ? 0.5 : 1; spd.textContent = (rate === 0.5 ? "½" : rate) + "×"; });

  // ------------------------------------------------------------ klik robot
  robotsEl.forEach((el, i) => {
    el.addEventListener("click", (ev) => { ev.stopPropagation(); sel = sel === i ? null : i; drawSel(); drawCharts(); });
  });

  // ------------------------------------------------------------ loop
  let frame = 0, last = 0, visible = false;
  function tick(now) {
    const dt = last ? Math.min((now - last) / 1000, 0.1) : 0;
    last = now;
    if (mode === "plan" && touring) {
      const LOOP = PLAN_END() + HOLD + FADE;
      tau = (tau + dt) % LOOP;
      const alpha = tau > PLAN_END() + HOLD ? Math.max(0, 1 - (tau - PLAN_END() - HOLD) / FADE) : 1;
      showPlan(Math.min(tau, PLAN_END()), alpha);
    }
    if (mode === "drive" && playing) {
      t = Math.min(END, t + dt * rate);
      if (t >= END) { playing = false; label(); }
      drawDrive();
    }
    frame = visible && ((mode === "plan" && touring) || playing) ? requestAnimationFrame(tick) : 0;
  }
  function run() { if (!frame && visible) { last = 0; frame = requestAnimationFrame(tick); } }
  new IntersectionObserver((en) => { visible = en[en.length - 1].isIntersecting; if (visible) run(); }, { rootMargin: "120px 0px" }).observe(stage);

  // Geser kursor di gambar menggulir waktu perencanaan (hanya saat mode rencana).
  stage.addEventListener("pointermove", (ev) => {
    if (mode !== "plan" || ev.target.closest("[data-part^='robot-']")) return;
    const b = stage.getBoundingClientRect();
    touring = false; clearTimeout(idle);
    tau = Math.max(0, Math.min(PLAN_END(), (((ev.clientX - b.left) / b.width - 0.1) / 0.8) * PLAN_END()));
    showPlan(tau);
    idle = setTimeout(() => { if (mode === "plan") { touring = !still.matches; run(); } }, 3600);
  });
  stage.addEventListener("keydown", (ev) => {
    if (ev.key === " ") { ev.preventDefault(); btn.click(); return; }
    const dir = ev.key === "ArrowRight" || ev.key === "ArrowUp" ? 1 : ev.key === "ArrowLeft" || ev.key === "ArrowDown" ? -1 : 0;
    if (!dir) return;
    ev.preventDefault();
    if (mode === "drive") { playing = false; t = Math.max(0, Math.min(END, t + dir * 0.5)); label(); drawDrive(); }
    else { touring = false; tau = Math.max(0, Math.min(PLAN_END(), tau + dir * 5 * SLOW)); showPlan(tau); }
  });

  const q = new URLSearchParams(location.search);
  load(q.get("k") && DATA[q.get("k")] ? q.get("k") : DEFAULT);
  if (q.has("drive")) { enterDrive(); t = Number(q.get("drive")) * END; label(); drawDrive(); }
  if (q.has("sel")) { sel = Number(q.get("sel")); drawSel(); drawCharts(); }
})();
