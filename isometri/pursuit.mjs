// Pelacak pure pursuit: port JavaScript dari PathTracker (forklift_planner.py)
// dengan parameter PARAM_PELACAK["pursuit"] di konfig.py, ditambah model
// aktuator simulator yang sama (tunda kemudi, orde-1, slew servo, ramp stepper).
// Faktor "sempit" (perlambatan dekat zona, jejak.py) tidak ikut dimodelkan.

export const PARAM = {
  lookahead: 26.0, segEndTol: 4.1, slowdownDist: 32.4, finalTurnDist: 35.0,
  forwardSps: 10000, reverseSps: 4000, minSpeedRatio: 0.5, curveSpeedRatio: 0.7, finalTurnRatio: 0.3,
  cuspPause: 0.6, cmPerStep: 0.00249, accelSps2: 12000,
  L: 22.5, maxSteerDeg: 55.0, planSteerRatio: 0.75,
  steerDead: 0.19, steerTau: 0.10, slewDegS: 500,
  control: 0.05, substep: 0.005,
};

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

function splitSegments(path) {
  const segs = [];
  let cur = [], d = null;
  for (const p of path) {
    if (d !== null && p[3] !== d) { segs.push({ pts: cur, d }); cur = [cur[cur.length - 1]]; }
    cur.push(p); d = p[3];
  }
  if (cur.length > 1) segs.push({ pts: cur, d });
  return segs;
}

function circleHit(x1, y1, x2, y2, px, py, r) {
  const dx = x2 - x1, dy = y2 - y1, fx = x1 - px, fy = y1 - py;
  const a = dx * dx + dy * dy;
  if (!a) return null;
  const b = 2 * (fx * dx + fy * dy), c = fx * fx + fy * fy - r * r, disc = b * b - 4 * a * c;
  if (disc < 0) return null;
  const t = (-b + Math.sqrt(disc)) / (2 * a);
  return t >= 0 && t <= 1 ? [x1 + t * dx, y1 + t * dy] : null;
}

export function tracker(pathDeg, Pm = PARAM) {
  const path = pathDeg.map(([x, y, yaw, d]) => [x, y, (yaw * Math.PI) / 180, d]);
  const segs = splitSegments(path);
  const maxSteer = (Pm.maxSteerDeg * Math.PI) / 180;
  const kRef = Math.tan(maxSteer * Pm.planSteerRatio) / Pm.L;
  for (const s of segs) {
    s.cum = [0];
    for (let i = 1; i < s.pts.length; i++) s.cum.push(s.cum[i - 1] + Math.hypot(s.pts[i][0] - s.pts[i - 1][0], s.pts[i][1] - s.pts[i - 1][1]));
    s.k = s.pts.map(() => 0);
    for (let j = 1; j < s.pts.length - 1; j++) {
      const ds = Math.hypot(s.pts[j + 1][0] - s.pts[j][0], s.pts[j + 1][1] - s.pts[j][1]) + Math.hypot(s.pts[j][0] - s.pts[j - 1][0], s.pts[j][1] - s.pts[j - 1][1]);
      s.k[j] = ds > 1e-6 ? Math.abs(wrap(s.pts[j + 1][2] - s.pts[j - 1][2])) / ds : 0;
    }
    if (s.k.length > 2) { s.k[0] = s.k[1]; s.k[s.k.length - 1] = s.k[s.k.length - 2]; }
  }
  const T = { segs, seg: 0, idx: 0, pauseUntil: 0, deviation: 0, done: !segs.length, finalHold: null, cusps: 0 };

  function advance(x, y, now) {
    while (!T.done) {
      const { pts, d } = segs[T.seg];
      const [ex, ey, eyaw] = pts[pts.length - 1];
      const tx = d * Math.cos(eyaw), ty = d * Math.sin(eyaw);
      const distEnd = Math.hypot(ex - x, ey - y);
      const passed = (x - ex) * tx + (y - ey) * ty > 0 && distEnd < Pm.lookahead;
      if (distEnd >= Pm.segEndTol && !passed) break;
      T.seg += 1; T.idx = 0;
      if (T.seg >= segs.length) T.done = true;
      else if (segs[T.seg].d !== d) { T.pauseUntil = now + Pm.cuspPause; T.cusps += 1; }
    }
    if (T.done) return true;
    const { pts } = segs[T.seg];
    let bj = T.idx, bd = Infinity;
    for (let j = T.idx; j < Math.min(pts.length, T.idx + 40); j++) {
      const dd = Math.hypot(pts[j][0] - x, pts[j][1] - y);
      if (dd < bd) { bj = j; bd = dd; }
    }
    T.idx = bj; T.deviation = bd;
    return false;
  }
  function curveRatio() {
    const s = segs[T.seg];
    let ratio = Pm.curveSpeedRatio;
    const final = Pm.finalTurnRatio < 1 && T.seg === segs.length - 1 && s.cum[s.cum.length - 1] - s.cum[T.idx] <= Pm.finalTurnDist;
    if (final) ratio = Math.min(ratio, Pm.finalTurnRatio);
    if (ratio >= 1 && !final) return 1;
    const limit = s.cum[T.idx] + Pm.lookahead;
    let kMax = 0;
    for (let j = T.idx; j < s.k.length && s.cum[j] <= limit; j++) kMax = Math.max(kMax, s.k[j]);
    const f = Math.min(1, kMax / kRef);
    let out = 1 - (1 - ratio) * f;
    if (final) { out = Math.min(out, T.finalHold ?? out); T.finalHold = out; }
    return out;
  }
  function speedRatio() {
    const s = segs[T.seg];
    const remaining = s.cum[s.cum.length - 1] - s.cum[T.idx];
    return Math.min(Math.max(Pm.minSpeedRatio, Math.min(1, remaining / Pm.slowdownDist)), curveRatio());
  }
  function target(x, y) {
    const { pts, d } = segs[T.seg];
    const ld = Pm.lookahead;
    for (let j = T.idx + 1; j < pts.length; j++) {
      if (Math.hypot(pts[j][0] - x, pts[j][1] - y) >= ld) return circleHit(pts[j - 1][0], pts[j - 1][1], pts[j][0], pts[j][1], x, y, ld) || [pts[j][0], pts[j][1]];
    }
    const [ex, ey, eyaw] = pts[pts.length - 1];
    const ext = Math.max(ld - Math.hypot(ex - x, ey - y), 0.3 * ld);
    return [ex + d * Math.cos(eyaw) * ext, ey + d * Math.sin(eyaw) * ext];
  }
  T.update = (x, y, yaw, now) => {
    if (advance(x, y, now)) return { sps: 0, steer: 0, target: null, done: true };
    const d = segs[T.seg].d;
    const tg = target(x, y);
    const motionYaw = d > 0 ? yaw : yaw + Math.PI;
    const alpha = wrap(Math.atan2(tg[1] - y, tg[0] - x) - motionYaw);
    const dist = Math.max(Math.hypot(tg[0] - x, tg[1] - y), 1e-6);
    const kappa = (d * 2 * Math.sin(alpha)) / dist;
    const delta = Math.max(-maxSteer, Math.min(maxSteer, Math.atan(Pm.L * kappa)));
    let sps = d * (d > 0 ? Pm.forwardSps : Pm.reverseSps) * speedRatio();
    const paused = now < T.pauseUntil;
    if (paused) sps = 0;
    return { sps: Math.trunc(sps), steer: delta, target: tg, done: false, paused, dir: d };
  };
  return T;
}

// Simulasi tertutup: pelacak 20 Hz, kendaraan sepeda dengan acuan poros depan.
// Mengembalikan sampel 20 Hz: [t, x, y, yaw_deg, steer_deg, v_cm_s, tx, ty, s_cm].
export function simulate(pathDeg, Pm = PARAM, { tMax = 120 } = {}) {
  const T = tracker(pathDeg, Pm);
  let [x, y, yawDeg] = pathDeg[0];
  let yaw = (yawDeg * Math.PI) / 180, delta = 0, v = 0, s = 0, t = 0;
  const maxSteer = (Pm.maxSteerDeg * Math.PI) / 180, slew = (Pm.slewDegS * Math.PI) / 180;
  const accel = Pm.accelSps2 * Pm.cmPerStep;
  const queue = [];               // perintah kemudi yang tertunda
  let cmd = { sps: 0, steer: 0, target: null, done: false };
  let steerCmd = 0, servo = 0, doneAt = null;
  const out = [];
  const nSub = Math.round(Pm.control / Pm.substep);
  while (t < tMax) {
    cmd = T.update(x, y, yaw, t);
    queue.push([t + Pm.steerDead, cmd.steer]);
    out.push([t, x, y, (yaw * 180) / Math.PI, (delta * 180) / Math.PI, v, cmd.target ? cmd.target[0] : null, cmd.target ? cmd.target[1] : null, s, T.deviation, cmd.paused ? 1 : 0]);
    if (cmd.done && doneAt === null) doneAt = t;
    if (doneAt !== null && Math.abs(v) < 1e-3) break;
    const vCmd = cmd.sps * Pm.cmPerStep;
    for (let i = 0; i < nSub; i++) {
      const dt = Pm.substep;
      while (queue.length && queue[0][0] <= t + 1e-9) steerCmd = queue.shift()[1];
      servo += Math.max(-slew * dt, Math.min(slew * dt, steerCmd - servo));
      delta += ((Math.max(-maxSteer, Math.min(maxSteer, servo)) - delta) * dt) / Pm.steerTau;
      v += Math.max(-accel * dt, Math.min(accel * dt, vCmd - v));
      x += v * Math.cos(yaw) * dt;
      y += v * Math.sin(yaw) * dt;
      yaw += ((v * Math.tan(delta)) / Pm.L) * dt;
      s += Math.abs(v) * dt;
      t += dt;
    }
  }
  return { samples: out, doneAt, cusps: T.cusps };
}

// ------------------------------------------------------------------ dua robot
// Versi ringkas wasit.py: bayangan badan sepanjang jalur sejauh jarak henti
// L = 5 + v*0,2 + v^2/(2a); aturan "di jalur" (bayangan A mendekati badan B di
// bawah jarak aman 5 cm -> A menunggu) dan "silang" (bayangan bertemu bayangan
// -> yang kalah prioritas menunggu). Prioritas: yang mundur, lalu yang sisa
// jalurnya lebih pendek. Pergeseran bayangan ke badan (ikut_badan) dan manuver
// minggir tidak dimodelkan.
export const MULTI = { lihatMin: 5.0, tReaksi: 0.2, jarakAman: 5.0, sisaSetara: 5.0, lanjut: 0.5 };

export function bodyPoly(x, y, yawRad, G = { front: 4.0, rear: 23.0, half: 8.75 }) {
  const c = Math.cos(yawRad), s = Math.sin(yawRad);
  return [[G.front, G.half], [-G.rear, G.half], [-G.rear, -G.half], [G.front, -G.half]].map(([u, v]) => [x + u * c - v * s, y + u * s + v * c]);
}
function intersects(P, Q) {
  for (const R of [P, Q]) for (let i = 0; i < R.length; i++) {
    const [ax, ay] = R[i], [bx, by] = R[(i + 1) % R.length], nx = by - ay, ny = ax - bx;
    let p0 = Infinity, p1 = -Infinity, q0 = Infinity, q1 = -Infinity;
    for (const [x, y] of P) { const d = nx * x + ny * y; p0 = Math.min(p0, d); p1 = Math.max(p1, d); }
    for (const [x, y] of Q) { const d = nx * x + ny * y; q0 = Math.min(q0, d); q1 = Math.max(q1, d); }
    if (p1 < q0 || q1 < p0) return false;
  }
  return true;
}
function segDist([px, py], [ax, ay], [bx, by]) {
  const dx = bx - ax, dy = by - ay, l = dx * dx + dy * dy;
  const t = l ? Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l)) : 0;
  return Math.hypot(px - ax - t * dx, py - ay - t * dy);
}
export function polyDist(P, Q) {
  if (intersects(P, Q)) return 0;
  let m = Infinity;
  for (const [A, B] of [[P, Q], [Q, P]]) for (const p of A) for (let i = 0; i < B.length; i++) m = Math.min(m, segDist(p, B[i], B[(i + 1) % B.length]));
  return m;
}
const setDist = (As, Bs) => { let m = Infinity; for (const a of As) for (const b of Bs) { m = Math.min(m, polyDist(a, b)); if (!m) return 0; } return m; };

export function simulateMulti(paths, Pm = PARAM, M = MULTI, { tMax = 120 } = {}) {
  const maxSteer = (Pm.maxSteerDeg * Math.PI) / 180, slew = (Pm.slewDegS * Math.PI) / 180;
  const accel = Pm.accelSps2 * Pm.cmPerStep;
  const R = paths.map((pathDeg, i) => {
    const [x, y, yawDeg] = pathDeg[0];
    return { i, T: tracker(pathDeg, Pm), x, y, yaw: (yawDeg * Math.PI) / 180, delta: 0, v: 0, s: 0, servo: 0, steerCmd: 0, queue: [], out: [], doneAt: null, wait: 0, waited: 0, why: "" };
  });
  const remaining = (r) => {
    const T = r.T; if (T.done) return 0;
    let rem = T.segs[T.seg].cum.at(-1) - T.segs[T.seg].cum[T.idx];
    for (let k = T.seg + 1; k < T.segs.length; k++) rem += T.segs[k].cum.at(-1);
    return rem;
  };
  const shadow = (r) => {
    const T = r.T; if (T.done) return [bodyPoly(r.x, r.y, r.yaw)];
    const L = M.lihatMin + Math.abs(r.v) * M.tReaksi + (r.v * r.v) / (2 * accel);
    const polys = []; let run = 0, last = null;
    for (let k = T.seg; k < T.segs.length && run < L; k++) {
      const pts = T.segs[k].pts;
      for (let j = k === T.seg ? T.idx : 1; j < pts.length; j++) {
        const p = pts[j];
        if (last) run += Math.hypot(p[0] - last[0], p[1] - last[1]);
        polys.push(bodyPoly(p[0], p[1], p[2]));
        last = p;
        if (run >= L) break;
      }
    }
    return polys;
  };
  let loser = null, t = 0, minGap = Infinity;
  const nSub = Math.round(Pm.control / Pm.substep);
  while (t < tMax) {
    const cmds = R.map((r) => r.T.update(r.x, r.y, r.yaw, t));
    // Wasit.
    const [a, b] = R;
    const bodyA = bodyPoly(a.x, a.y, a.yaw), bodyB = bodyPoly(b.x, b.y, b.yaw);
    const dNow = polyDist(bodyA, bodyB);
    minGap = Math.min(minGap, dNow);
    const runA = !a.T.done, runB = !b.T.done;
    const lim = Math.min(M.jarakAman, dNow - Math.min(0.5, dNow / 2));
    const shA = shadow(a), shB = shadow(b);
    const ab = runA && setDist(shA, [bodyB]) < lim, ba = runB && setDist(shB, [bodyA]) < lim;
    const pick = () => {
      if (loser !== null) return R[loser];
      const da = cmds[0].dir ?? 1, db = cmds[1].dir ?? 1;
      if (da < 0 && db > 0) return b;
      if (db < 0 && da > 0) return a;
      const ra = remaining(a), rb = remaining(b);
      return Math.abs(ra - rb) <= M.sisaSetara ? b : ra < rb ? b : a;
    };
    const waits = [false, false];
    if (ab && ba) { const l = pick(); loser = l.i; waits[l.i] = true; R[l.i].why = "berhadapan"; }
    else if (ab) { waits[0] = true; a.why = "robot 02 ada di jalur"; }
    else if (ba) { waits[1] = true; b.why = "robot 01 ada di jalur"; }
    else if (runA && runB && setDist(shA, shB) < lim) { const l = pick(); loser = l.i; waits[l.i] = true; R[l.i].why = "memberi jalan ke robot 0" + (2 - l.i); }
    else loser = null;
    for (const r of R) {
      const cmd = cmds[r.i];
      if (waits[r.i]) r.wait = M.lanjut; else r.wait = Math.max(0, r.wait - Pm.control);
      const waiting = r.wait > 0 && !cmd.done;
      if (waiting) r.waited += Pm.control;
      r.queue.push([t + Pm.steerDead, cmd.steer]);
      r.out.push([t, r.x, r.y, (r.yaw * 180) / Math.PI, (r.delta * 180) / Math.PI, r.v, cmd.target ? cmd.target[0] : null, cmd.target ? cmd.target[1] : null, r.s, r.T.deviation, cmd.paused ? 1 : 0, waiting ? 1 : 0]);
      if (cmd.done && r.doneAt === null) r.doneAt = t;
      r.vCmd = waiting ? 0 : cmd.sps * Pm.cmPerStep;
    }
    if (R.every((r) => r.doneAt !== null && Math.abs(r.v) < 1e-3)) break;
    for (let i = 0; i < nSub; i++) {
      const dt = Pm.substep;
      for (const r of R) {
        while (r.queue.length && r.queue[0][0] <= t + 1e-9) r.steerCmd = r.queue.shift()[1];
        r.servo += Math.max(-slew * dt, Math.min(slew * dt, r.steerCmd - r.servo));
        r.delta += ((Math.max(-maxSteer, Math.min(maxSteer, r.servo)) - r.delta) * dt) / Pm.steerTau;
        r.v += Math.max(-accel * dt, Math.min(accel * dt, r.vCmd - r.v));
        r.x += r.v * Math.cos(r.yaw) * dt;
        r.y += r.v * Math.sin(r.yaw) * dt;
        r.yaw += ((r.v * Math.tan(r.delta)) / Pm.L) * dt;
        r.s += Math.abs(r.v) * dt;
      }
      t += dt;
    }
  }
  return { robots: R.map((r) => ({ samples: r.out, doneAt: r.doneAt, waited: r.waited, cusps: r.T.cusps })), minGap };
}
