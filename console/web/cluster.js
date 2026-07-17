/**
 * @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      cluster.js
 * @brief     IONIQ5 ccNC-style cluster renderer (display only)
 *
 * @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
 *            2026-07-17 updated by Junhyeok Seo (jun2342@chungbuk.ac.kr)
 *              : add gear (PRND) display, reverse/parking visuals, mirrored steering & road tuning
 */
"use strict";
// Receives vECU state from /stream (SSE) and draws it on Canvas. Nothing is sent on the bus.
// Fixed 8:3 design coordinate system (1600x600, real-car 12.3" ccNC ratio) letterbox-fit,
// so ratios, gauge spacing and car size are locked regardless of window size.

const DESIGN_W = 1600;            // 8 : 3
const DESIGN_H = 600;

const canvas = document.getElementById("cluster");
const ctx = canvas.getContext("2d");
const statusEl = document.getElementById("status");

const COL = {
  bg: "#05070a",
  face: "#16181f",   // gauge inner face (slightly dark gray, ccNC look)
  ring: "#22262e",
  ringL: "#222832",  // left (speed/accel) background track
  ringR: "#2b2a27",  // right (steer/brake) background track
  speed: "#5aa9e6",
  steer: "#e6a23c",
  accel: "#5aa9e6",
  brake: "#e0584f",
  text: "#f2f4f7",
  white: "#ffffff",  // unified color for bottom-arc current value (accel/brake)
  access: "#4ade80", // ACCESS color when bus connected
  indGreen: "#4ade80",  // indicator: control mode
  indOrange: "#e6a23c", // indicator: partially controlled (manual)
  indRed: "#e0584f",    // indicator: fault
  sub: "#8b9099",
  lane: "#e7ecf2",
  road: "#0e1117",
  bar: "#1a1d24",    // bottom-center bar
};

let target = {
  speed: 0, speed_max: 180,
  steer_deg: 0, steer_limit: 480,
  accel_pct: 0, brake_mm: 0, brake_max_mm: 60, brake_on: false,
  connected: false,
  steer_ctrl: false, steer_fault: false,
  brake_ctrl: false, brake_fault: false,
  accel_ctrl: false,
  gear: "P",
};
let cur = Object.assign({}, target);

// -- car image (fallback render if missing) ---------------------------------
function loadImg(src) {
  const im = new Image();
  im.ready = false;
  im.onload = () => { im.ready = true; };
  im.src = src;
  return im;
}
const carBasic = loadImg("/assets/ioniq5_basic.png");
const carBrake = loadImg("/assets/ioniq5_brake.png");

// -- lab logo (bottom-right watermark) ---------------------------------------
const avlabLogo = loadImg("/assets/avlab_logo.png");

// -- indicator icons (alpha silhouette -> recolored) -------------------------
const icoManual = loadImg("/assets/indicator_manual.png");
const icoSteer = loadImg("/assets/indicator_steering.png");
const icoBrake = loadImg("/assets/indicator_brake.png");
const icoAccel = loadImg("/assets/indicator_accel.png");
const icoParking = loadImg("/assets/indicator_parking_brake.png");

// source-in compositing keeps the icon alpha (shape) and fills a solid color.
// (each icon x color combo is built once and cached)
const _tintCache = new Map();
function tinted(img, color) {
  if (!img.ready || !img.naturalWidth) return null;
  const key = img.src + "|" + color;
  let off = _tintCache.get(key);
  if (off) return off;
  off = document.createElement("canvas");
  off.width = img.naturalWidth;
  off.height = img.naturalHeight;
  const o = off.getContext("2d");
  o.drawImage(img, 0, 0);
  o.globalCompositeOperation = "source-in";
  o.fillStyle = color;
  o.fillRect(0, 0, off.width, off.height);
  _tintCache.set(key, off);
  return off;
}

// -- car / per-gear display tuning constants (tune position/size here) -------
const CAR_W = 150;                    // car width (design px)
const CAR_BASE_Y = DESIGN_H * 0.64;   // car top y in normal driving
const CAR_LIFT = -200;                // upward shift in R (negative = up)
const CAR_HOME_EPS = 2;               // within this = "back home" (delays lane re-show)
let carLift = 0;                      // actual applied offset (eased on R transition)

// R reverse parking guide (light yellow curves)
const GUIDE_COL = "rgba(255,236,120,0.85)";
// R reverse lights (car rear)
const REV_LIGHT = { w: 0.04, h: 0.03, y: 0.6, dx: 0.19, color: "#fffdf5", glow: 16 };
// P parking-brake indicator (bottom-left, red)
const PARK_ICON = { x: 5, y: DESIGN_H - 40, size: 40 };

function carHeight() {
  const img = cur.brake_on ? carBrake : carBasic;
  return (img.ready && img.naturalWidth)
    ? CAR_W * (img.naturalHeight / img.naturalWidth) : CAR_W * 0.55;
}

// -- SSE ---------------------------------------------------------------------
function connect() {
  const es = new EventSource("/stream");
  es.onopen = () => { statusEl.textContent = "Connected"; statusEl.className = "online"; };
  es.onmessage = (e) => { try { target = JSON.parse(e.data); } catch (_) {} };
  es.onerror = () => { statusEl.textContent = "Disconnected - reconnecting..."; statusEl.className = "offline"; };
}
connect();

// -- canvas sizing + 8:3 letterbox fit ---------------------------------------
let fit = { scale: 1, offX: 0, offY: 0, dpr: 1 };
function resize() {
  const dpr = window.devicePixelRatio || 1;
  const W = window.innerWidth, H = window.innerHeight;
  canvas.width = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  const scale = Math.min(W / DESIGN_W, H / DESIGN_H);
  fit = {
    scale,
    offX: (W - DESIGN_W * scale) / 2,
    offY: (H - DESIGN_H * scale) / 2,
    dpr,
  };
}
window.addEventListener("resize", resize);
resize();

// -- drawing helpers (all in the 1600x600 design coordinate system) ----------
const D2R = Math.PI / 180;
const lerp = (a, b, t) => a + (b - a) * t;
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

function arc(cx, cy, r, a0, a1, width, color, anti) {
  ctx.beginPath();
  ctx.arc(cx, cy, r, a0 * D2R, a1 * D2R, !!anti);
  ctx.lineWidth = width;
  ctx.strokeStyle = color;
  ctx.lineCap = "round";
  ctx.stroke();
}

function text(s, x, y, size, color, align = "center", weight = "300") {
  ctx.fillStyle = color;
  ctx.textAlign = align;
  ctx.textBaseline = "middle";
  ctx.font = `${weight} ${size}px "Segoe UI","Noto Sans KR",system-ui,sans-serif`;
  ctx.fillText(s, x, y);
}

function strokePath(pts, color, w) {
  ctx.beginPath();
  ctx.moveTo(pts[0][0], pts[0][1]);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
  ctx.lineWidth = w;
  ctx.strokeStyle = color;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.stroke();
}

// Main (top 230 deg) + bottom (90 deg) form one circle at the same radius, 20 deg gap between them.
//  main: 155 -> 385 (=25) 230 deg,  bottom: 135 -> 45 90 deg (bottom, anti)
const MAIN_A0 = 155, MAIN_A1 = 385, MAIN_MID = 270;
const BOT_A0 = 135, BOT_A1 = 45;
function gauge(cx, cy, R, o) {
  // inner face (slightly dark gray disc)
  ctx.beginPath();
  ctx.arc(cx, cy, R * 1.04, 0, Math.PI * 2);
  ctx.fillStyle = COL.face;
  ctx.fill();

  // Draw both arcs so their outer edge touches the outermost inner-face disc (R*1.04),
  // insetting by half the line width.
  const MAIN_W = R * 0.085;
  const Rmain = R * 1.04 - MAIN_W / 2;
  const BOT_W = R * 0.024;
  const Rbot = R * 1.04 - BOT_W / 2;
  const ringCol = o.ringColor || COL.ring;   // top track color (the not-yet-filled part)

  // background rings (top thick - track color / bottom thin - default ring color)
  arc(cx, cy, Rmain, MAIN_A0, MAIN_A1, MAIN_W, ringCol);
  arc(cx, cy, Rbot, BOT_A0, BOT_A1, BOT_W, COL.ring, true);

  // main value (thick top arc)
  if (o.bidir) {
    // mirror=true: fill positive values to the left (matches HILS/CARLA left/right). Number unchanged.
    const dir = o.mirror ? -1 : 1;
    const ang = MAIN_MID + dir * clamp(o.value / o.max, -1, 1) * ((MAIN_A1 - MAIN_A0) / 2);
    arc(cx, cy, Rmain, Math.min(MAIN_MID, ang), Math.max(MAIN_MID, ang), R * 0.078, o.color);
  } else {
    const f = clamp(o.value / o.max, 0, 1);
    if (f > 0) arc(cx, cy, Rmain, MAIN_A0, MAIN_A0 + (MAIN_A1 - MAIN_A0) * f, R * 0.078, o.color);
  }
  // bottom value (accel/brake) - thin bottom arc, current value unified white
  const bf = clamp(o.bottomValue / o.bottomMax, 0, 1);
  if (bf > 0) arc(cx, cy, Rbot, BOT_A0, BOT_A0 - (BOT_A0 - BOT_A1) * bf, BOT_W, COL.white, true);

  // center big number (small + bold)
  text(o.big, cx, cy - R * 0.05, R * 0.3, COL.text, "center", "350");
  text(o.unit, cx, cy + R * 0.20, R * 0.1, COL.sub);
  // ACCEL/BRAKE labels - attached to the bottom arc (like ccNC fuel gauge)
  text(o.bottomLabel, cx, cy + R * 0.75, R * 0.10, COL.sub);
  text(o.bottomMinLabel, cx - R * 0.55, cy + R * 0.65, R * 0.095, COL.sub);
  text(o.bottomMaxLabel, cx + R * 0.55, cy + R * 0.65, R * 0.095, COL.sub);
}

// (3) center: lane that actually bends with steering via t^2 + car
//   - the near (bottom) center is always fixed to the car center (cx); only the far (top) end bends.
//   - the centerline is left/right symmetric, so full-left and full-right steering mirror each other.
//
// -- road (drivable area) tuning ---------------------------------------------
//   position/size/bend are all adjusted only here.
//   * halfTop/halfBot are auto-capped so they never exceed GRAY_HALF (gray divider half-width).
//   * the road is clipped inside the gray divider width (cx +/- GRAY_HALF) so it never overlaps the side arcs.
const GRAY_HALF = 155;   // indicators() divider half-width = line width / display-area cap
const ROAD = {
  halfTop: 60,     // far (top) half-width      <- tune directly
  halfBot: 120,    // near (bottom) half-width   <- tune directly (capped at GRAY_HALF)
  topY: 230,       // top start y   <- tune directly (smaller = higher; cluster middle ~300)
  botY: 540,       // bottom end y  <- tune directly
  topBend: 190,    // far (top) bend amount <- tune directly (larger = bends more)
};
function road() {
  const cx = DESIGN_W / 2, topY = ROAD.topY, botY = ROAD.botY;
  // cap half-widths so the lane never exceeds the gray divider width
  const halfBot = Math.min(ROAD.halfBot, GRAY_HALF);
  const halfTop = Math.min(ROAD.halfTop, GRAY_HALF);
  // bend positive angles to the left (matches HILS/CARLA left/right). Displayed steer_deg unchanged.
  //  dir>0 -> road bends to the right (+x) (wheel turned right).
  const dir = clamp(cur.steer_deg / cur.steer_limit, -1, 1) * -1;
  const topBend = dir * ROAD.topBend;    // only the far (top) end moves; the near (t=0) end stays at cx
  const N = 28;

  // near (t=0) center = cx (car center); bends by topBend toward the far (t=1) end.
  const center = (t) => cx + topBend * t * t;
  const halfW = (t) => halfTop + (halfBot - halfTop) * Math.pow(1 - t, 1.55);
  const sample = (side) => {
    const pts = [];
    for (let i = 0; i <= N; i++) {
      const t = i / N;
      pts.push([center(t) + side * halfW(t), botY - (botY - topY) * t]);
    }
    return pts;
  };
  const L = sample(-1), Rr = sample(1);

  ctx.save();
  ctx.beginPath();
  // draw the road only within the gray divider width (cx +/- GRAY_HALF) (avoids overlapping the side arcs).
  ctx.rect(cx - GRAY_HALF, topY - 12, GRAY_HALF * 2, botY - topY + 60);
  ctx.clip();

  // In reverse (R), erase the road (surface + lanes) and draw only the parking guide. After leaving R,
  // the surface/lanes reappear only once the car has settled back home (carLift returns).
  if (cur.gear === "R") {
    reverseGuides(cx);                       // reverse: guide lines behind the car (inside the clip)
  } else if (carLift > -CAR_HOME_EPS) {
    // road surface
    ctx.beginPath();
    ctx.moveTo(L[0][0], L[0][1]);
    for (const p of L) ctx.lineTo(p[0], p[1]);
    for (let i = Rr.length - 1; i >= 0; i--) ctx.lineTo(Rr[i][0], Rr[i][1]);
    ctx.closePath();
    ctx.fillStyle = COL.road;
    ctx.fill();
    // side lanes (no center dashes)
    strokePath(L, COL.lane, 3);
    strokePath(Rr, COL.lane, 3);
  }
  ctx.restore();

  drawCar(cx);
}

// R reverse parking guide: curves reaching from the car rear toward the camera (bottom), bending with steering.
function reverseGuides(cx) {
  const y0 = CAR_BASE_Y + carLift + carHeight() + 6;  // start right behind the car
  const y1 = 540;
  const halfTop = 34, halfBot = Math.min(ROAD.halfBot, GRAY_HALF);  // near car width -> toward camera (<= gray divider width)
  const bend = clamp(cur.steer_deg / cur.steer_limit, -1, 1) * -120;  // same direction as road
  const N = 20;
  const center = (t) => cx + bend * t * t;
  const halfW = (t) => halfTop + (halfBot - halfTop) * t;
  const sample = (side) => {
    const pts = [];
    for (let i = 0; i <= N; i++) {
      const t = i / N;
      pts.push([center(t) + side * halfW(t), y0 + (y1 - y0) * t]);
    }
    return pts;
  };
  strokePath(sample(-1), GUIDE_COL, 4);
  strokePath(sample(1), GUIDE_COL, 4);
  // distance bands (3 horizontal lines)
  for (const t of [0.33, 0.62, 0.9]) {
    const y = y0 + (y1 - y0) * t, c = center(t), hw = halfW(t);
    strokePath([[c - hw, y], [c + hw, y]], GUIDE_COL, 3);
  }
}

function drawCar(cx) {
  const img = cur.brake_on ? carBrake : carBasic;
  const cw = CAR_W;                     // fixed width (keep aspect ratio, avoid squashing)
  const y = CAR_BASE_Y + carLift;       // shifted up in R (smoothly eased)
  const ch = carHeight();
  const x = cx - cw / 2;
  if (img.ready) {
    ctx.drawImage(img, x, y, cw, ch);
  } else {
    roundRect(x, y, cw, ch, 12);
    ctx.fillStyle = "#9aa0aa"; ctx.fill();
    ctx.fillStyle = cur.brake_on ? COL.brake : "#5b6068";
    ctx.fillRect(x + cw * 0.12, y + ch * 0.22, cw * 0.76, ch * 0.16);
  }
  if (cur.gear === "R") drawReverseLights(x, y, cw, ch);
}

// R reverse lights (car rear left/right, bright). Position/size via REV_LIGHT constants.
function drawReverseLights(x, y, cw, ch) {
  const lw = cw * REV_LIGHT.w, lh = ch * REV_LIGHT.h;
  const cy = y + ch * REV_LIGHT.y, cx = x + cw / 2;
  ctx.save();
  ctx.shadowColor = REV_LIGHT.color;
  ctx.shadowBlur = REV_LIGHT.glow;
  ctx.fillStyle = REV_LIGHT.color;
  for (const s of [-1, 1]) {
    roundRect(cx + s * cw * REV_LIGHT.dx - lw / 2, cy - lh / 2, lw, lh, 3);
    ctx.fill();
  }
  ctx.restore();
}

// P parking-brake indicator (bottom-left, red)
function parkingIndicator() {
  const t = tinted(icoParking, COL.indRed);
  if (t) ctx.drawImage(t, PARK_ICON.x, PARK_ICON.y, PARK_ICON.size, PARK_ICON.size);
}

// bottom center: rounded-top rect (left ACCESS icon / right gear) - ccNC look
//  left: show 'ACCESS' only when bus connected (hidden when disconnected). No temperature.
function bottomBar() {
  const w = 420, h = 50, r = 14;
  const x = DESIGN_W / 2 - w / 2, y = DESIGN_H - h, cy = y + h / 2;
  ctx.beginPath();
  ctx.moveTo(x, y + h);
  ctx.lineTo(x, y + r);
  ctx.arcTo(x, y, x + r, y, r);
  ctx.lineTo(x + w - r, y);
  ctx.arcTo(x + w, y, x + w, y + r, r);
  ctx.lineTo(x + w, y + h);
  ctx.closePath();
  ctx.fillStyle = COL.bar;
  ctx.fill();

  // left: show 'ACCESS' text only when bus connected
  if (cur.connected) {
    text("ACCESS", x + 30, cy, 22, COL.access, "left", "700");
  }
  // right: gear (input.py PRND keys -> gear_link UDP -> snapshot.gear)
  text(cur.gear, x + w - 30, cy, 28, COL.text, "right", "700");
}

// top-center indicators (MANUAL/STEER/BRAKE/ACCEL) + a divider line.
//  servo (steer/brake): control=green, fault=red, else (manual)=white.
//  accel: control=green, else=white (2 colors).
//  manual: among the 3, control count 0=green / 1~2=orange / 3=white.
function actuatorColor(ctrl, fault) {
  return fault ? COL.indRed : ctrl ? COL.indGreen : COL.white;
}
function manualColor() {
  const n = (cur.steer_ctrl ? 1 : 0) + (cur.brake_ctrl ? 1 : 0)
          + (cur.accel_ctrl ? 1 : 0);
  return n === 0 ? COL.indGreen : n === 3 ? COL.white : COL.indOrange;
}

function indicators() {
  const items = [
    [icoManual, manualColor()],
    [icoSteer, actuatorColor(cur.steer_ctrl, cur.steer_fault)],
    [icoBrake, actuatorColor(cur.brake_ctrl, cur.brake_fault)],
    [icoAccel, cur.accel_ctrl ? COL.indGreen : COL.white],
  ];
  const cx = DESIGN_W / 2, size = 40, gap = 74, y = 24;

  // icon row
  items.forEach(([img, color], i) => {
    const x = cx + (i - (items.length - 1) / 2) * gap - size / 2;
    const t = tinted(img, color);
    if (t) ctx.drawImage(t, x, y, size, size);
  });

  // faint white divider line (below the icons)
  const lineY = y + size + 10, halfW = 155;
  ctx.beginPath();
  ctx.moveTo(cx - halfW, lineY);
  ctx.lineTo(cx + halfW, lineY);
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = "rgba(255,255,255,0.14)";
  ctx.stroke();
}

function roundRect(x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

// bottom-right lab logo (AVLAB) - drawn last so it sits on top
function logo() {
  if (!avlabLogo.ready || !avlabLogo.naturalWidth) return;
  const w = 130;
  const h = w * (avlabLogo.naturalHeight / avlabLogo.naturalWidth);
  const x = DESIGN_W - w;   // flush to the right edge
  const y = DESIGN_H - h;   // flush to the bottom edge
  ctx.save();
  ctx.globalAlpha = 0.7;
  ctx.drawImage(avlabLogo, x, y, w, h);
  ctx.restore();
}

// -- render loop -------------------------------------------------------------
function smooth() {
  const t = 0.18;
  cur.speed = lerp(cur.speed, target.speed, t);
  // steer renders encoder_pos as-is -> left/right matches HILS/CARLA (direct real-ECU link, the reference).
  // (the cluster is display-only, so the sign is just a render convention and never touches bus data)
  cur.steer_deg = lerp(cur.steer_deg, target.steer_deg, t);
  cur.accel_pct = lerp(cur.accel_pct, target.accel_pct, t);
  cur.brake_mm = lerp(cur.brake_mm, target.brake_mm, t);
  cur.speed_max = target.speed_max;
  cur.steer_limit = target.steer_limit;
  cur.brake_max_mm = target.brake_max_mm;
  cur.brake_on = target.brake_on;
  cur.connected = target.connected;
  cur.steer_ctrl = target.steer_ctrl;
  cur.steer_fault = target.steer_fault;
  cur.brake_ctrl = target.brake_ctrl;
  cur.brake_fault = target.brake_fault;
  cur.accel_ctrl = target.accel_ctrl;
  cur.gear = target.gear;
  // lift the car up in R (smoothly); return home when switched to another gear
  carLift = lerp(carLift, cur.gear === "R" ? CAR_LIFT : 0, 0.1);
}

function frame() {
  smooth();

  // clear all to black (letterbox area)
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = "#000";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // fit into the 8:3 design coordinate system
  const s = fit.scale * fit.dpr;
  ctx.setTransform(s, 0, 0, s, fit.offX * fit.dpr, fit.offY * fit.dpr);

  ctx.fillStyle = COL.bg;
  ctx.fillRect(0, 0, DESIGN_W, DESIGN_H);

  const R = 258;
  const cy = 296;
  const lx = 336, rx = DESIGN_W - 336;

  // (1) speed (left)  +  (4) accel (bottom-left)
  gauge(lx, cy, R, {
    value: cur.speed, max: cur.speed_max, ringColor: COL.ringL,
    big: Math.round(cur.speed).toString(), unit: "km/h", color: COL.speed,
    bottomValue: cur.accel_pct, bottomMax: 100,
    bottomLabel: `ACCEL ${cur.accel_pct.toFixed(0)}%`,
    bottomMinLabel: "0", bottomMaxLabel: "100",
  });
  // (2) steer (right, bidirectional)  +  (5) brake (bottom-right)
  gauge(rx, cy, R, {
    value: cur.steer_deg, max: cur.steer_limit, bidir: true, mirror: true, ringColor: COL.ringR,
    big: `${cur.steer_deg >= 0 ? "+" : ""}${cur.steer_deg.toFixed(0)}°`,
    unit: "deg", color: COL.steer,
    bottomValue: cur.brake_mm, bottomMax: cur.brake_max_mm,
    bottomLabel: `BRAKE ${cur.brake_mm.toFixed(1)}mm`,
    bottomMinLabel: "0", bottomMaxLabel: `${Math.round(cur.brake_max_mm)}`,
  });

  // (3) center
  road();
  bottomBar();
  indicators();
  if (cur.gear === "P") parkingIndicator();   // P: bottom-left parking-brake light (red)
  logo();

  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
