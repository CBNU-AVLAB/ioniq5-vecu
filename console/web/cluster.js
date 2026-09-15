/**
 * @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
 *            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 *
 * @file      cluster.js
 * @brief     IONIQ5 ccNC-style cluster renderer (display only)
 *
 * @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
 *            2026-07-17 updated by Junhyeok Seo (jun2342@chungbuk.ac.kr)
 *              : add gear (PRND) display, reverse/parking visuals, mirrored steering & road tuning
 */

"use strict";

const DESIGN_W = 1600;            // 8 : 3
const DESIGN_H = 600;

const canvas = document.getElementById("cluster");
const ctx = canvas.getContext("2d");
const statusEl = document.getElementById("status");

const COL = {
  bg: "#05070a",
  face: "#16181f",   // gauge face
  ring: "#22262e",
  ringL: "#222832",  // left track (speed/accel)
  ringR: "#2b2a27",  // right track (steering/brake)
  speed: "#5aa9e6",
  steer: "#e6a23c",
  accel: "#5aa9e6",
  brake: "#e0584f",
  text: "#f2f4f7",
  white: "#ffffff",  // bottom arc value (accel/brake)
  access: "#4ade80", // ACCESS text while the bus is connected
  indGreen: "#4ade80",  // indicator: control mode
  indOrange: "#e6a23c", // indicator: partial control
  indRed: "#e0584f",    // indicator: fault
  sub: "#8b9099",
  lane: "#e7ecf2",
  road: "#0e1117",
  bar: "#1a1d24",    // bottom center bar
};

let target = {
  speed: 0, speed_max: 180,
  steer_deg: 0, steer_limit: 480,
  accel_pct: 0, brake_mm: 0, brake_max_mm: 170, brake_on: false,
  connected: false,
  steer_ctrl: false, steer_fault: false,
  brake_ctrl: false, brake_fault: false,
  accel_ctrl: false,
  gear: "P",
};
let cur = Object.assign({}, target);

// ── Car images (fallback rendering if missing) ──────────────────
function loadImg(src) {
  const im = new Image();
  im.ready = false;
  im.onload = () => { im.ready = true; };
  im.src = src;
  return im;
}
const carBasic = loadImg("/assets/ioniq5_basic.png");
const carBrake = loadImg("/assets/ioniq5_brake.png");

// ── Lab logo (bottom-right watermark) ───────────────────────────
const avlabLogo = loadImg("/assets/avlab_logo.png");

// ── Indicator icons (alpha silhouettes, tinted when drawn) ──────
const icoManual = loadImg("/assets/indicator_manual.png");
const icoSteer = loadImg("/assets/indicator_steering.png");
const icoBrake = loadImg("/assets/indicator_brake.png");
const icoAccel = loadImg("/assets/indicator_accel.png");
const icoParking = loadImg("/assets/indicator_parking_brake.png");

// Tint an icon with source-in compositing (cached per icon and color).
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

// ── Car and gear display constants ──────────────────────────────
const CAR_W = 150;                    // car width (design px)
const CAR_BASE_Y = DESIGN_H * 0.64;   // car top y
const CAR_LIFT = -200;                // upward offset in R (negative = up)
const CAR_HOME_EPS = 2;               // offset treated as back home (the road reappears)
let carLift = 0;                      // current offset (eased)

// R: parking guide color
const GUIDE_COL = "rgba(255,236,120,0.85)";
// R: reverse lights at the rear of the car
const REV_LIGHT = { w: 0.04, h: 0.03, y: 0.6, dx: 0.19, color: "#fffdf5", glow: 16 };
// P: parking brake indicator (bottom left, red)
const PARK_ICON = { x: 5, y: DESIGN_H - 40, size: 40 };

function carHeight() {
  const img = cur.brake_on ? carBrake : carBasic;
  return (img.ready && img.naturalWidth)
    ? CAR_W * (img.naturalHeight / img.naturalWidth) : CAR_W * 0.55;
}

// ── SSE ─────────────────────────────────────────────────────────────────────
function connect() {
  const es = new EventSource("/stream");
  es.onopen = () => { statusEl.textContent = "Connected"; statusEl.className = "online"; };
  es.onmessage = (e) => { try { target = JSON.parse(e.data); } catch (_) {} };
  es.onerror = () => { statusEl.textContent = "Disconnected - reconnecting..."; statusEl.className = "offline"; };
}
connect();

// ── Canvas sizing + 8:3 letterbox fit ───────────────────────────
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

// ── Drawing helpers (1600x600 design coordinates) ───────────────
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

// Main arc (top, 230°) and bottom arc (90°) share one circle with 20° gaps.
//  main: 155°→385° (=25°), bottom: 135°→45° (anticlockwise)
const MAIN_A0 = 155, MAIN_A1 = 385, MAIN_MID = 270;
const BOT_A0 = 135, BOT_A1 = 45;
function gauge(cx, cy, R, o) {
  // gauge face
  ctx.beginPath();
  ctx.arc(cx, cy, R * 1.04, 0, Math.PI * 2);
  ctx.fillStyle = COL.face;
  ctx.fill();

  // Arcs are inset by half their width so their outer edge meets the face (R*1.04).
  const MAIN_W = R * 0.085;
  const Rmain = R * 1.04 - MAIN_W / 2;
  const BOT_W = R * 0.024;
  const Rbot = R * 1.04 - BOT_W / 2;
  const ringCol = o.ringColor || COL.ring;   // top track color

  // background tracks
  arc(cx, cy, Rmain, MAIN_A0, MAIN_A1, MAIN_W, ringCol);
  arc(cx, cy, Rbot, BOT_A0, BOT_A1, BOT_W, COL.ring, true);

  // main value (top arc)
  if (o.bidir) {
    // mirror=true fills positive values to the left; the number is unchanged.
    const dir = o.mirror ? -1 : 1;
    const ang = MAIN_MID + dir * clamp(o.value / o.max, -1, 1) * ((MAIN_A1 - MAIN_A0) / 2);
    arc(cx, cy, Rmain, Math.min(MAIN_MID, ang), Math.max(MAIN_MID, ang), R * 0.078, o.color);
  } else {
    const f = clamp(o.value / o.max, 0, 1);
    if (f > 0) arc(cx, cy, Rmain, MAIN_A0, MAIN_A0 + (MAIN_A1 - MAIN_A0) * f, R * 0.078, o.color);
  }
  // bottom value (accel/brake)
  const bf = clamp(o.bottomValue / o.bottomMax, 0, 1);
  if (bf > 0) arc(cx, cy, Rbot, BOT_A0, BOT_A0 - (BOT_A0 - BOT_A1) * bf, BOT_W, COL.white, true);

  // center value
  text(o.big, cx, cy - R * 0.05, R * 0.3, COL.text, "center", "350");
  text(o.unit, cx, cy + R * 0.20, R * 0.1, COL.sub);
  // ACCEL/BRAKE labels along the bottom arc
  text(o.bottomLabel, cx, cy + R * 0.75, R * 0.10, COL.sub);
  text(o.bottomMinLabel, cx - R * 0.55, cy + R * 0.65, R * 0.095, COL.sub);
  text(o.bottomMaxLabel, cx + R * 0.55, cy + R * 0.65, R * 0.095, COL.sub);
}

// Center: road that bends with the steering angle (t^2), plus the car.
//   The near end stays at the car center; only the far end bends.
//
// ── Road geometry ─────────────────────────────────────────────────────────
//   halfTop/halfBot are capped at GRAY_HALF; the road is clipped to cx±GRAY_HALF.
const GRAY_HALF = 155;   // half width of the indicator divider (road width limit)
const ROAD = {
  halfTop: 60,     // far half width
  halfBot: 120,    // near half width (capped at GRAY_HALF)
  topY: 230,       // top y
  botY: 540,       // bottom y
  topBend: 190,    // bend at the far end
};
function road() {
  const cx = DESIGN_W / 2, topY = ROAD.topY, botY = ROAD.botY;
  // cap the half widths at the divider width
  const halfBot = Math.min(ROAD.halfBot, GRAY_HALF);
  const halfTop = Math.min(ROAD.halfTop, GRAY_HALF);
  // Positive angles bend the road to the left (dir > 0 bends it to the right, +x).
  const dir = clamp(cur.steer_deg / cur.steer_limit, -1, 1) * -1;
  const topBend = dir * ROAD.topBend;    // only the far end moves
  const N = 28;

  // center: cx at t=0 (near), cx + topBend at t=1 (far)
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
  // clip to the divider width
  ctx.rect(cx - GRAY_HALF, topY - 12, GRAY_HALF * 2, botY - topY + 60);
  ctx.clip();

  // In R only the parking guides are drawn; the road returns once the car is back home.
  if (cur.gear === "R") {
    reverseGuides(cx);                       // guide lines behind the car
  } else if (carLift > -CAR_HOME_EPS) {
    // road surface
    ctx.beginPath();
    ctx.moveTo(L[0][0], L[0][1]);
    for (const p of L) ctx.lineTo(p[0], p[1]);
    for (let i = Rr.length - 1; i >= 0; i--) ctx.lineTo(Rr[i][0], Rr[i][1]);
    ctx.closePath();
    ctx.fillStyle = COL.road;
    ctx.fill();
    // lane lines
    strokePath(L, COL.lane, 3);
    strokePath(Rr, COL.lane, 3);
  }
  ctx.restore();

  drawCar(cx);
}

// R: parking guides from the rear of the car, bending with the steering.
function reverseGuides(cx) {
  const y0 = CAR_BASE_Y + carLift + carHeight() + 6;  // start just behind the car
  const y1 = 540;
  const halfTop = 34, halfBot = Math.min(ROAD.halfBot, GRAY_HALF);  // car width -> near width
  const bend = clamp(cur.steer_deg / cur.steer_limit, -1, 1) * -120;  // same direction as road()
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
  // distance bands
  for (const t of [0.33, 0.62, 0.9]) {
    const y = y0 + (y1 - y0) * t, c = center(t), hw = halfW(t);
    strokePath([[c - hw, y], [c + hw, y]], GUIDE_COL, 3);
  }
}

function drawCar(cx) {
  const img = cur.brake_on ? carBrake : carBasic;
  const cw = CAR_W;                     // fixed width; height keeps the aspect ratio
  const y = CAR_BASE_Y + carLift;       // raised in R
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

// R: reverse lights (see REV_LIGHT)
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

// P: parking brake indicator (bottom left, red)
function parkingIndicator() {
  const t = tinted(icoParking, COL.indRed);
  if (t) ctx.drawImage(t, PARK_ICON.x, PARK_ICON.y, PARK_ICON.size, PARK_ICON.size);
}

// Bottom bar: ACCESS on the left while the bus is connected, gear on the right.
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

  // left: ACCESS while the bus is connected
  if (cur.connected) {
    text("ACCESS", x + 30, cy, 22, COL.access, "left", "700");
  }
  // right: gear (input.py -> gear_link -> snapshot.gear)
  text(cur.gear, x + w - 30, cy, 28, COL.text, "right", "700");
}

// Top indicators (MANUAL, STEER, BRAKE, ACCEL) and a divider.
//  steer/brake: control = green, fault = red, otherwise white
//  accel: control = green, otherwise white
//  manual: actuators under control 0 = green / 1~2 = orange / 3 = white
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

  // divider below the icons
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

// Lab logo (AVLAB), drawn last
function logo() {
  if (!avlabLogo.ready || !avlabLogo.naturalWidth) return;
  const w = 130;
  const h = w * (avlabLogo.naturalHeight / avlabLogo.naturalWidth);
  const x = DESIGN_W - w;   // right edge
  const y = DESIGN_H - h;   // bottom edge
  ctx.save();
  ctx.globalAlpha = 0.7;
  ctx.drawImage(avlabLogo, x, y, w, h);
  ctx.restore();
}

// ── Render loop ─────────────────────────────────────────────────
function smooth() {
  const t = 0.18;
  cur.speed = lerp(cur.speed, target.speed, t);
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
  // raise the car in R, lower it otherwise
  carLift = lerp(carLift, cur.gear === "R" ? CAR_LIFT : 0, 0.1);
}

function frame() {
  smooth();

  // clear (including the letterbox area)
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = "#000";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // fit to the 8:3 design coordinates
  const s = fit.scale * fit.dpr;
  ctx.setTransform(s, 0, 0, s, fit.offX * fit.dpr, fit.offY * fit.dpr);

  ctx.fillStyle = COL.bg;
  ctx.fillRect(0, 0, DESIGN_W, DESIGN_H);

  const R = 258;
  const cy = 296;
  const lx = 336, rx = DESIGN_W - 336;

  // left: speed + accel
  gauge(lx, cy, R, {
    value: cur.speed, max: cur.speed_max, ringColor: COL.ringL,
    big: Math.round(cur.speed).toString(), unit: "km/h", color: COL.speed,
    bottomValue: cur.accel_pct, bottomMax: 100,
    bottomLabel: `ACCEL ${cur.accel_pct.toFixed(0)}%`,
    bottomMinLabel: "0", bottomMaxLabel: "100",
  });
  // right: steering (bidirectional) + brake
  gauge(rx, cy, R, {
    value: cur.steer_deg, max: cur.steer_limit, bidir: true, mirror: true, ringColor: COL.ringR,
    big: `${cur.steer_deg >= 0 ? "+" : ""}${cur.steer_deg.toFixed(0)}°`,
    unit: "deg", color: COL.steer,
    bottomValue: cur.brake_mm, bottomMax: cur.brake_max_mm,
    bottomLabel: `BRAKE ${cur.brake_mm.toFixed(1)}mm`,
    bottomMinLabel: "0", bottomMaxLabel: `${Math.round(cur.brake_max_mm)}`,
  });

  // center
  road();
  bottomBar();
  indicators();
  if (cur.gear === "P") parkingIndicator();   // P: parking brake indicator
  logo();

  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
