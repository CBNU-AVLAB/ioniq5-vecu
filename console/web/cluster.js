/**
 * @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      cluster.js
 * @brief     IONIQ5 ccNC-style cluster renderer (display only)
 *
 * @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
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
    const ang = MAIN_MID + clamp(o.value / o.max, -1, 1) * ((MAIN_A1 - MAIN_A0) / 2);
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
function road() {
  const cx = DESIGN_W / 2, topY = 128, botY = 548;
  const halfBot = 150, halfTop = 12;
  const bend = clamp(cur.steer_deg / cur.steer_limit, -1, 1) * 190;
  const N = 28;

  const center = (t) => cx + bend * t * t;             // curve centerline (straight near -> bends far)
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
  ctx.rect(590, topY - 12, DESIGN_W - 590 * 2, botY - topY + 60);
  ctx.clip();

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
  ctx.restore();

  drawCar(cx);
}

function drawCar(cx) {
  const img = cur.brake_on ? carBrake : carBasic;
  const cw = 150;                       // width kept moderate (keep aspect ratio, avoid squashing)
  const y = DESIGN_H * 0.65;            // start ~1/4 up from the bottom
  // keep image natural aspect (fit width, height follows ratio)
  let ch = cw * 0.55;
  if (img.ready && img.naturalWidth) ch = cw * (img.naturalHeight / img.naturalWidth);
  const x = cx - cw / 2;
  if (img.ready) {
    ctx.drawImage(img, x, y, cw, ch);
  } else {
    roundRect(x, y, cw, ch, 12);
    ctx.fillStyle = "#9aa0aa"; ctx.fill();
    ctx.fillStyle = cur.brake_on ? COL.brake : "#5b6068";
    ctx.fillRect(x + cw * 0.12, y + ch * 0.22, cw * 0.76, ch * 0.16);
  }
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
  // right: gear
  text("D", x + w - 30, cy, 28, COL.text, "right", "700");
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
    value: cur.steer_deg, max: cur.steer_limit, bidir: true, ringColor: COL.ringR,
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
  logo();

  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
