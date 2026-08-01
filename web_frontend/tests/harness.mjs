// Headless browser stub for the renderer tests.
//
// app.js is a browser module: it grabs a canvas at import time, draws every
// frame, and talks to the simulator over fetch. To test what it actually PUTS
// ON SCREEN, rather than a re-implementation of it, this file supplies just
// enough of the DOM for the real module to load and run under Node, and swaps
// the 2D context for one that records every drawing call.
//
// Three things are deliberately fake and controllable:
//   - the clock, so a test can step frames and polls without waiting;
//   - requestAnimationFrame, so the test decides when a frame happens;
//   - nothing else. fetch goes to a REAL running traffic_simulator, so the
//     data under test is the data the browser would get.

"use strict";

// ---------------------------------------------------------------------------
// Clock: monotonic, advanced only by the test.
// ---------------------------------------------------------------------------
export const clock = {
  t: 1000,                      // start past 0 so "page just loaded" guards show up
  advance(ms) { this.t += ms; },
};

// ---------------------------------------------------------------------------
// Recording 2D context
// ---------------------------------------------------------------------------
// Records one entry per stroke()/fill()/drawImage(), carrying the style in
// force and the path built since the last beginPath(). That is enough to ask
// "how many road segments were highlighted in the route colour" without
// knowing anything about how app.js structures its drawing.
export function makeRecorder(width, height) {
  const calls = [];
  let path = [];
  const ctx = {
    canvas: null,
    strokeStyle: "#000", fillStyle: "#000", lineWidth: 1, lineCap: "butt",
    globalAlpha: 1, font: "", textAlign: "", textBaseline: "",
    shadowColor: "", shadowBlur: 0,

    beginPath() { path = []; },
    moveTo(x, y) { path.push(["M", x, y]); },
    lineTo(x, y) { path.push(["L", x, y]); },
    closePath() { path.push(["Z"]); },
    arc(x, y, r) { path.push(["A", x, y, r]); },
    ellipse(x, y, rx, ry) { path.push(["E", x, y, rx, ry]); },
    rect(x, y, w, h) { path.push(["R", x, y, w, h]); },
    quadraticCurveTo() {}, bezierCurveTo() {},

    stroke() {
      calls.push({ op: "stroke", style: this.strokeStyle, lineWidth: this.lineWidth,
                   alpha: this.globalAlpha, path: path.slice() });
    },
    fill() {
      calls.push({ op: "fill", style: this.fillStyle, alpha: this.globalAlpha,
                   path: path.slice() });
    },
    fillRect(x, y, w, h) {
      calls.push({ op: "fillRect", style: this.fillStyle, alpha: this.globalAlpha,
                   x, y, w, h });
    },
    strokeRect() {},
    clearRect() { calls.push({ op: "clearRect" }); },
    fillText(text, x, y) { calls.push({ op: "text", text, x, y }); },
    drawImage(img, ...rest) {
      calls.push({ op: "drawImage", img, args: rest, alpha: this.globalAlpha });
    },
    save() {}, restore() {},
    translate() {}, rotate() {}, scale() {},
    setTransform(...a) { calls.push({ op: "setTransform", args: a }); },
    resetTransform() {},
    setLineDash() {}, getLineDash() { return []; },
    createPattern() { return { setTransform() {} }; },
    createRadialGradient() {
      // Counted: this is the per-prop shadow whose cost the scenery rule exists
      // to avoid, so a test can assert it is not happening.
      calls.push({ op: "radialGradient" });
      return { addColorStop() {} };
    },
    createLinearGradient() { return { addColorStop() {} }; },
    measureText(t) { return { width: (t || "").length * 6 }; },
  };
  ctx.__calls = calls;
  ctx.__reset = () => { calls.length = 0; };
  ctx.__w = width; ctx.__h = height;
  return ctx;
}

// ---------------------------------------------------------------------------
// Minimal element. Any id resolves; nothing is ever null, because the tests
// care about the renderer, not about which chrome the page happens to carry.
// ---------------------------------------------------------------------------
function makeElement(tag, id) {
  const classes = new Set();
  const el = {
    tagName: (tag || "div").toUpperCase(),
    id: id || "",
    style: {}, dataset: {},
    value: "", textContent: "", innerHTML: "", checked: false, disabled: false,
    clientWidth: 1200, clientHeight: 600, width: 0, height: 0,
    children: [], parentNode: null,
    __handlers: new Map(),

    classList: {
      add: (...c) => c.forEach((x) => classes.add(x)),
      remove: (...c) => c.forEach((x) => classes.delete(x)),
      toggle: (c, on) => {
        const want = on === undefined ? !classes.has(c) : !!on;
        want ? classes.add(c) : classes.delete(c);
        return want;
      },
      contains: (c) => classes.has(c),
    },
    addEventListener(type, fn) {
      if (!this.__handlers.has(type)) this.__handlers.set(type, []);
      this.__handlers.get(type).push(fn);
    },
    removeEventListener() {},
    dispatch(type, ev = {}) {
      for (const fn of this.__handlers.get(type) || []) fn(ev);
    },
    getBoundingClientRect() {
      return { left: 0, top: 0, width: this.clientWidth, height: this.clientHeight,
               right: this.clientWidth, bottom: this.clientHeight };
    },
    getContext() {
      if (!this.__ctx) { this.__ctx = makeRecorder(this.width, this.height); this.__ctx.canvas = this; }
      return this.__ctx;
    },
    setPointerCapture() {}, releasePointerCapture() {}, hasPointerCapture() { return false; },
    appendChild(c) { this.children.push(c); c.parentNode = this; return c; },
    removeChild(c) { this.children = this.children.filter((x) => x !== c); return c; },
    prepend(c) { this.children.unshift(c); c.parentNode = this; return c; },
    remove() {},
    querySelector() { return null; },
    querySelectorAll() { return []; },
    closest() { return null; },
    focus() {}, blur() {}, click() {},
    get lastChild() { return this.children[this.children.length - 1] || null; },
    get firstChild() { return this.children[0] || null; },
  };
  return el;
}

// ---------------------------------------------------------------------------
// install(): put the fakes on globalThis, then hand back the handles a test
// needs. Call before importing app.js.
// ---------------------------------------------------------------------------
export function install({ origin, canvasW = 2400, canvasH = 1200 } = {}) {
  const byId = new Map();
  const getEl = (id) => {
    if (!byId.has(id)) byId.set(id, makeElement("div", id));
    return byId.get(id);
  };

  const mapCanvas = getEl("map");
  mapCanvas.tagName = "CANVAS";
  mapCanvas.clientWidth = canvasW / 2;      // dpr 2
  mapCanvas.clientHeight = canvasH / 2;

  const frames = [];                         // queued rAF callbacks

  globalThis.document = {
    getElementById: getEl,
    createElement: (tag) => makeElement(tag),
    querySelector: () => null,
    querySelectorAll: () => [],
    addEventListener: () => {},
    body: makeElement("body"),
    documentElement: makeElement("html"),
  };
  globalThis.window = {
    devicePixelRatio: 2,
    innerWidth: canvasW / 2, innerHeight: canvasH / 2,
    addEventListener: (type, fn) => {
      if (!globalThis.window.__handlers[type]) globalThis.window.__handlers[type] = [];
      globalThis.window.__handlers[type].push(fn);
    },
    __handlers: {},
    location: { origin, href: origin + "/" },
  };
  // Only now() is faked. Node's own fetch reaches into performance for resource
  // timing, so replacing the whole object breaks every request.
  const realPerf = globalThis.performance;
  globalThis.performance = new Proxy(realPerf, {
    get(target, prop) {
      if (prop === "now") return () => clock.t;
      const v = Reflect.get(target, prop);
      return typeof v === "function" ? v.bind(target) : v;
    },
  });
  globalThis.requestAnimationFrame = (fn) => { frames.push(fn); return frames.length; };
  globalThis.cancelAnimationFrame = () => {};
  globalThis.DOMMatrix = class { constructor(a) { this.a = a; } };
  globalThis.Image = class {
    constructor() { this.naturalWidth = 0; this.naturalHeight = 0; }
    set src(_v) { setTimeout(() => this.onerror && this.onerror(), 0); }
  };

  // fetch: relative URLs go to the live simulator.
  const realFetch = globalThis.fetch;
  globalThis.fetch = (url, opts) =>
    realFetch(url.startsWith("http") ? url : origin + url, opts);

  return {
    canvas: mapCanvas,
    ctx: mapCanvas.getContext("2d"),
    el: getEl,
    // Run exactly one animation frame: drain the queue once.
    frame(dtMs = 16) {
      clock.advance(dtMs);
      const due = frames.splice(0, frames.length);
      for (const fn of due) fn(clock.t);
    },
    pendingFrames: () => frames.length,
    windowEvent(type, ev = {}) {
      for (const fn of globalThis.window.__handlers[type] || []) fn(ev);
    },
  };
}

// ---------------------------------------------------------------------------
// Tiny assertion helpers
// ---------------------------------------------------------------------------
let passed = 0;
const failures = [];

export function check(name, cond, detail = "") {
  if (cond) { passed++; console.log(`  ok   ${name}${detail ? "  — " + detail : ""}`); }
  else { failures.push(name + (detail ? "  — " + detail : "")); console.log(`  FAIL ${name}${detail ? "  — " + detail : ""}`); }
}

export function section(title) { console.log(`\n${title}`); }

export function report() {
  console.log(`\n${passed} passed, ${failures.length} failed`);
  for (const f of failures) console.log(`  FAILED: ${f}`);
  return failures.length === 0;
}

export const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
