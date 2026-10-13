/* B.U.D.-E schematic viewer: orbit camera, painter's-algorithm renderer, UI wiring.
 *
 * No third-party code. The renderer is deliberately simple so the page works offline and
 * nothing can be swapped under you. It is an APPROXIMATION for looking at a design, not a
 * CAD or physics tool, and it draws DECLARED geometry only.
 */
'use strict';

(function () {
  const canvas = document.getElementById('view');
  const ctx = canvas.getContext('2d');
  const statusEl = document.getElementById('status');
  const $ = (id) => document.getElementById(id);

  const state = {
    q: MODEL.HOME.slice(),
    gripper: 0.7,                 // 1 = open, 0 = closed
    plan: null,
    speed: 1.0,
    rover: { on: false, visible: false, x: -140, y: -170, heading: 0.7 },
    trace: null,
    mesh: null,
    meshName: null,
    showMesh: false,
    sight: null,
    message: 'loading model\u2026',
    sliderEls: []
  };

  const view = { az: -1.15, el: 0.42, dist: 1250, target: [140, 0, 80], fov: 42 };

  // ------------------------------------------------------------------ projection ---
  function resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.max(320, canvas.clientWidth * dpr);
    canvas.height = Math.max(240, canvas.clientHeight * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  window.addEventListener('resize', resize);

  let CAM = null;
  function camera() {
    const ce = Math.cos(view.el), se = Math.sin(view.el);
    const eye = [view.target[0] + view.dist * ce * Math.cos(view.az),
      view.target[1] + view.dist * ce * Math.sin(view.az),
      view.target[2] + view.dist * se];
    const f = [view.target[0] - eye[0], view.target[1] - eye[1], view.target[2] - eye[2]];
    const fl = Math.hypot(f[0], f[1], f[2]) || 1;
    const fwd = [f[0] / fl, f[1] / fl, f[2] / fl];
    // right = normalize(cross(fwd, worldUp)) with worldUp = (0, 0, 1)
    let right = [fwd[1], -fwd[0], 0];
    const rl = Math.hypot(right[0], right[1], right[2]) || 1;
    right = [right[0] / rl, right[1] / rl, right[2] / rl];
    // up = cross(right, fwd): no sign flip, or the image renders upside down.
    const up = [right[1] * fwd[2] - right[2] * fwd[1],
      right[2] * fwd[0] - right[0] * fwd[2],
      right[0] * fwd[1] - right[1] * fwd[0]];
    return { eye, fwd, right, up };
  }

  function project(p) {
    const v = [p[0] - CAM.eye[0], p[1] - CAM.eye[1], p[2] - CAM.eye[2]];
    const zc = v[0] * CAM.fwd[0] + v[1] * CAM.fwd[1] + v[2] * CAM.fwd[2];
    if (zc <= 1) return null;
    const xc = v[0] * CAM.right[0] + v[1] * CAM.right[1] + v[2] * CAM.right[2];
    const yc = v[0] * CAM.up[0] + v[1] * CAM.up[1] + v[2] * CAM.up[2];
    const fl = (0.5 * canvas.clientHeight) / Math.tan((view.fov * Math.PI) / 360);
    const s = fl / zc;
    return [canvas.clientWidth / 2 + xc * s, canvas.clientHeight / 2 - yc * s, zc];
  }
  function depthOf(p) {
    const v = [p[0] - CAM.eye[0], p[1] - CAM.eye[1], p[2] - CAM.eye[2]];
    return v[0] * CAM.fwd[0] + v[1] * CAM.fwd[1] + v[2] * CAM.fwd[2];
  }

  // --------------------------------------------------------------------- drawing ---
  function queue(list, point, paint) {
    const s = project(point);
    if (!s) return;
    list.push({ d: depthOf(point), paint: () => paint(s) });
  }
  function line(list, a, b, color, width) {
    queue(list, [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2], (sa) => {
      const sb = project(b);
      if (!sb) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(sa[0], sa[1]);
      ctx.lineTo(sb[0], sb[1]);
      ctx.stroke();
    });
  }
  function poly(list, pts, fill, stroke) {
    const c = pts.reduce((m, p) => [m[0] + p[0] / pts.length, m[1] + p[1] / pts.length,
      m[2] + p[2] / pts.length], [0, 0, 0]);
    queue(list, c, () => {
      const scr = pts.map(project);
      if (scr.some((s) => !s)) return;
      ctx.beginPath();
      ctx.moveTo(scr[0][0], scr[0][1]);
      for (let i = 1; i < scr.length; i++) ctx.lineTo(scr[i][0], scr[i][1]);
      ctx.closePath();
      ctx.fillStyle = fill;
      ctx.fill();
      if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke(); }
    });
  }
  /** Vertical cylinder: correct for the base, the palm and a standing cup. */
  function cylinder(list, cx, cy, z0, z1, r, color, seg) {
    seg = seg || 18;
    for (let i = 0; i < seg; i++) {
      const a0 = (i / seg) * 2 * Math.PI, a1 = ((i + 1) / seg) * 2 * Math.PI;
      const p0 = [cx + r * Math.cos(a0), cy + r * Math.sin(a0), z0];
      const p1 = [cx + r * Math.cos(a1), cy + r * Math.sin(a1), z0];
      const p2 = [cx + r * Math.cos(a1), cy + r * Math.sin(a1), z1];
      const p3 = [cx + r * Math.cos(a0), cy + r * Math.sin(a0), z1];
      poly(list, [p0, p1, p2, p3], color, '#00000030');
      poly(list, [[cx, cy, z1], p3, p2], color, null);
    }
  }
  /** Wheel: a disc whose axis runs along +Y, which is how the printed wheels must face. */
  function wheel(list, cx, cy, cz, r, thickness, color, seg) {
    seg = seg || 20;
    const h = thickness / 2;
    for (let i = 0; i < seg; i++) {
      const a0 = (i / seg) * 2 * Math.PI, a1 = ((i + 1) / seg) * 2 * Math.PI;
      const x0 = cx + r * Math.cos(a0), z0 = cz + r * Math.sin(a0);
      const x1 = cx + r * Math.cos(a1), z1 = cz + r * Math.sin(a1);
      poly(list, [[x0, cy - h, z0], [x1, cy - h, z1], [x1, cy + h, z1], [x0, cy + h, z0]],
        color, '#00000030');
      poly(list, [[cx, cy - h, cz], [x0, cy - h, z0], [x1, cy - h, z1]], color, null);
      poly(list, [[cx, cy + h, cz], [x1, cy + h, z1], [x0, cy + h, z0]], color, null);
    }
  }
  function box(list, cx, cy, cz, sx, sy, sz, color) {
    const x0 = cx - sx / 2, x1 = cx + sx / 2, y0 = cy - sy / 2, y1 = cy + sy / 2;
    const z0 = cz - sz / 2, z1 = cz + sz / 2;
    const c = (x, y, z) => [x, y, z];
    [[c(x0, y0, z0), c(x1, y0, z0), c(x1, y1, z0), c(x0, y1, z0)],
    [c(x0, y0, z1), c(x1, y0, z1), c(x1, y1, z1), c(x0, y1, z1)],
    [c(x0, y0, z0), c(x1, y0, z0), c(x1, y0, z1), c(x0, y0, z1)],
    [c(x1, y0, z0), c(x1, y1, z0), c(x1, y1, z1), c(x1, y0, z1)],
    [c(x1, y1, z0), c(x0, y1, z0), c(x0, y1, z1), c(x1, y1, z1)],
    [c(x0, y1, z0), c(x0, y0, z0), c(x0, y0, z1), c(x0, y1, z1)]]
      .forEach((f) => poly(list, f, color, '#00000045'));
  }

  // ---------------------------------------------------------------- scene build ----
  function grid(list) {
    const N = 6, STEP = 100;
    for (let i = -N; i <= N; i++) {
      line(list, [i * STEP, -N * STEP, 0], [i * STEP, N * STEP, 0], '#1c2530', 1);
      line(list, [-N * STEP, i * STEP, 0], [N * STEP, i * STEP, 0], '#1c2530', 1);
    }
    line(list, [0, 0, 0], [0, 0, 300], '#2f3b48', 1.5);
  }

  function armScene(list, origin, q, gripper) {
    const pts = fk(q);
    const off = (p) => [p[0] + origin[0], p[1] + origin[1], p[2] + origin[2]];
    const base = off(pts.base), sh = off(pts.shoulder), el = off(pts.elbow);
    const wr = off(pts.wrist), tcp = off(pts.tcp);

    cylinder(list, origin[0], origin[1], origin[2], origin[2] + 14, 46, '#38475a', 24);
    cylinder(list, origin[0], origin[1], origin[2] + 14, origin[2] + 26, 32, '#46586b', 20);

    line(list, base, sh, '#6f8398', 24);
    line(list, sh, el, '#8fa3b6', 16);
    line(list, el, wr, '#8fa3b6', 13);
    line(list, wr, tcp, '#b9c6d2', 11);
    [sh, el, wr].forEach((p) => queue(list, p, (s) => {
      ctx.fillStyle = '#5cc2f2';
      ctx.beginPath();
      ctx.arc(s[0], s[1], 6, 0, 2 * Math.PI);
      ctx.fill();
    }));

    // Claw fingers, opening with the slider, oriented by the TCP frame.
    const fr = tcpFrame(pts);
    const w = 9 + gripper * 22;
    [-1, 1].forEach((s) => {
      const tip = [tcp[0] + fr.y[0] * s * w, tcp[1] + fr.y[1] * s * w, tcp[2] + fr.y[2] * s * w];
      const mid = [wr[0] + (tip[0] - wr[0]) * 0.72, wr[1] + (tip[1] - wr[1]) * 0.72,
      wr[2] + (tip[2] - wr[2]) * 0.72];
      line(list, wr, mid, '#c8d3de', 6);
      line(list, mid, tip, '#c8d3de', 5);
    });

    // Wrist camera at the declared mount offset, with a frustum wireframe.
    const cam = [0, 1, 2].map((i) => tcp[i] + fr.x[i] * MODEL.CAM_OFFSET[0]
      + fr.y[i] * MODEL.CAM_OFFSET[1] + fr.z[i] * MODEL.CAM_OFFSET[2]);
    box(list, cam[0], cam[1], cam[2], 16, 16, 12, '#2f4a5c');
    const reach = 300, half = Math.tan((MODEL.CAM_HFOV_DEG * Math.PI) / 360) * reach;
    const corners = [[1, 1], [1, -1], [-1, -1], [-1, 1]].map((s) => [0, 1, 2].map((i) =>
      cam[i] + fr.z[i] * reach + (fr.x[i] * s[0] + fr.y[i] * s[1]) * half));
    corners.forEach((c) => line(list, cam, c, '#2b6d8c', 1));
    for (let i = 0; i < 4; i++) line(list, corners[i], corners[(i + 1) % 4], '#2b6d8c', 1);
  }

  function targetScene(list) {
    (window.SCENE_TARGETS || []).forEach((t) => {
      const c = `rgb(${t.rgb[0]},${t.rgb[1]},${t.rgb[2]})`;
      if (t.name === 'cup') {
        cylinder(list, t.pos[0], t.pos[1], t.pos[2], t.pos[2] + t.len, 22, c, 16);
      } else if (t.name === 'screwdriver') {
        // A screwdriver lies on the desk at an angle, which is why the wrist roll matters.
        const a = 20 * Math.PI / 180;
        const half = t.len / 2;
        line(list, [t.pos[0] - half * Math.cos(a), t.pos[1] - half * Math.sin(a), t.pos[2] + 5],
          [t.pos[0] + half * Math.cos(a), t.pos[1] + half * Math.sin(a), t.pos[2] + 5], c, 9);
      } else {
        box(list, t.pos[0], t.pos[1], t.pos[2] + 4, 22, 14, 8, c);
      }
    });
  }

  function roverScene(list) {
    const r = state.rover;
    const deckZ = MODEL.DECK_TOP_Z - MODEL.GROUND_GAP;      // chassis underside height
    const rot = (dx, dy) => [r.x + dx * Math.cos(r.heading) - dy * Math.sin(r.heading),
    r.y + dx * Math.sin(r.heading) + dy * Math.cos(r.heading)];
    const corner = (dx, dy, z) => { const p = rot(dx, dy); return [p[0], p[1], z]; };
    poly(list, [corner(73, 52, deckZ + 5), corner(73, -52, deckZ + 5),
    corner(-73, -52, deckZ + 5), corner(-73, 52, deckZ + 5)], '#41505f', '#00000055');
    [[73, 52], [73, -52], [-73, -52], [-73, 52]].forEach((p, i, arr) => {
      line(list, corner(p[0], p[1], deckZ), corner(p[0], p[1], deckZ + 5), '#39485a', 2);
    });
    // Wheels: axis along the rover's local Y, so draw them as Y-axis discs.
    [[0, -58], [0, 58]].forEach((s) => {
      const w = rot(s[0], s[1]);
      const c = Math.cos(r.heading), sn = Math.sin(r.heading);
      // Rotate the wheel's local offset into world axes for the disc helper.
      const cx = w[0], cy = w[1], cz = MODEL.AXLE_Z;
      const rw = MODEL.WHEEL_D / 2;
      const ux = [c, sn], uy = [-sn, c];
      const pts = (offset) => {
        const out = [];
        for (let i = 0; i < 20; i++) {
          const a = (i / 20) * 2 * Math.PI;
          const lx = rw * Math.cos(a), lz = rw * Math.sin(a);
          out.push([cx + lx * ux[0] + offset * uy[0], cy + lx * ux[1] + offset * uy[1], cz + lz]);
        }
        return out;
      };
      poly(list, pts(-3), '#20262e', '#00000055');
      poly(list, pts(3), '#20262e', '#00000055');
      for (let i = 0; i < 20; i++) {
        const a = pts(-3), b = pts(3);
        poly(list, [a[i], a[(i + 1) % 20], b[(i + 1) % 20], b[i]], '#191f26', null);
      }
    });
    // Heading arrow on the deck.
    line(list, corner(-40, 0, deckZ + 6), corner(70, 0, deckZ + 6), '#5cc2f2', 2);
  }

  function meshScene(list) {
    if (!state.mesh || !state.showMesh) return;
    const m = state.mesh, light = [0.35, -0.5, 0.8];
    m.tris.forEach((tri) => {
      const a = [0, 1, 2].map((i) => tri[0][i] * m.scale + m.origin[i]);
      const b = [0, 1, 2].map((i) => tri[1][i] * m.scale + m.origin[i]);
      const c = [0, 1, 2].map((i) => tri[2][i] * m.scale + m.origin[i]);
      const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
      const v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
      let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
      const nl = Math.hypot(n[0], n[1], n[2]) || 1;
      n = [n[0] / nl, n[1] / nl, n[2] / nl];
      const lam = Math.abs(n[0] * light[0] + n[1] * light[1] + n[2] * light[2]);
      const s = Math.round(96 + 120 * lam);
      poly(list, [a, b, c], `rgb(${s},${Math.round(s * 0.92)},${Math.round(s * 0.78)})`, '#00000040');
    });
  }

  // ---------------------------------------------------------------------- render ---
  function draw() {
    CAM = camera();
    const list = [];
    grid(list);
    const roverActive = state.rover.on || state.rover.visible;
    const origin = roverActive
      ? [state.rover.x, state.rover.y, MODEL.DECK_TOP_Z] : [0, 0, 0];
    if (roverActive) roverScene(list);
    armScene(list, origin, state.q, state.gripper);
    targetScene(list);
    meshScene(list);
    if (state.sight) {
      const pts = fk(state.q);
      const tcp = [pts.tcp[0] + origin[0], pts.tcp[1] + origin[1], pts.tcp[2] + origin[2]];
      line(list, tcp, [state.sight[0], state.sight[1], 0], '#5cc2f255', 1.5);
    }
    list.sort((a, b) => b.d - a.d);
    const g = ctx.createLinearGradient(0, 0, 0, canvas.clientHeight);
    g.addColorStop(0, '#0e1116');
    g.addColorStop(1, '#151a21');
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);
    list.forEach((item) => item.paint());
    ctx.fillStyle = '#8b97a5';
    ctx.font = '11px ui-monospace,SFMono-Regular,Menlo,monospace';
    if (state.message) ctx.fillText(state.message, 12, 20);
  }

  // ------------------------------------------------------------------------ loop ---
  let last = performance.now();
  function tick(now) {
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    step(dt);
    draw();
    requestAnimationFrame(tick);
  }

  function step(dt) {
    const p = state.plan;
    if (!p) return;
    p.t += dt * state.speed;
    const n = p.samples.length;
    const idx = Math.min(n - 1, Math.max(0, (p.t / p.duration_s) * (n - 1)));
    const i0 = Math.floor(idx), i1 = Math.min(n - 1, i0 + 1), f = idx - i0;
    for (let k = 0; k < 5; k++) {
      state.q[k] = p.samples[i0][1][k] * (1 - f) + p.samples[i1][1][k] * f;
    }
    syncSliders();
    if (p.t >= p.duration_s) {
      state.plan = null;
      state.message = 'plan finished';
      status('Plan finished. A real stop needs the hardware e-stop; this button stops pixels.');
    } else {
      status(`playing ${p.t.toFixed(2)} / ${p.duration_s.toFixed(2)} s`);
    }
  }

  // ------------------------------------------------------------------- animation ---
  function startFetchSim() {
    state.message = 'asking the server for an offline drive plan\u2026';
    api('/api/fetch?x=520&y=140').then((d) => {
      if (!d.ok) { state.message = d.error || 'drive plan failed'; return; }
      state.rover.on = true;
      state.rover.visible = true;
      state.trace = d.trace;
      state.message = `offline policy: arrived=${d.arrived} in ${d.ticks} ticks`;
      status(`OFFLINE drive simulation\n${d.ticks} ticks, arrived=${d.arrived}\n` +
        `final pose (${d.final_pose[0].toFixed(0)}, ${d.final_pose[1].toFixed(0)}) mm, ` +
        `standoff error ${d.distance_to_standoff_mm.toFixed(1)} mm\n\n` + d.note);
      playTrace(0);
    }).catch((e) => { state.message = 'request failed: ' + e; });
  }

  function playTrace(i) {
    const tr = state.trace;
    if (!tr || i >= tr.length) {
      state.message = 'rover in place \u2014 planning the pick';
      planTo(185, 0, -60, 0);
      return;
    }
    const row = tr[i];
    state.rover.x = row[1];
    state.rover.y = row[2];
    const nxt = tr[Math.min(tr.length - 1, i + 8)];
    state.rover.heading = Math.atan2(nxt[2] - row[2], nxt[1] - row[1]);
    state.sight = [520, 140];
    state.message = `driving: tick ${i + 1} / ${tr.length}`;
    setTimeout(() => playTrace(i + 8), 16);
  }

  // ------------------------------------------------------------------------- net ---
  function api(url) { return fetch(url).then((r) => r.json()); }
  const readNum = (id) => $(id).value;
  function status(text) { statusEl.textContent = text; }

  function showIk(d) {
    if (!d.ok) {
      status(`REFUSED\n${d.error}\n\n${d.hint || ''}`);
      state.message = 'refused: ' + d.error;
      return;
    }
    const best = d.solutions[0];
    const badge = best.safe ? '<span class="badge ok">model OK</span>'
      : '<span class="badge bad">keep-out</span>';
    let text = `${d.solutions.length} branch(es)  ${badge}\n` +
      `TCP (${best.tcp.map((v) => v.toFixed(1)).join(', ')}) mm\n` +
      `achieved pitch ${d.achieved_pitches[0].toFixed(1)} deg ` +
      `(asked ${d.pitch_requested.toFixed(1)})\n` +
      (d.pitch_adapted ? 'PITCH ADAPTED - not the angle you asked for\n' : '') +
      `q = [${best.q.map((v) => v.toFixed(1)).join(', ')}]`;
    if (best.reasons.length) text += '\n\n' + best.reasons.join('\n');
    status(text);
    state.message = best.safe ? 'IK solved' : 'IK solved, keep-out violated';
  }

  function reloadPose() {
    api(`/api/ik?x=${readNum('ikx')}&y=${readNum('iky')}&z=${readNum('ikz')}` +
      `&pitch=${readNum('ikp')}&roll=${readNum('ikr')}` +
      `&adapt=${$('ikadapt').checked ? 1 : 0}`).then(showIk);
  }

  function planTo(x, y, pitch, roll) {
    const from = state.q.map((v) => v.toFixed(3)).join(',');
    api(`/api/plan?x=${x}&y=${y}&z=${readNum('ikz')}&pitch=${pitch}&roll=${roll}&from=${from}`)
      .then((d) => {
        if (!d.ok) {
          status(`PLAN REFUSED\n${d.error}\n${d.hint || ''}`);
          state.message = 'plan refused: ' + d.error;
          return;
        }
        state.plan = { samples: d.samples, duration_s: d.duration_s, t: 0 };
        status(`planned ${d.samples.length} samples in ${d.duration_s.toFixed(2)} s\n` +
          `min TCP z ${d.min_tcp_z_mm.toFixed(1)} mm\n` +
          `peak joint speeds ${d.peak_velocity_deg_s.map((v) => v.toFixed(1)).join(', ')} deg/s\n` +
          `limiting joint J${d.limiting_joint}\n` +
          `achieved pitch ${d.achieved_pitch.toFixed(1)} deg\n\n` + d.note);
        state.message = 'playing minimum-jerk plan';
      });
  }

  // ------------------------------------------------------------------------- stl ---
  function parseSTL(buf) {
    const dv = new DataView(buf);
    if (buf.byteLength >= 84) {
      const n = dv.getUint32(80, true);
      if (84 + 50 * n === buf.byteLength && n > 0 && n < 200000) {
        const tris = [];
        for (let i = 0; i < n; i++) {
          const o = 84 + i * 50, tri = [];
          for (let k = 0; k < 3; k++) {
            const p = o + 12 + k * 12;
            tri.push([dv.getFloat32(p, true), dv.getFloat32(p + 4, true),
            dv.getFloat32(p + 8, true)]);
          }
          tris.push(tri);
        }
        return tris;
      }
    }
    const text = new TextDecoder().decode(buf);
    const tris = [];
    const re = /vertex\s+(-?[\d.eE+]+)\s+(-?[\d.eE+]+)\s+(-?[\d.eE+]+)/g;
    let m, cur = [];
    while ((m = re.exec(text))) {
      cur.push([+m[1], +m[2], +m[3]]);
      if (cur.length === 3) { tris.push(cur); cur = []; }
    }
    return tris;
  }

  function loadPart(name) {
    if (!name) return;
    fetch('/api/stl/' + name).then((r) => r.arrayBuffer()).then((buf) => {
      const tris = parseSTL(buf);
      if (!tris.length) { $('partInfo').textContent = 'no triangles parsed'; return; }
      let mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
      tris.forEach((t) => t.forEach((p) => {
        for (let i = 0; i < 3; i++) { mn[i] = Math.min(mn[i], p[i]); mx[i] = Math.max(mx[i], p[i]); }
      }));
      const size = Math.max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) || 1;
      const scale = 240 / size;
      const cx = (mn[0] + mx[0]) / 2, cy = (mn[1] + mx[1]) / 2;
      state.mesh = {
        tris, scale,
        // Park the part beside the bench, sitting ON the desk, at a readable size.
        origin: [-330 - cx * scale, -cy * scale, 20 - mn[2] * scale]
      };
      state.meshName = name;
      $('partInfo').textContent =
        `${name}: ${tris.length} triangles, ` +
        `${(mx[0] - mn[0]).toFixed(1)} x ${(mx[1] - mn[1]).toFixed(1)} x ` +
        `${(mx[2] - mn[2]).toFixed(1)} mm, drawn at ${(scale * 100).toFixed(0)}%`;
    }).catch((e) => { $('partInfo').textContent = 'load failed: ' + e; });
  }

  // -------------------------------------------------------------------------- ui ---
  function buildSliders() {
    const host = $('sliders');
    const names = ['J1 base yaw', 'J2 shoulder', 'J3 elbow', 'J4 wrist pitch', 'J5 wrist roll'];
    host.innerHTML = '';
    state.sliderEls = [];
    names.forEach((name, i) => {
      const row = document.createElement('div');
      row.className = 'joint';
      const lim = MODEL.LIMITS[i];
      row.innerHTML = `<span>${name}</span>
        <input type="range" min="${lim[0]}" max="${lim[1]}" step="0.5" value="${state.q[i]}">
        <output>${state.q[i].toFixed(1)}\u00b0</output>`;
      host.appendChild(row);
      const input = row.querySelector('input'), out = row.querySelector('output');
      input.addEventListener('input', () => {
        state.q[i] = +input.value;
        state.plan = null;
        out.textContent = (+input.value).toFixed(1) + '\u00b0';
      });
      state.sliderEls.push({ input, out });
    });
  }

  function syncSliders() {
    state.sliderEls.forEach((s, i) => {
      s.input.value = state.q[i].toFixed(1);
      s.out.textContent = state.q[i].toFixed(1) + '\u00b0';
    });
  }

  function wire() {
    $('grip').addEventListener('input', (e) => {
      state.gripper = e.target.value / 100;
      $('gripOut').textContent = state.gripper > 0.8 ? 'open'
        : state.gripper < 0.25 ? 'closed' : 'part open';
    });
    document.querySelectorAll('[data-pose]').forEach((b) => b.addEventListener('click', () => {
      state.plan = null;
      state.q = (MODEL.NAMED[b.dataset.pose] || MODEL.HOME).slice();
      syncSliders();
      state.message = 'named pose: ' + b.dataset.pose;
    }));
    $('solve').addEventListener('click', reloadPose);
    $('plan').addEventListener('click', () => {
      state.sight = [+readNum('ikx'), +readNum('iky')];
      planTo(+readNum('ikx'), +readNum('iky'), +readNum('ikp'), +readNum('ikr'));
    });
    $('stop').addEventListener('click', () => {
      state.plan = null;
      state.message = 'stopped';
      status('Stopped the VIEWER. A real stop is the physical e-stop, not a button.');
    });
    $('rover').addEventListener('change', (e) => {
      state.rover.on = e.target.checked;
      if (!state.rover.on) state.rover.visible = false;
    });
    $('fetch').addEventListener('click', startFetchSim);
    $('showPart').addEventListener('change', (e) => {
      state.showMesh = e.target.checked;
      if (state.showMesh && !state.mesh && state.meshName) loadPart(state.meshName);
    });
    $('parts').addEventListener('change', (e) => {
      loadPart(e.target.value);
      state.showMesh = true;
      $('showPart').checked = true;
    });

    let dragging = false, px = 0, py = 0;
    canvas.addEventListener('pointerdown', (e) => {
      dragging = true; px = e.clientX; py = e.clientY; canvas.classList.add('drag');
    });
    window.addEventListener('pointerup', () => {
      dragging = false; canvas.classList.remove('drag');
    });
    window.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      view.az -= (e.clientX - px) * 0.006;
      view.el = Math.max(-0.25, Math.min(1.45, view.el + (e.clientY - py) * 0.005));
      px = e.clientX; py = e.clientY;
    });
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      view.dist = Math.max(450, Math.min(3200, view.dist * (1 + Math.sign(e.deltaY) * 0.09)));
    }, { passive: false });
  }

  // ------------------------------------------------------------------------ boot ---
  function boot() {
    resize();
    buildSliders();
    wire();
    api('/api/model').then((d) => {
      if (!d.ok) return;
      const L = d.link_lengths_mm;
      const drift = Math.abs(L.H - MODEL.H) + Math.abs(L.UPPER - MODEL.UPPER)
        + Math.abs(L.FOREARM - MODEL.FOREARM) + Math.abs(L.TOOL - MODEL.TOOL);
      if (drift > 1e-9) {
        MODEL.H = L.H; MODEL.UPPER = L.UPPER; MODEL.FOREARM = L.FOREARM; MODEL.TOOL = L.TOOL;
        console.warn('link lengths reloaded from the server: model.js was stale');
      }
      MODEL.LIMITS = d.limits_deg;
      window.SCENE_TARGETS = d.targets;
      status('model loaded from the server\n' +
        `H=${MODEL.H}  upper=${MODEL.UPPER}  forearm=${MODEL.FOREARM}  tool=${MODEL.TOOL} mm\n` +
        `nominal camera fx=${d.camera.intrinsics.fx.toFixed(1)} px at ` +
        `${d.camera.intrinsics.image_size[0]}x${d.camera.intrinsics.image_size[1]}\n\n` + d.note);
      state.message = 'ready';
    }).catch((e) => status('could not load /api/model: ' + e));

    api('/api/parts').then((d) => {
      if (!d.ok) return;
      $('parts').innerHTML = '<option value="">-- pick a part --</option>' +
        d.parts.map((p) => `<option value="${p.name}">${p.name} (${p.triangles} tris)</option>`)
          .join('');
      $('partInfo').textContent = `${d.parts.length} STL files. ` + d.note;
    });
  }

  boot();
  requestAnimationFrame(tick);
})();
