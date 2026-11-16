// The solo auto's branches on one field, as SVG: shared by solo.html (the player) and build.js (node).
// Data: solo-trace.js, written by SoloAutoSimTest with WRITE_SOLO_TRACE=1. Field coordinates are metres, red's
// half at the bottom, the audience wall on the right, as tools/shots/maps/volley_spots.svg. The field picture is
// AdvantageScope's 2026-2027 FTC field (build.js finds it): red's wall on its left, so it is drawn turned a quarter.
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.SoloView = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  const W = 1000, H = 840;
  const OX = 30, OY = 78, SIZE = 600;
  const PANEL_X = 660;
  const SHARED = '#6a51a3';
  const RUN_COLORS = ['#1b9e77', '#e8590c', '#d6336c'];
  const CATEGORY = {
    shoot: ['#d62728', 'shoot'], garden: ['#bcbd22', 'Garden'], check: ['#9467bd', 'Hive check'],
    flower: ['#2ca02c', 'Flower'], drive: ['#8c8c8c', 'drive'],
  };
  const DETAIL = {
    SHOOT_PRELOAD: 'shooting the preloads', FACE_GARDEN: 'turning to the Garden', ONTO_GARDEN_WALL: 'onto the wall',
    SWEEP_GARDEN: 'sweeping the Garden', CHECK_TIP: 'checking the Hive', SHOOT_NEAR: 'shooting, near Cell',
    SHOOT_FAR: 'shooting, far Cell',
    DRIVE_TO_WALL_FLOWER: "to our wall's Flower", LOWER_ARM_WALL: 'arm down', INTAKE_WALL_FLOWER: 'taking the Flower',
    BACK_FROM_WALL_FLOWER: 'backing out', DRIVE_TO_REAR_FLOWER: 'to the rear Flower', LOWER_ARM_REAR: 'arm down',
    INTAKE_REAR_FLOWER: 'taking the Flower', BACK_FROM_REAR_FLOWER: 'backing out', DRIVE_TO_PARK: 'to the park',
    PARKED: 'parked',
  };

  /** The panel's and timeline's entry for a step: its name and category. */
  function group(step) {
    if (step === 'SHOOT_PRELOAD') return ['Preloads into the near Cell', 'shoot'];
    if (/GARDEN/.test(step)) return ['Garden', 'garden'];
    if (step === 'CHECK_TIP') return ['Check the Hive', 'check'];
    if (step === 'SHOOT_NEAR') return ['Shoot into the near Cell', 'shoot'];
    if (step === 'SHOOT_FAR') return ['Shoot into the far Cell', 'shoot'];
    if (/WALL$|WALL_FLOWER/.test(step)) return ["Our wall's Flower", 'flower'];
    if (/REAR/.test(step)) return ['Rear Flower', 'flower'];
    if (step === 'DRIVE_TO_PARK') return ['To the park', 'drive'];
    return ['Parked', 'drive'];
  }

  function segments(data, run) {
    const out = [];
    for (const r of run.trace) {
      const [name, cat] = group(data.steps[r[6]]);
      const last = out[out.length - 1];
      if (last && last.name === name) last.t1 = r[0];
      else out.push({name, cat, t0: r[0], t1: r[0]});
    }
    for (let i = 0; i + 1 < out.length; i++) out[i].t1 = out[i + 1].t0;
    out[out.length - 1].t1 = 30;
    return out;
  }

  const S = SIZE / 3.66;
  const px = (x) => OX + (x + 1.83) * S;
  const py = (y) => OY + (1.83 - y) * S;
  const f = (v) => v.toFixed(1);
  /** When the run goes its own way: the Hive check's split, or later where it leaves its parent run. */
  const start = (data, run) => (run.parent >= 0 ? run.branchS : data.splitS);
  const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;');
  const text = (x, y, s, a = '') => `<text x="${f(x)}" y="${f(y)}" ${a}>${esc(s)}</text>`;
  const rect = (x0, x1, y0, y1, a) =>
    `<rect x="${f(px(x0))}" y="${f(py(y1))}" width="${f((x1 - x0) * S)}" height="${f((y1 - y0) * S)}" ${a}/>`;

  /** The run's pose and counts at t, between its samples. */
  function at(run, t) {
    const tr = run.trace;
    const dt = tr[1][0] - tr[0][0];
    const i = Math.max(0, Math.min(tr.length - 2, Math.floor(t / dt)));
    const a = tr[i], b = tr[i + 1];
    const u = Math.max(0, Math.min(1, (t - a[0]) / (b[0] - a[0])));
    let dh = b[3] - a[3];
    if (dh > Math.PI) dh -= 2 * Math.PI;
    if (dh < -Math.PI) dh += 2 * Math.PI;
    const r = u < 0.5 ? a : b;
    return {x: a[1] + (b[1] - a[1]) * u, y: a[2] + (b[2] - a[2]) * u, h: a[3] + dh * u, shot: r[4], held: r[5],
      step: r[6]};
  }

  function trail(run, t0, t1, color, extra = '') {
    const pts = [];
    for (const r of run.trace) {
      if (r[0] >= t0 && r[0] <= t1) pts.push(`${f(px(r[1]))},${f(py(r[2]))}`);
    }
    const end = at(run, t1);
    pts.push(`${f(px(end.x))},${f(py(end.y))}`);
    return `<polyline points="${pts.join(' ')}" fill="none" stroke="${color}" stroke-width="3" stroke-linejoin="round"
      stroke-linecap="round" ${extra}/>`;
  }

  /** The robot at p; its labels lifted liftPx, clear of another robot's close by. */
  function robot(data, p, color, label, sub, liftPx = 0) {
    const half = data.field.robotHalf * S;
    const deg = -p.h * 180 / Math.PI;
    return `<g transform="translate(${f(px(p.x))} ${f(py(p.y))})">
      <g transform="rotate(${f(deg)})">
        <rect x="${f(-half)}" y="${f(-half)}" width="${f(2 * half)}" height="${f(2 * half)}" rx="4" fill="${color}"
          fill-opacity="0.85" stroke="#111" stroke-width="1.5"/>
        <rect x="${f(half - 6)}" y="${f(-half + 4)}" width="6" height="${f(2 * half - 8)}" fill="#111"/>
        <circle cx="${f(-half + 9)}" cy="0" r="5" fill="#fff" stroke="#111"/>
      </g>
      ${text(0, -half - 22 - liftPx, label, `text-anchor="middle" font-weight="bold" fill="${color}" stroke="#fff"
        stroke-width="4" paint-order="stroke"`)}
      ${text(0, -half - 8 - liftPx, sub, `text-anchor="middle" font-size="12" fill="#222" stroke="#fff"
        stroke-width="3" paint-order="stroke"`)}
    </g>`;
  }

  /** The field picture, turned so red's wall is at the bottom, then the walls' names, Cells and spots. */
  function field(data, img) {
    const fd = data.field;
    const o = [];
    const halo = 'stroke="#fff" stroke-width="3.5" paint-order="stroke"';
    o.push(`<rect x="${OX}" y="${OY}" width="${SIZE}" height="${SIZE}" fill="#777"/>`);
    if (img) {
      // The picture's field square, img.topLeft to img.bottomRight in its pixels, is img.widthM across
      const fieldPx = img.bottomRight[0] - img.topLeft[0];
      const k = img.widthM * S / fieldPx;
      const cx = (img.topLeft[0] + img.bottomRight[0]) / 2, cy = (img.topLeft[1] + img.bottomRight[1]) / 2;
      o.push(`<g transform="translate(${f(px(0))} ${f(py(0))}) rotate(-90) scale(${k.toFixed(5)})
        translate(${f(-cx)} ${f(-cy)})"><image href="${img.href}" width="${img.width}" height="${img.height}"/></g>`);
    }
    o.push(`<rect x="${OX}" y="${OY + SIZE + 2}" width="${SIZE}" height="6" fill="#d62728"/>`);
    o.push(text(OX + SIZE / 2, OY + SIZE + 22, 'RED (our) wall', `text-anchor="middle" fill="#d62728"
      font-weight="bold"`));
    o.push(`<rect x="${OX}" y="${OY - 8}" width="${SIZE}" height="6" fill="#1f77b4"/>`);
    o.push(text(OX + SIZE / 2, OY - 12, 'BLUE wall', `text-anchor="middle" fill="#1f77b4" font-weight="bold"`));
    o.push(text(OX + SIZE + 16, OY + SIZE / 2, 'AUDIENCE wall', `text-anchor="middle" fill="#444" font-weight="bold"
      transform="rotate(90 ${OX + SIZE + 16} ${OY + SIZE / 2})"`));
    for (const [name, p] of Object.entries(fd.cells)) {
      const near = name === fd.nearCell;
      o.push(`<circle cx="${f(px(p[0]))}" cy="${f(py(p[1]))}" r="5" fill="none" stroke="#fff" stroke-width="2"/>`);
      o.push(text(px(p[0]), py(p[1]) + 20, near ? 'near Cell' : 'far Cell', `text-anchor="middle" font-size="12"
        font-weight="bold" fill="#a01818" ${halo}`));
    }
    for (const s of fd.spots) {
      const x = px(s.x), y = py(s.y);
      o.push(`<path d="M${f(x)} ${f(y - 6)} L${f(x + 6)} ${f(y)} L${f(x)} ${f(y + 6)} L${f(x - 6)} ${f(y)} Z"
        fill="#fff" stroke="#222"/>`);
      const nearWall = s.x > 0.8; // Labels toward the audience wall go left of the spot, on the field
      o.push(text(nearWall ? x - 9 : x + 9, y + (nearWall ? -10 : 17), s.name, `font-size="11"
        fill="#111" ${nearWall ? 'text-anchor="end"' : ''} ${halo}`));
    }
    return o.join('\n');
  }

  function panel(data, segs, t) {
    const o = [];
    let y = OY;
    const row = (s, color) => {
      const now = t >= s.t0 && t < s.t1;
      const past = t >= s.t1;
      const [c] = CATEGORY[s.cat];
      if (now) o.push(`<rect x="${PANEL_X - 4}" y="${y - 12}" width="330" height="17" fill="${color}"
        fill-opacity="0.15"/>`);
      o.push(`<rect x="${PANEL_X}" y="${y - 9}" width="10" height="10" fill="${c}"/>`);
      o.push(text(PANEL_X + 16, y, `${s.t0.toFixed(1).padStart(4)} s  ${s.name}`, `font-size="12"
        font-family="DejaVu Sans Mono, monospace" ${now ? 'font-weight="bold"' : ''}
        fill="${past || now ? '#111' : '#999'}"`));
      y += 17;
    };
    const shared = segs[0].filter((s) => s.t0 < data.splitS - 1e-6);
    o.push(text(PANEL_X, y, 'Every run', `font-weight="bold" fill="${SHARED}"`));
    y += 20;
    shared.forEach((s) => row(s, SHARED));
    y += 4;
    o.push(text(PANEL_X, y, `Split at ${data.splitS.toFixed(1)} s by the Hive check`, `font-size="12"
      font-style="italic" fill="#555"`));
    y += 24;
    data.runs.forEach((run, i) => {
      const p = at(run, t);
      o.push(text(PANEL_X, y, `${run.name}`, `font-weight="bold" fill="${RUN_COLORS[i]}"`));
      o.push(text(PANEL_X + 120, y, `${p.shot} of 16 shot · parked ${run.parkedS.toFixed(1)} s`, `font-size="12"
        fill="#333"`));
      y += 20;
      if (run.parent >= 0) {
        o.push(text(PANEL_X, y, `Left ${data.runs[run.parent].name} at ${run.branchS.toFixed(1)} s: ${run.why}`,
          'font-size="12" font-style="italic" fill="#555"'));
        y += 18;
      }
      segs[i].filter((s) => s.t0 >= start(data, run) - 1e-6).forEach((s) => row(s, RUN_COLORS[i]));
      y += 14;
    });
    return o.join('\n');
  }

  function timeline(data, segs, t) {
    const o = [];
    const x0 = OX + 90, x1 = W - 20, y0 = OY + SIZE + 42;
    const base = y0 + data.runs.length * 24 - 4; // Below the last run's row
    const tx = (v) => x0 + (x1 - x0) * v / 30;
    data.runs.forEach((run, i) => {
      const y = y0 + i * 24;
      o.push(text(OX, y + 13, run.name, `font-size="12" font-weight="bold" fill="${RUN_COLORS[i]}"`));
      for (const s of segs[i]) {
        o.push(`<rect x="${f(tx(s.t0))}" y="${y}" width="${f(Math.max(1, tx(s.t1) - tx(s.t0)))}" height="16"
          fill="${CATEGORY[s.cat][0]}" stroke="#fff" stroke-width="0.5"/>`);
      }
      if (run.parent >= 0) {
        o.push(`<line x1="${f(tx(run.branchS))}" y1="${y - 26}" x2="${f(tx(run.branchS))}" y2="${y + 18}"
          stroke="${RUN_COLORS[i]}" stroke-width="2" stroke-dasharray="3 2"/>`);
      }
    });
    for (let v = 0; v <= 30; v += 5) {
      o.push(`<line x1="${f(tx(v))}" y1="${base}" x2="${f(tx(v))}" y2="${base + 5}" stroke="#555"/>`);
      o.push(text(tx(v), base + 17, `${v} s`, 'text-anchor="middle" font-size="11" fill="#555"'));
    }
    o.push(`<line x1="${f(tx(data.splitS))}" y1="${y0 - 6}" x2="${f(tx(data.splitS))}" y2="${base + 2}"
      stroke="${SHARED}" stroke-dasharray="3 2"/>`);
    o.push(`<line x1="${f(tx(t))}" y1="${y0 - 8}" x2="${f(tx(t))}" y2="${base + 4}" stroke="#000"
      stroke-width="2.5"/>`);
    let lx = x0;
    for (const [c, name] of Object.values(CATEGORY)) {
      o.push(`<rect x="${f(lx)}" y="${base + 26}" width="10" height="10" fill="${c}"/>`);
      o.push(text(lx + 14, base + 35, name, 'font-size="11" fill="#333"'));
      lx += 24 + name.length * 7;
    }
    return o.join('\n');
  }

  /** Balls in the air: a short line from where each left to its Cell's aim point. */
  function balls(run, t, color) {
    const o = [];
    const FLIGHT_S = 0.45;
    for (const b of run.balls) {
      const age = t - b[0];
      if (age < 0 || age > FLIGHT_S) continue;
      const u = age / FLIGHT_S;
      o.push(`<line x1="${f(px(b[1]))}" y1="${f(py(b[2]))}" x2="${f(px(b[3]))}" y2="${f(py(b[4]))}"
        stroke="${color}" stroke-width="2" stroke-dasharray="4 3" stroke-opacity="${f(1 - u * 0.7)}"/>`);
      const bx = b[1] + (b[3] - b[1]) * u, by = b[2] + (b[4] - b[2]) * u;
      o.push(`<circle cx="${f(px(bx))}" cy="${f(py(by))}" r="6" fill="#ffd400" stroke="#7a6400"/>`);
    }
    return o.join('\n');
  }

  /**
   * One frame at t seconds. img: the field picture, {href, width, height, topLeft, bottomRight, widthM}, as
   * AdvantageScope's config.json; null for none.
   */
  function frame(data, t, img) {
    const segs = data.runs.map((r) => segments(data, r));
    const split = Math.min(t, data.splitS);
    const o = [`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}"
      font-family="DejaVu Sans, Arial, sans-serif" font-size="13">`, `<rect width="${W}" height="${H}" fill="#fff"/>`];
    o.push(text(OX, 26, `Solo ${data.alliance === 'RED' ? 'Red' : 'Blue'}: every branch, simulated`,
      'font-size="20" font-weight="bold"'));
    o.push(text(OX, 46, 'Pedro on the simulated robot (SoloAutoSimTest), its drives matched to the regionals logs.'
      + ' Black bar: intake, white dot: shooter', 'font-size="12" fill="#666"'));
    o.push(text(W - 20, 40, `t = ${t.toFixed(1)} s`, 'text-anchor="end" font-size="26" font-weight="bold"'));
    o.push(field(data, img));
    o.push(trail(data.runs[0], 0, split, SHARED, 'stroke-opacity="0.9"'));
    // A run is drawn from where it goes its own way; before that it is its parent, or the shared robot
    const live = data.runs.map((run) => t > start(data, run));
    const marker = (p, color, label) => {
      o.push(`<circle cx="${f(px(p.x))}" cy="${f(py(p.y))}" r="7" fill="#fff" stroke="${color}" stroke-width="3"/>`);
      o.push(text(px(p.x) - 12, py(p.y) + 4, label, `text-anchor="end" font-size="12" font-weight="bold"
        fill="${color}" stroke="#fff" stroke-width="3" paint-order="stroke"`));
    };
    if (t > data.splitS) {
      data.runs.forEach((run, i) => live[i] && o.push(trail(run, start(data, run), t, RUN_COLORS[i])));
      marker(at(data.runs[0], data.splitS), SHARED, 'split');
      data.runs.forEach((run, i) => live[i] && run.parent >= 0 && marker(at(run, run.branchS), RUN_COLORS[i], 'recheck'));
    }
    if (t <= data.splitS) {
      const p = at(data.runs[0], t);
      o.push(balls(data.runs[0], t, SHARED));
      o.push(robot(data, p, SHARED, 'every run', DETAIL[data.steps[p.step]]));
    } else {
      data.runs.forEach((run, i) => live[i] && o.push(balls(run, t, RUN_COLORS[i])));
      const drawn = [];
      data.runs.forEach((run, i) => {
        if (!live[i]) return;
        const p = at(run, t);
        const near = drawn.filter((q) => Math.hypot(q.x - p.x, q.y - p.y) < 0.45).length;
        o.push(robot(data, p, RUN_COLORS[i], run.name, DETAIL[data.steps[p.step]], near * 30));
        drawn.push(p);
      });
    }
    o.push(panel(data, segs, t));
    o.push(timeline(data, segs, t));
    o.push('</svg>');
    return o.join('\n');
  }

  return {frame, segments, at, W, H};
});
