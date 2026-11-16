// node tools/solo/build.js [--gif] [--mp4]: solo.html, a player for the solo auto's two branches on the real field,
// from solo-trace.js (SoloAutoSimTest with WRITE_SOLO_TRACE=1). --gif writes solo.gif (ImageMagick's magick), --mp4
// solo.mp4 in real time (magick and ffmpeg: $FFMPEG, else ffmpeg on the PATH).
// The field picture is AdvantageScope's 2026-2027 FTC field from its assets folder, which AdvantageScope downloads
// once it has run online; SOLO_FIELD_DIR picks another folder holding image.png and config.json.
const fs = require('fs');
const os = require('os');
const path = require('path');
const {execFile, execFileSync} = require('child_process');
const SoloView = require('./view.js');

const dir = __dirname;

function fieldDir() {
  if (process.env.SOLO_FIELD_DIR) return process.env.SOLO_FIELD_DIR;
  const homes = [
    process.env.APPDATA && path.join(process.env.APPDATA, 'AdvantageScope'),
    path.join(os.homedir(), '.config', 'AdvantageScope'),
    path.join(os.homedir(), 'Library', 'Application Support', 'AdvantageScope'),
  ].filter(Boolean);
  for (const home of homes) {
    for (const kind of ['userAssets', 'autoAssets']) {
      const assets = path.join(home, kind);
      if (!fs.existsSync(assets)) continue;
      const found = fs.readdirSync(assets).filter((n) => n.startsWith('Field2d_20262027FTCField')).sort().pop();
      if (found) return path.join(assets, found);
    }
  }
  throw new Error('No AdvantageScope 2026-2027 FTC field found: run AdvantageScope online once, or set SOLO_FIELD_DIR');
}

/** The field picture as SoloView.frame takes it, the image embedded. */
function fieldImage(file) {
  const folder = fieldDir();
  const config = JSON.parse(fs.readFileSync(path.join(folder, 'config.json'), 'utf8'));
  const png = fs.readFileSync(file || path.join(folder, 'image.png'));
  const full = fs.readFileSync(path.join(folder, 'image.png'));
  const scale = png.readUInt32BE(16) / full.readUInt32BE(16); // A smaller copy for the GIF's frames
  return {
    href: 'data:image/png;base64,' + png.toString('base64'),
    width: png.readUInt32BE(16), height: png.readUInt32BE(20),
    topLeft: config.topLeft.map((v) => v * scale), bottomRight: config.bottomRight.map((v) => v * scale),
    widthM: config.widthInches * 0.0254, folder,
  };
}

const traceJs = fs.readFileSync(path.join(dir, 'solo-trace.js'), 'utf8');
const data = new Function(traceJs + '; return SOLO_TRACE;')();
const img = fieldImage();
const viewJs = fs.readFileSync(path.join(dir, 'view.js'), 'utf8');

const html = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Solo auto: both branches</title>
<style>
  body { font-family: "DejaVu Sans", Arial, sans-serif; margin: 16px; background: #fafafa; }
  #controls { display: flex; gap: 12px; align-items: center; margin-bottom: 8px; max-width: 1000px; }
  #t { flex: 1; }
  button { font-size: 15px; padding: 4px 14px; }
  #view svg { max-width: 100%; height: auto; background: #fff; box-shadow: 0 1px 4px #0002; }
</style>
</head>
<body>
<div id="controls">
  <button id="play">Pause</button>
  <input id="t" type="range" min="0" max="30" step="0.02" value="0">
  <label>Speed <select id="speed">
    <option value="0.5">0.5x</option><option value="1" selected>1x</option>
    <option value="2">2x</option><option value="4">4x</option>
  </select></label>
</div>
<div id="view"></div>
<p style="color:#666;font-size:13px;max-width:1000px">Space plays and pauses, the arrow keys step 0.5 s. Built by
  tools/solo/build.js; field picture from AdvantageScope.</p>
<script>${traceJs}</script>
<script>${viewJs}</script>
<script>
  const FIELD = ${JSON.stringify({...img, folder: undefined})};
  const view = document.getElementById('view');
  const slider = document.getElementById('t');
  const playButton = document.getElementById('play');
  const speed = document.getElementById('speed');
  let t = 0, playing = true, last = null;

  function show() {
    view.innerHTML = SoloView.frame(SOLO_TRACE, t, FIELD);
    slider.value = t;
  }
  function setPlaying(p) {
    playing = p;
    playButton.textContent = p ? 'Pause' : 'Play';
    last = null;
  }
  function tick(now) {
    if (playing) {
      if (last !== null) {
        t += (now - last) / 1000 * Number(speed.value);
        if (t >= 30) { t = 30; setPlaying(false); }
        show();
      }
      last = now;
    }
    requestAnimationFrame(tick);
  }
  playButton.onclick = () => {
    if (!playing && t >= 30) t = 0;
    setPlaying(!playing);
  };
  slider.oninput = () => { t = Number(slider.value); setPlaying(false); show(); };
  document.onkeydown = (e) => {
    if (e.code === 'Space') { e.preventDefault(); playButton.onclick(); }
    if (e.code === 'ArrowRight') { t = Math.min(30, t + 0.5); setPlaying(false); show(); }
    if (e.code === 'ArrowLeft') { t = Math.max(0, t - 0.5); setPlaying(false); show(); }
  };
  show();
  requestAnimationFrame(tick);
</script>
</body>
</html>
`;
fs.writeFileSync(path.join(dir, 'solo.html'), html);
console.log(`${path.join(dir, 'solo.html')} (field from ${img.folder})`);

/** Each frame's SVG, every stepS of the auto, in a new temporary folder, the field picture resized to widthPx. */
function writeFrames(stepS, widthPx) {
  const frames = fs.mkdtempSync(path.join(os.tmpdir(), 'solo-frames-'));
  const small = path.join(frames, 'field.png');
  execFileSync('magick', [path.join(img.folder, 'image.png'), '-resize', `${widthPx}x`, small]);
  const frameImg = fieldImage(small);
  const files = [];
  for (let i = 0; i * stepS <= 30 + 1e-9; i++) {
    const file = path.join(frames, `f${String(i).padStart(4, '0')}.svg`);
    fs.writeFileSync(file, SoloView.frame(data, Math.min(30, i * stepS), frameImg));
    files.push(file);
  }
  return {frames, files};
}

/** Runs magick on each file, as many at once as there are CPUs. */
async function eachInParallel(files, args) {
  let next = 0;
  const worker = async () => {
    while (next < files.length) {
      const file = files[next++];
      await new Promise((resolve, reject) =>
        execFile('magick', args(file), (err) => (err ? reject(err) : resolve())));
    }
  };
  await Promise.all(Array.from({length: os.cpus().length}, worker));
}

async function main() {
  if (process.argv.includes('--gif')) {
    const {frames, files} = writeFrames(0.2, 1000); // Each frame shown 0.1 s: twice real time
    const lastFrame = files.pop();
    const out = path.join(dir, 'solo.gif');
    execFileSync('magick', ['-delay', '10', ...files, '-delay', '300', lastFrame, '-resize', '800x', '+dither',
      '-loop', '0', '-layers', 'Optimize', out], {stdio: 'inherit'});
    fs.rmSync(frames, {recursive: true});
    console.log(`${out}: ${files.length + 1} frames, ${(fs.statSync(out).size / 1e6).toFixed(1)} MB`);
  }
  if (process.argv.includes('--mp4')) {
    const FPS = 30;
    const {frames, files} = writeFrames(1 / FPS, 1200); // The field is 1200 px across at twice the size
    // Twice the SVG's size, then scaled down in the encode: sharper text and lines
    await eachInParallel(files, (file) => ['-density', '192', file, file.replace(/\.svg$/, '.png')]);
    const out = path.join(dir, 'solo.mp4');
    execFileSync(process.env.FFMPEG || 'ffmpeg', ['-y', '-loglevel', 'error', '-framerate', String(FPS),
      '-i', path.join(frames, 'f%04d.png'), '-vf', 'scale=1500:-2:flags=lanczos,tpad=stop_mode=clone:stop_duration=3',
      '-c:v', 'libx264', '-preset', 'slow', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out],
    {stdio: 'inherit'});
    fs.rmSync(frames, {recursive: true});
    console.log(`${out}: ${files.length} frames at ${FPS} fps, ${(fs.statSync(out).size / 1e6).toFixed(1)} MB`);
  }
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
