'use strict';
// Velxio Desktop - Electron wrapper (Linux + Windows).
// On launch:
//   1. copies the backend code to a writable folder (userData/backend)
//   2. installs Arduino cores with the bundled arduino-cli (first run only)
//   3. starts the backend (bundled Python + uvicorn) and a tiny web server for the frontend
//   4. opens the app window

const { app, BrowserWindow, dialog, shell } = require('electron');
const { spawn } = require('child_process');
const http = require('http');
const net = require('net');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const httpProxy = require('http-proxy');

// AppImage cannot use Chromium's setuid sandbox; the window only ever shows localhost content.
if (process.env.APPIMAGE) app.commandLine.appendSwitch('no-sandbox');

const FRONT_PORT = 38080; // fixed on purpose: keeps browser storage (your saved projects) stable
const API_PORT = 38081;
const IS_WIN = process.platform === 'win32';
const RP2040_INDEX =
  'https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json';
const ATTINY_INDEX = 'http://drazzy.com/package_drazzy.com_index.json';

const RES = app.isPackaged
  ? path.join(process.resourcesPath, 'app-resources')
  : path.join(__dirname, 'resources');
const DATA = app.getPath('userData'); // Linux: ~/.config/Velxio   Windows: %APPDATA%\Velxio
const LOG = path.join(DATA, 'backend.log');
const PYTHON = IS_WIN ? path.join(RES, 'python', 'python.exe') : path.join(RES, 'python', 'bin', 'python3');
const ARDUINO_CLI = path.join(RES, 'bin', IS_WIN ? 'arduino-cli.exe' : 'arduino-cli');

const MIME = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8',
  '.json': 'application/json', '.svg': 'image/svg+xml', '.png': 'image/png',
  '.jpg': 'image/jpeg', '.gif': 'image/gif', '.ico': 'image/x-icon',
  '.wasm': 'application/wasm', '.woff': 'font/woff', '.woff2': 'font/woff2',
  '.txt': 'text/plain; charset=utf-8',
};

let win = null;
let backend = null;
let server = null;
let quitting = false;

function run(cmd, args, opts = {}) {
  return new Promise((resolve, reject) => {
    const p = spawn(cmd, args, { ...opts, stdio: ['ignore', 'pipe', 'pipe'], windowsHide: true });
    let out = '';
    const keep = (d) => { out = (out + d).slice(-4000); };
    p.stdout.on('data', keep);
    p.stderr.on('data', keep);
    p.on('error', reject);
    p.on('close', (code) =>
      code === 0 ? resolve(out) : reject(new Error(`${path.basename(cmd)} ${args.join(' ')} failed:\n${out}`))
    );
  });
}

function status(msg) {
  const html =
    '<body style="margin:0;height:100vh;display:flex;align-items:center;justify-content:center;' +
    'background:#1e2327;color:#eee;font-family:sans-serif;text-align:center">' +
    `<div><h2>Velxio</h2><p>${msg}</p></div></body>`;
  win.loadURL('data:text/html;charset=utf-8,' + encodeURIComponent(html));
}

function getSecret() {
  const f = path.join(DATA, 'secret.key');
  if (!fs.existsSync(f)) {
    fs.mkdirSync(DATA, { recursive: true });
    fs.writeFileSync(f, crypto.randomBytes(32).toString('hex'), { mode: 0o600 });
  }
  return fs.readFileSync(f, 'utf8').trim();
}

function prepareBackend() {
  const dest = path.join(DATA, 'backend');
  const marker = path.join(dest, '.app-version');
  const cur = fs.existsSync(marker) ? fs.readFileSync(marker, 'utf8') : '';
  if (cur !== app.getVersion()) {
    fs.mkdirSync(dest, { recursive: true });
    // overwrites code files; the "data" folder is not in the source, so your data is kept
    fs.cpSync(path.join(RES, 'backend'), dest, { recursive: true, force: true });
    fs.writeFileSync(marker, app.getVersion());
  }
  return dest;
}

async function ensureArduino(env) {
  const marker = path.join(DATA, '.arduino-ready');
  if (fs.existsSync(marker)) return;
  const dataDir = env.ARDUINO_DIRECTORIES_DATA;
  fs.mkdirSync(dataDir, { recursive: true });
  fs.mkdirSync(env.ARDUINO_DIRECTORIES_USER, { recursive: true });

  // Same as the Docker entrypoint: seed vendored board indexes so a flaky index host can't break compiles
  const seedDir = path.join(RES, 'backend', 'board-indexes');
  if (fs.existsSync(seedDir)) {
    for (const f of fs.readdirSync(seedDir)) {
      const dest = path.join(dataDir, f);
      if (/^package_.*\.json$/.test(f) && !fs.existsSync(dest)) fs.copyFileSync(path.join(seedDir, f), dest);
    }
  }

  try {
    status('First run: downloading Arduino compilers (several minutes, needs internet)...');
    await run(ARDUINO_CLI, ['config', 'init'], { env }).catch(() => {});
    await run(ARDUINO_CLI, ['config', 'add', 'board_manager.additional_urls', RP2040_INDEX], { env }).catch(() => {});
    await run(ARDUINO_CLI, ['config', 'add', 'board_manager.additional_urls', ATTINY_INDEX], { env }).catch(() => {});
    await run(ARDUINO_CLI, ['core', 'update-index'], { env }).catch(() => {});
    await run(ARDUINO_CLI, ['core', 'install', 'arduino:avr'], { env });
    await run(ARDUINO_CLI, ['core', 'install', 'rp2040:rp2040'], { env });
    await run(ARDUINO_CLI, ['core', 'install', 'ATTinyCore:avr@1.4.1'], { env }).catch(() => {}); // optional
    fs.writeFileSync(marker, 'ok');
  } catch (e) {
    dialog.showErrorBox(
      'Arduino setup not finished',
      'Could not download the Arduino compilers. Check your internet connection.\n' +
        'The app will try again next time you open it.\n\n' + e.message
    );
  }
}

function startBackend(backendDir, env) {
  const log = fs.createWriteStream(LOG, { flags: 'a' });
  backend = spawn(
    PYTHON,
    ['-m', 'uvicorn', 'app.main:app', '--host', '127.0.0.1', '--port', String(API_PORT)],
    { cwd: backendDir, env, windowsHide: true }
  );
  backend.stdout.pipe(log, { end: false });
  backend.stderr.pipe(log, { end: false });
  backend.on('exit', (code) => {
    if (!quitting) {
      dialog.showErrorBox('Velxio backend stopped', `Exit code ${code}.\nSee log: ${LOG}`);
      app.quit();
    }
  });
}

function waitForPort(port, timeoutMs) {
  const t0 = Date.now();
  return new Promise((resolve, reject) => {
    const attempt = () => {
      const s = net.connect(port, '127.0.0.1');
      s.on('connect', () => { s.destroy(); resolve(); });
      s.on('error', () => {
        s.destroy();
        if (Date.now() - t0 > timeoutMs) reject(new Error(`Backend did not start. See log: ${LOG}`));
        else setTimeout(attempt, 500);
      });
    };
    attempt();
  });
}

const isBackendPath = (u) => u.startsWith('/api') || u === '/health';

function startFrontServer() {
  const root = path.join(RES, 'frontend');
  const proxy = httpProxy.createProxyServer({ target: `http://127.0.0.1:${API_PORT}`, ws: true });
  proxy.on('error', (err, req, res) => {
    try {
      if (res && res.writeHead) { res.writeHead(502); res.end('Backend error'); }
      else if (res && res.destroy) res.destroy();
    } catch (_) { /* ignore */ }
  });

  const srv = http.createServer((req, res) => {
    let url = '/';
    try { url = decodeURIComponent(req.url.split('?')[0]); } catch (_) { /* keep "/" */ }
    if (isBackendPath(url)) return proxy.web(req, res);

    let file = path.join(root, path.normalize(url));
    if (!file.startsWith(root)) { res.writeHead(403); return res.end(); }
    if (!fs.existsSync(file) || fs.statSync(file).isDirectory()) file = path.join(root, 'index.html');
    res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'application/octet-stream' });
    fs.createReadStream(file).pipe(res);
  });
  srv.on('upgrade', (req, socket, head) => {
    if (req.url.startsWith('/api')) proxy.ws(req, socket, head);
    else socket.destroy();
  });
  return new Promise((resolve, reject) => {
    srv.on('error', reject);
    srv.listen(FRONT_PORT, '127.0.0.1', () => resolve(srv));
  });
}

if (!app.requestSingleInstanceLock()) {
  app.quit();
} else {
  app.on('second-instance', () => {
    if (win) { if (win.isMinimized()) win.restore(); win.focus(); }
  });

  app.whenReady().then(async () => {
    win = new BrowserWindow({
      width: 1400, height: 900, backgroundColor: '#1e2327', autoHideMenuBar: true, title: 'Velxio',
    });
    win.webContents.setWindowOpenHandler(({ url }) => {
      if (url.startsWith(`http://127.0.0.1:${FRONT_PORT}`)) return { action: 'allow' };
      shell.openExternal(url);
      return { action: 'deny' };
    });

    try {
      status('Starting...');
      const backendDir = prepareBackend();
      const env = {
        ...process.env,
        PATH: `${path.join(RES, 'bin')}${path.delimiter}${process.env.PATH || ''}`,
        PYTHONUNBUFFERED: '1',
        PYTHONDONTWRITEBYTECODE: '1',
        PYTHONNOUSERSITE: '1',
        SECRET_KEY: getSecret(),
        DATA_DIR: path.join(backendDir, 'data'),
        ARDUINO_DIRECTORIES_DATA: path.join(DATA, 'arduino15'),
        ARDUINO_DIRECTORIES_USER: path.join(DATA, 'Arduino'),
      };
      await ensureArduino(env);
      status('Starting Velxio...');
      startBackend(backendDir, env);
      await waitForPort(API_PORT, 90000);
      server = await startFrontServer();
      win.loadURL(`http://127.0.0.1:${FRONT_PORT}`);
    } catch (e) {
      dialog.showErrorBox('Velxio failed to start', e.message);
      app.quit();
    }
  });

  app.on('before-quit', () => {
    quitting = true;
    if (backend) backend.kill();
    if (server) server.close();
  });
  app.on('window-all-closed', () => app.quit());
}
