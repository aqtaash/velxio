/**
 * test_mpu6050_simulation.mjs
 *
 * Full end-to-end test for the ESP32 + MPU-6050 I2C simulation.
 * Mirrors exactly what the frontend does:
 *   1. POST /api/compile/  → get firmware_b64
 *   2. WebSocket /api/simulation/ws/{id}
 *   3. send start_esp32 with firmware + sensors:[{sensor_type:'mpu6050',…}],
 *      the record carrying where the panel's sliders are
 *   4. Read the serial output and hold every printed value to the record
 *   5. Move one slider (esp32_sensor_update) and hold the values again
 *
 * The sketch selects 8 g and 500 deg/s, so a model that ignores the range
 * prints 39.23 m/s^2 for 1 g and twice the rotation; one that starts from
 * values of its own prints its own temperature until a slider moves.
 *
 * Run from the backend/ directory:
 *   node test_mpu6050_simulation.mjs [--timeout 30]
 *
 * Prerequisites: Backend running on http://localhost:8001
 */

// ─── Config ───────────────────────────────────────────────────────────────────
const BACKEND   = process.env.BACKEND_URL ?? process.argv.find(a => a.startsWith('--backend='))?.slice(10) ?? 'http://localhost:8001';
const WS_BASE   = BACKEND.replace(/^https?:/, m => m === 'https:' ? 'wss:' : 'ws:');
const SESSION   = `test-mpu6050-${Date.now()}`;
const TIMEOUT_S = parseInt(process.argv.find(a => a.startsWith('--timeout='))?.slice(10) ?? '40');

// The low byte of an I2C event, as the worker names it (esp32_worker.py,
// _I2C_OP_NAME): QEMU's enum i2c_event, then the write and the read of
// hw/i2c/picsimlab_i2c.c.
const I2C_OP = {
  0x00: 'START_RECV', 0x01: 'START_SEND', 0x02: 'START_ASYNC',
  0x03: 'FINISH',     0x04: 'NACK',
  0x05: 'WRITE',      0x06: 'READ',
};

// ─── MPU-6050 sketch (same as the example in examples.ts) ────────────────────
const SKETCH = `// ESP32 — MPU-6050 Accelerometer & Gyroscope (I2C)
// Requires: Adafruit MPU6050, Adafruit Unified Sensor libraries
// Wiring: SDA → D21  |  SCL → D22  |  VCC → 3V3  |  GND → GND

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA=21, SCL=22
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring.");
    while (true) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 ready!");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.printf("Accel X=%.2f Y=%.2f Z=%.2f m/s^2\\n",
    a.acceleration.x, a.acceleration.y, a.acceleration.z);
  Serial.printf("Gyro  X=%.3f Y=%.3f Z=%.3f rad/s\\n",
    g.gyro.x, g.gyro.y, g.gyro.z);
  Serial.printf("Temp: %.1f C\\n---\\n", temp.temperature);
  delay(500);
}`;

// ─── What the panel holds, and what the sketch must print for it ─────────────
// The record the tab's part files for the worker (parts/ProtocolParts.ts): the
// panel's values under the names its updates use. None of them is a value the
// model starts from by itself.
const PANEL = { accelX: 0.5, accelY: -0.25, accelZ: 1, gyroX: 100, gyroY: 0, gyroZ: -50, temp: 30.5 };
// One slider moves while the sketch runs; the others stay.
const MOVED = { accelX: -1 };

const G = 9.80665;                 // SENSORS_GRAVITY_STANDARD, m/s^2 per g
const DPS = Math.PI / 180;         // SENSORS_DPS_TO_RADS
// Half a count at the ranges the sketch selects (4096 LSB/g, 65.5 LSB per
// deg/s, 340 LSB per C), plus the rounding of the print.
const TOLERANCE = { accel: 0.01, gyro: 0.001, temp: 0.06 };

/** What getEvent() returns for these panel values, in the units it prints. */
function expected(panel) {
  return {
    accel: [panel.accelX * G, panel.accelY * G, panel.accelZ * G],
    gyro:  [panel.gyroX * DPS, panel.gyroY * DPS, panel.gyroZ * DPS],
    temp:  panel.temp,
  };
}

/** One printed block (Accel, Gyro, Temp lines) as numbers, or null while incomplete. */
function parseReading(lines) {
  const three = (line) => line?.match(/X=(-?[\d.]+) Y=(-?[\d.]+) Z=(-?[\d.]+)/)?.slice(1).map(Number);
  const accel = three(lines.find(l => l.startsWith('Accel')));
  const gyro  = three(lines.find(l => l.startsWith('Gyro')));
  const temp  = lines.find(l => l.startsWith('Temp:'))?.match(/Temp: (-?[\d.]+)/)?.[1];
  if (!accel || !gyro || temp === undefined) return null;
  return { accel, gyro, temp: Number(temp) };
}

/** Every value of a reading that is not what the panel says, as text. */
function mismatches(reading, panel) {
  const want = expected(panel);
  const out = [];
  ['X', 'Y', 'Z'].forEach((axis, i) => {
    if (Math.abs(reading.accel[i] - want.accel[i]) > TOLERANCE.accel)
      out.push(`Accel ${axis}=${reading.accel[i]} (expected ${want.accel[i].toFixed(2)})`);
    if (Math.abs(reading.gyro[i] - want.gyro[i]) > TOLERANCE.gyro)
      out.push(`Gyro ${axis}=${reading.gyro[i]} (expected ${want.gyro[i].toFixed(3)})`);
  });
  if (Math.abs(reading.temp - want.temp) > TOLERANCE.temp)
    out.push(`Temp=${reading.temp} (expected ${want.temp.toFixed(1)})`);
  return out;
}

// ─── Logging helpers ───────────────────────────────────────────────────────────
const T0 = Date.now();
const ts  = () => `[+${((Date.now() - T0)/1000).toFixed(3)}s]`;

const LOG_LEVELS = { INFO: '\x1b[36m', WARN: '\x1b[33m', ERROR: '\x1b[31m', OK: '\x1b[32m', I2C: '\x1b[35m', SERIAL: '\x1b[32m', RESET: '\x1b[0m' };
const log   = (lvl, ...args) => console.log(`${LOG_LEVELS[lvl] ?? ''}${ts()} [${lvl}]${LOG_LEVELS.RESET}`, ...args);
const info  = (...a) => log('INFO',   ...a);
const warn  = (...a) => log('WARN',   ...a);
const ok    = (...a) => log('OK',     ...a);
const err   = (...a) => log('ERROR',  ...a);
const i2c   = (...a) => log('I2C',    ...a);
const serial = (...a) => log('SERIAL', ...a);

// ─── Counters & state ─────────────────────────────────────────────────────────
let totalEvents   = 0;
let i2cEvents     = 0;
let i2cTraceCount = 0;
let serialLines   = [];
let serialText    = '';
let foundOK       = false;
let foundFail     = false;
// The printed blocks, in order, each with the panel values it was read under.
let block         = [];
let readings      = [];
let panel         = { ...PANEL };
let updateSent    = false;
let inFlight      = 0;
const READINGS_BEFORE_UPDATE = 3;
const READINGS_AFTER_UPDATE  = 3;

// ─── Step 1: Compile the sketch ───────────────────────────────────────────────
async function compile() {
  info('Compiling MPU6050 sketch via POST /api/compile/ ...');
  const res = await fetch(`${BACKEND}/api/compile/`, {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify({
      files:      [{ name: 'sketch.ino', content: SKETCH }],
      board_fqbn: 'esp32:esp32:esp32',
    }),
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(`Compilation failed HTTP ${res.status}: ${text.slice(0, 300)}`);
  }
  const body = await res.json();
  if (!body.success) {
    throw new Error(`Compilation error:\n${(body.error ?? body.stderr ?? 'unknown').slice(0, 500)}`);
  }
  // API returns firmware as base64 in 'binary_content' (ESP32 flash image)
  const firmware_b64 = body.binary_content ?? body.firmware_b64;
  if (!firmware_b64) {
    throw new Error(`No firmware in response. Keys: ${Object.keys(body).join(', ')}`);
  }
  const sizeKB = Math.round(firmware_b64.length * 0.75 / 1024);
  ok(`Compilation succeeded — ${sizeKB} KB firmware (has_wifi=${body.has_wifi})`);
  return firmware_b64;
}

// ─── Step 2: Run simulation via WebSocket ─────────────────────────────────────
function runSimulation(firmware_b64) {
  return new Promise((resolve, reject) => {
    const wsUrl = `${WS_BASE}/api/simulation/ws/${SESSION}`;
    info(`Connecting WebSocket → ${wsUrl}`);

    const ws = new WebSocket(wsUrl);

    const timer = setTimeout(() => {
      info(`Timeout reached (${TIMEOUT_S}s) — stopping simulation`);
      ws.close();
      resolve({ timedOut: true });
    }, TIMEOUT_S * 1000);

    ws.addEventListener('open', () => {
      ok('WebSocket connected');
      const payload = {
        type: 'start_esp32',
        data: {
          board:        'esp32',
          firmware_b64,
          sensors: [
            // Mirror what the mpu6050 part files for the worker
            { sensor_type: 'mpu6050', pin: 200 + 0x68, addr: 0x68, ...PANEL }
          ],
          wifi_enabled: false,
        },
      };
      info('Sending start_esp32 with sensors:', JSON.stringify(payload.data.sensors));
      ws.send(JSON.stringify(payload));
    });

    ws.addEventListener('message', ev => {
      totalEvents++;
      let msg;
      try { msg = JSON.parse(ev.data); } catch { return; }

      const { type, data } = msg;

      // ── Serial output ──────────────────────────────────────────────────────
      if (type === 'serial_output') {
        // A message is whatever the UART had when it was flushed, not a line:
        // a printf with a float in it arrives in pieces.
        serialText += data?.data ?? '';
        const lines = serialText.split(/\r?\n/);
        serialText = lines.pop();
        for (const line of lines) {
          if (!line.trim()) continue;
          serialLines.push(line);
          serial(`UART: ${line}`);
          if (line.includes('MPU6050 ready!') || line.includes('===BEGIN_OK===')) {
            foundOK = true;
          }
          if (line.includes('not found') || line.includes('===BEGIN_FAILED===')) {
            foundFail = true;
            warn('begin() returned FALSE — firmware reported "not found"');
          }
          // A block is complete at its separator.
          if (line.startsWith('---')) {
            const reading = parseReading(block);
            block = [];
            if (reading && inFlight > 0) inFlight--;
            else if (reading) readings.push({ reading, panel: { ...panel }, afterUpdate: updateSent });
          } else {
            block.push(line);
          }
          if (foundOK && !updateSent && readings.length >= READINGS_BEFORE_UPDATE) {
            info('Moving one slider:', JSON.stringify(MOVED));
            ws.send(JSON.stringify({ type: 'esp32_sensor_update',
                                     data: { pin: 200 + 0x68, ...MOVED } }));
            updateSent = true;
            panel = { ...panel, ...MOVED };
            // The block in print and the one after it may have been read
            // before the update reached the worker: they prove nothing.
            inFlight = 2;
          }
          if (readings.filter(r => r.afterUpdate).length >= READINGS_AFTER_UPDATE) {
            clearTimeout(timer);
            ws.close();
            resolve({ timedOut: false });
          }
        }
        // Fail fast: if error already confirmed, still wait for a bit more events
        if (foundFail && i2cEvents > 5) {
          // Give 5 more seconds to see remaining I2C events
          setTimeout(() => { clearTimeout(timer); ws.close(); resolve({ timedOut: false }); }, 5000);
        }
        return;
      }

      // ── I2C trace (the worker's model answered the event). Only a backend
      //    started with VELXIO_I2C_TRACE=1 emits it; the test does not need it. ─
      if (type === 'i2c_trace') {
        i2cTraceCount++;
        const { bus, addr, event, op, result, reg_ptr, wai_count } = data;
        const regHex = reg_ptr != null ? `0x${reg_ptr.toString(16).padStart(2,'0')}` : '??';
        const resHex = `0x${(result??0).toString(16).padStart(2,'0')}`;
        i2c(`[slave] bus=${bus} addr=0x${(addr??0).toString(16).padStart(2,'0')} ` +
            `op=${op} result=${resHex} reg_ptr=${regHex} wai=${wai_count}`);
        return;
      }

      // ── I2C event (forwarded from Python worker when addr NOT in _i2c_slaves) ─
      if (type === 'i2c_event') {
        i2cEvents++;
        const { bus, addr, event, response } = data;
        const op     = event & 0xFF;
        const d      = (event >> 8) & 0xFF;
        const opName = I2C_OP[op] ?? `0x${op.toString(16)}`;
        i2c(`[UNHANDLED slave] bus=${bus} addr=0x${(addr??0).toString(16).padStart(2,'0')} ` +
            `event=0x${(event??0).toString(16).padStart(4,'0')} op=${opName} data=0x${d.toString(16).padStart(2,'0')} ` +
            `resp=0x${(response??0).toString(16).padStart(2,'0')}`);
        return;
      }

      // ── gpio_change — show I2C pin activity ───────────────────────────────
      if (type === 'gpio_change') {
        const { pin, state } = data ?? {};
        if (pin === 21 || pin === 22) {
          // SDA=21, SCL=22 — these toggling means I2C is active on the bus
          i2c(`I2C pin toggle: GPIO${pin} (${pin===21?'SDA':'SCL'}) → ${state}`);
        }
        return;
      }

      // ── system / error ─────────────────────────────────────────────────────
      if (type === 'system') {
        info(`system event: ${JSON.stringify(data)}`);
        return;
      }
      if (type === 'error') {
        err(`simulation error: ${JSON.stringify(data)}`);
        return;
      }

      // ── Everything else ───────────────────────────────────────────────────
      if (!['gpio_change'].includes(type)) {
        info(`event type=${type} data=${JSON.stringify(data).slice(0,120)}`);
      }
    });

    ws.addEventListener('close', ev => {
      clearTimeout(timer);
      info(`WebSocket closed (code=${ev.code})`);
      resolve({ timedOut: false });
    });

    ws.addEventListener('error', ev => {
      clearTimeout(timer);
      err('WebSocket error:', ev.message ?? ev.type);
      reject(new Error('WebSocket error'));
    });
  });
}

// ─── Main ──────────────────────────────────────────────────────────────────────
async function main() {
  console.log('\n' + '═'.repeat(60));
  console.log(' TEST: ESP32 + MPU-6050 I2C Simulation');
  console.log(' Session:', SESSION);
  console.log(' Backend:', BACKEND);
  console.log(' Timeout:', TIMEOUT_S, 's');
  console.log('═'.repeat(60) + '\n');

  let firmware_b64;
  try {
    firmware_b64 = await compile();
  } catch (e) {
    err('Compilation failed:', e.message);
    process.exit(1);
  }

  console.log('\n' + '─'.repeat(60));
  console.log(' Starting simulation...');
  console.log('─'.repeat(60) + '\n');
  info('NOTE: a backend started with VELXIO_I2C_TRACE=1 also shows its I2C trace, with [slave] prefix.');
  console.log();

  const result = await runSimulation(firmware_b64);

  // ─── Summary ────────────────────────────────────────────────────────────────
  console.log('\n' + '═'.repeat(60));
  console.log(' SUMMARY');
  console.log('═'.repeat(60));
  console.log(`  Total WebSocket events received : ${totalEvents}`);
  console.log(`  I2C trace events (slave handled): ${i2cTraceCount}`);
  console.log(`  I2C events (no slave registered): ${i2cEvents}`);
  console.log(`  Serial lines received           : ${serialLines.length}`);
  console.log(`  Timed out                       : ${result.timedOut}`);
  console.log();
  console.log('  Serial output:');
  for (const l of serialLines) console.log(`    ${l}`);
  console.log();

  const wrong = readings.flatMap(({ reading, panel: p, afterUpdate }, i) =>
    mismatches(reading, p).map(m => `reading ${i + 1}${afterUpdate ? ' (after the slider moved)' : ''}: ${m}`));
  const before = readings.filter(r => !r.afterUpdate).length;
  const after  = readings.filter(r => r.afterUpdate).length;
  console.log(`  Readings checked                : ${before} before the slider moved, ${after} after`);
  console.log();

  if (foundOK && wrong.length > 0) {
    console.log('\x1b[31m  ✗ FAIL — the sketch printed values the panel does not hold\x1b[0m');
    for (const w of wrong) console.log(`    ${w}`);
    process.exit(1);
  } else if (foundOK && before >= READINGS_BEFORE_UPDATE && after >= READINGS_AFTER_UPDATE) {
    console.log('\x1b[32m  ✓ PASS — MPU6050 detected, and every reading is what the panel holds\x1b[0m');
    process.exit(0);
  } else if (foundFail) {
    console.log('\x1b[31m  ✗ FAIL — mpu.begin() returned false ("not found")\x1b[0m');
    console.log('\x1b[33m  → Check the backend (uvicorn) terminal for I2C event trace.\x1b[0m');
    console.log('\x1b[33m  → Look for "I2C bus=0 addr=0x68" lines to see the full sequence.\x1b[0m');
    process.exit(1);
  } else if (result.timedOut) {
    console.log('\x1b[33m  ? TIMEOUT — no "ready" or "not found" in serial output\x1b[0m');
    process.exit(1);
  } else {
    console.log('\x1b[33m  ? INCONCLUSIVE — WebSocket closed before result determined\x1b[0m');
    process.exit(1);
  }
}

main().catch(e => { err('Unhandled error:', e); process.exit(1); });
