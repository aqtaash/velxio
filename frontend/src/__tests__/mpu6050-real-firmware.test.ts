/**
 * The MPU-6050 part under firmware that was compiled, not imitated: real
 * sketches on avr8js, the ATmega's TWI on the bus fabric, the part attached
 * the way the canvas attaches it. What is proved here is what a user sees in
 * the serial monitor.
 *
 * fixtures/avr-mpu6050-adafruit is the gallery example esp32-mpu6050 built for
 * an Arduino Uno with Adafruit_MPU6050 2.2.9, the driver most sketches use:
 *
 *   - begin() returns. Its reset() waits for DEVICE_RESET to clear with no
 *     timeout, and against a model that stored the bit the sketch printed
 *     BEGIN and nothing else, on every board whose firmware runs in the tab.
 *   - 1 g reads 9.81 m/s2 at the 8 g range the sketch selects, and 100 deg/s
 *     reads 1.745 rad/s at 500 deg/s. A model that encodes for the power-on
 *     ranges whatever the sketch selected prints 39.23 and 3.491.
 *   - The driver trace of test/fixtures/i2c-vectors/mpu6050.json was read
 *     from the library's source. Here the compiled library puts its own
 *     traffic next to it.
 *
 * fixtures/avr-mpu6050-probe is the register probe the staging board matrix
 * runs on every board (project i2c-model-fidelity-2026-09, harness/mk.py):
 * what passes here is what that matrix expects of the tab model.
 */
import { describe, it, expect, afterEach } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

import { AVRSimulator } from '../simulation/AVRSimulator';
import { PinManager } from '../simulation/PinManager';
import { PartSimulationRegistry } from '../simulation/parts/PartSimulationRegistry';
import '../simulation/parts/ProtocolParts';
import { dispatchSensorUpdate } from '../simulation/SensorUpdateRegistry';
import { busRegistry } from '../simulation/buses';
import type { BusDiagnostic } from '../simulation/buses';
import { bareBoard, wireI2cPins, clearBench } from './helpers/i2cBench';

const firmware = (name: string): string =>
  readFileSync(
    fileURLToPath(new URL(`./fixtures/${name}/${name}.ino.hex`, import.meta.url)),
    'utf-8',
  );
const ADAFRUIT = firmware('avr-mpu6050-adafruit');
const PROBE = firmware('avr-mpu6050-probe');

interface VectorStep {
  op: string;
  data?: string;
  reg?: string;
  n?: number;
  expect?: string;
  values?: Record<string, number>;
}

const VECTORS: { vectors: Array<{ driver?: string; steps: VectorStep[] }> } = JSON.parse(
  readFileSync(
    fileURLToPath(new URL('../../../test/fixtures/i2c-vectors/mpu6050.json', import.meta.url)),
    'utf8',
  ),
);

// The Uno's TWI pins (A4 / A5) as board pins of the fabric.
const UNO = 'uno';
const UNO_SDA = 18;
const UNO_SCL = 19;
const CYCLES_PER_MS = 16_000;

const hexText = (bytes: readonly number[]): string =>
  bytes.map((b) => b.toString(16).toUpperCase().padStart(2, '0')).join(' ');

interface Bench {
  /** Everything the sketch printed so far. */
  out(): string;
  /** Run until the sketch has printed `count` more lines that match, or fail. */
  lines(pattern: RegExp, count: number, withinMs?: number): string[];
  /** What the firmware put on the wire, as steps of the vector format. */
  traffic: VectorStep[];
  notes: BusDiagnostic[];
}

const listeners: Array<() => void> = [];

function bench(hex: string): Bench {
  const sim = new AVRSimulator(new PinManager(), 'uno');
  sim.loadHex(hex);
  let out = '';
  sim.onSerialData = (ch: string) => {
    out += ch;
  };
  bareBoard(UNO, 'arduino-uno', sim);
  wireI2cPins('imu', { boardId: UNO, pin: UNO_SDA }, { boardId: UNO, pin: UNO_SCL });
  const el = { addEventListener() {}, removeEventListener() {}, dispatchEvent() {} };
  PartSimulationRegistry.get('mpu6050')!.attachEvents!(
    el as unknown as HTMLElement,
    sim as never,
    () => null,
    'imu',
  );

  const notes: BusDiagnostic[] = [];
  listeners.push(busRegistry.onDiagnostic((d) => notes.push(d)));

  // Every START, byte and STOP the ATmega's TWI puts on the bus.
  const bus = busRegistry.fabric(UNO).i2cBuses.get(UNO_SDA)!;
  const traffic: VectorStep[] = [];
  let phases: Array<{ read: boolean; bytes: number[] }> = [];
  const { start, write, read, stop } = bus;
  bus.start = (address, rd) => {
    phases.push({ read: rd, bytes: [] });
    return start.call(bus, address, rd);
  };
  bus.write = (byte) => {
    phases[phases.length - 1].bytes.push(byte);
    return write.call(bus, byte);
  };
  bus.read = () => {
    const v = read.call(bus);
    phases[phases.length - 1].bytes.push(v);
    return v;
  };
  bus.stop = () => {
    const [first, second] = phases;
    if (phases.length === 1 && !first.read) {
      traffic.push({ op: 'write', data: hexText(first.bytes) });
    } else if (phases.length === 2 && !first.read && first.bytes.length === 1 && second.read) {
      traffic.push({
        op: 'read',
        reg: hexText(first.bytes),
        n: second.bytes.length,
        expect: hexText(second.bytes),
      });
    } else {
      traffic.push({ op: `unexpected: ${JSON.stringify(phases)}` });
    }
    phases = [];
    stop.call(bus);
  };

  const cycles = () => (sim as unknown as { cpu: { cycles: number } }).cpu.cycles;
  let seen = 0;
  return {
    out: () => out,
    traffic,
    notes,
    lines(pattern, count, withinMs = 3000) {
      const found: string[] = [];
      const limit = cycles() + withinMs * CYCLES_PER_MS;
      for (;;) {
        for (let nl = out.indexOf('\n', seen); nl >= 0; nl = out.indexOf('\n', seen)) {
          const line = out.slice(seen, nl).trim();
          seen = nl + 1;
          if (pattern.test(line)) found.push(line);
          if (found.length === count) return found;
        }
        if (cycles() >= limit) {
          throw new Error(
            `${found.length} of ${count} lines matching ${pattern} in ${withinMs} ms of guest time; ` +
              `the sketch printed ${JSON.stringify(out)}`,
          );
        }
        for (let i = 0; i < 20_000; i++) sim.step();
      }
    },
  };
}

afterEach(() => {
  for (const off of listeners.splice(0)) off();
  clearBench();
});

describe('Adafruit_MPU6050 on an Arduino Uno, compiled', () => {
  it('begin() returns and the sketch reads 1 g as 9.81 m/s2 at the 8 g range', () => {
    const b = bench(ADAFRUIT);
    expect(b.lines(/^(READY|NOT FOUND)$/, 1)).toEqual(['READY']);
    expect(b.out().startsWith('BEGIN\r\nREADY\r\n')).toBe(true);
    expect(b.lines(/^AZ=/, 2)).toEqual(['AZ=9.81 GX=0.000 T=24.00', 'AZ=9.81 GX=0.000 T=24.00']);
    // The driver wakes the chip before it reads it: there is nothing to say.
    expect(b.notes).toEqual([]);
  }, 120_000);

  it('follows the panel: 100 deg/s reads 1.745 rad/s, and 12 g stays at the top of the 8 g range', () => {
    const b = bench(ADAFRUIT);
    b.lines(/^AZ=/, 1);
    dispatchSensorUpdate('imu', { gyroX: 100, accelZ: 2, temp: 31.5 });
    // The line being printed when the slider moved may still carry the old sample.
    expect(b.lines(/^AZ=/, 3).pop()).toBe('AZ=19.61 GX=1.745 T=31.50');
    dispatchSensorUpdate('imu', { accelZ: 12, gyroX: -100 });
    // 32767 counts at 4096 per g.
    expect(b.lines(/^AZ=/, 3).pop()).toBe('AZ=78.45 GX=-1.745 T=31.50');
  }, 120_000);

  it('puts on the wire what the driver trace of the shared vectors says', () => {
    const b = bench(ADAFRUIT);
    b.lines(/^AZ=/, 1);
    const golden = VECTORS.vectors.find((v) => v.driver === 'adafruit-mpu6050-arduino')!.steps;
    // Up to the first getEvent(); after it the vector moves a slider.
    const untilTheSlider = golden.slice(
      0,
      golden.findIndex((s) => s.op === 'inputs'),
    );
    expect(b.traffic.slice(0, untilTheSlider.length)).toEqual(untilTheSlider);
  }, 120_000);
});

describe('the register probe of the staging board matrix, on an Arduino Uno', () => {
  it('finds the chip asleep, its reset bits cleared by the first read, and 1 g at 4096 counts in the 8 g range', () => {
    const b = bench(PROBE);
    expect(b.lines(/^P\d |^PROBE_DONE$/, 8)).toEqual([
      'P0 whoami=0x68',
      'P1 pwr_at_boot=0x40',
      'P2 after_reset pwr=0x40 polls=0',
      'P3 sig_path_reset=0x0',
      'P4 user_ctrl=0x0',
      'P5 az_raw_at_8g=4096',
      'P6 int_status=0x0',
      'PROBE_DONE',
    ]);
    // It wakes the chip before it reads the sample block.
    expect(b.notes).toEqual([]);
  }, 120_000);
});
