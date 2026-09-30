/**
 * ProtocolParts.ts — Simulation for I2C, SPI, and custom-protocol components.
 *
 * Implements eight components that require specific communication stacks:
 *
 *  ssd1306      — I2C OLED display (0x3C). Full command/data decoder.
 *  ds1307       — I2C Real-Time Clock (0x68). Returns browser system time.
 *  mpu6050      — I2C 6-axis IMU (0x68/0x69). Full register map simulation.
 *  dht22        — Single-wire temp/humidity. Drives DATA pin after start signal.
 *  hx711        — 2-wire load cell amplifier. Clocks out 24-bit ADC value.
 *  ir-receiver  — IR demodulator. Owns its pin through the line contract and
 *                 puts what crosses simulation/ir/irAir on it, at real timing.
 *  ir-remote    — IR handset. No pins: it transmits into the air.
 *  microsd-card — SPI SD card. Responds to CMD0/CMD8/ACMD41/CMD58 init.
 *
 * NOTE — the timing-sensitive parts (dht22, ir-receiver) do NOT live here any
 *   more. They are line-owning models under simulation/line/models, which place
 *   edges on the guest's own cycle counter, so a µs-accurate protocol is exact
 *   and an interrupt-driven library (IRremote, Adafruit DHT) decodes it. What
 *   is left in this file drives pins from host time and is fine with that.
 */

import { PartSimulationRegistry } from './PartSimulationRegistry';
import { attachSpiDevice, type SpiDevice } from '../buses';
import {
  loadSdBusChip,
  SdSpiCard,
  sdCardRemoteBlobWrite,
  sdCardHasRemoteModel,
  sdCardRemoteModel,
  sdSpiFabricDevice,
} from './sdSpiCard';
import { requestLine, releaseLineGap } from '../line/requestLine';
import { VirtualDS1307, VirtualBMP280, VirtualDS3231, VirtualPCF8574 } from '../I2CBusManager';
import type { I2CDevice } from '../I2CBusManager';
import { attachI2cPart } from './i2cPart';
import { HD44780Decoder } from '../HD44780Decoder';
import { registerSensorUpdate, unregisterSensorUpdate } from '../SensorUpdateRegistry';
import { getSensorControl, sensorControlDefault } from '../sensorControlConfig';
import {
  emitIr,
  listenIr,
  necEncode,
  necEncodeRepeat,
  NEC_REPEAT_PERIOD_MS,
  type IrAirFrame,
  type IrPulse,
} from '../ir';
import { useSimulatorStore, registerSdImageReader } from '../../store/useSimulatorStore';

// ─── SSD1306 OLED ────────────────────────────────────────────────────────────

/**
 * SSD1306Core — shared GDDRAM buffer, command decoder, and rendering logic.
 *
 * The SSD1306 command set is identical for I2C and SPI; only the transport
 * differs.  This core is used by both VirtualSSD1306 (I2C) and
 * attachSSD1306SPI (SPI).
 *
 * Supported commands:
 *  - 0x20 Set Memory Addressing Mode (horizontal / vertical / page)
 *  - 0x21 Set Column Address
 *  - 0x22 Set Page Address
 *  - 0x40–0x7F Set Display Start Line
 *  - 0xAF Display ON / 0xAE Display OFF
 *  - All other parameterized commands are parsed but ignored.
 */
class SSD1306Core {
  /** 1024-byte GDDRAM: 8 pages × 128 columns. Each byte = 8 vertical pixels. */
  readonly buffer = new Uint8Array(128 * 8);

  // GDDRAM cursor
  private col = 0;
  private page = 0;
  private colStart = 0;
  private colEnd = 127;
  private pageStart = 0;
  private pageEnd = 7;
  // 0=horizontal, 1=vertical, 2=page. SSD1306 power-on default is PAGE
  // addressing (datasheet 10b). Adafruit_SSD1306 overrides it to horizontal
  // via 0x20,0x00; page-mode drivers (Tiny4kOLED, U8g2 page buffer) rely on
  // this default and never send 0x20 — so the default MUST be 2 or their
  // setCursor (0xB0-0xB7 + 0x00-0x1F) renders garbled.
  private memMode = 2;

  // Multi-byte command accumulation
  private cmdBuf: number[] = [];
  private cmdWant = 0;

  /** How many parameter bytes does this command require? */
  static cmdParams(cmd: number): number {
    if (
      cmd === 0x20 ||
      cmd === 0x81 ||
      cmd === 0x8d ||
      cmd === 0xa8 ||
      cmd === 0xd3 ||
      cmd === 0xd5 ||
      cmd === 0xd8 ||
      cmd === 0xd9 ||
      cmd === 0xda ||
      cmd === 0xdb
    )
      return 1;
    if (cmd === 0x21 || cmd === 0x22) return 2;
    return 0;
  }

  /** Write a data byte to GDDRAM and advance cursor. */
  writeData(value: number): void {
    this.buffer[this.page * 128 + this.col] = value;
    this.advanceCursor();
  }

  /**
   * Drop a half-received command. The MCU restarting leaves the panel
   * powered, so its GDDRAM and its configuration survive (the glass keeps
   * showing the last frame); only the bytes of a command that will never be
   * completed have to go, or the first byte of the new init would be eaten
   * as their parameter.
   */
  resetCommand(): void {
    this.cmdBuf = [];
    this.cmdWant = 0;
  }

  /** Feed a command or parameter byte. Multi-byte commands are accumulated. */
  writeCommand(value: number): void {
    if (this.cmdWant > 0) {
      this.cmdBuf.push(value);
      this.cmdWant--;
      if (this.cmdWant === 0) this.applyCmd();
      return;
    }
    this.cmdBuf = [value];
    this.cmdWant = SSD1306Core.cmdParams(value);
    if (this.cmdWant === 0) this.applyCmd();
  }

  private applyCmd(): void {
    const [cmd, p1, p2] = this.cmdBuf;
    switch (cmd) {
      case 0x20:
        this.memMode = p1 & 0x03;
        break;
      case 0x21:
        this.colStart = p1 & 0x7f;
        this.colEnd = p2 & 0x7f;
        this.col = this.colStart;
        break;
      case 0x22:
        this.pageStart = p1 & 0x07;
        this.pageEnd = p2 & 0x07;
        this.page = this.pageStart;
        break;
      default:
        // Page-addressing-mode cursor commands (single-byte). Used by
        // Tiny4kOLED / U8g2 page buffer / classic SSD1306 drivers whose
        // setCursor() does NOT use the 0x21/0x22 column/page-range commands.
        if (cmd >= 0xb0 && cmd <= 0xb7) {
          // set page start address (B0..B7 → page 0..7)
          this.page = cmd & 0x07;
        } else if (cmd <= 0x0f) {
          // set lower column nibble (0x00..0x0F)
          this.col = (this.col & 0xf0) | (cmd & 0x0f);
        } else if (cmd >= 0x10 && cmd <= 0x1f) {
          // set higher column nibble (0x10..0x1F)
          this.col = (this.col & 0x0f) | ((cmd & 0x0f) << 4);
        } else if (cmd >= 0x40 && cmd <= 0x7f) {
          /* display start line — visual, skip */
        }
        break;
    }
  }

  private advanceCursor(): void {
    if (this.memMode === 0) {
      // horizontal addressing
      this.col++;
      if (this.col > this.colEnd) {
        this.col = this.colStart;
        this.page++;
        if (this.page > this.pageEnd) this.page = this.pageStart;
      }
    } else if (this.memMode === 1) {
      // vertical addressing
      this.page++;
      if (this.page > this.pageEnd) {
        this.page = this.pageStart;
        this.col++;
        if (this.col > this.colEnd) this.col = this.colStart;
      }
    } else {
      // page addressing
      this.col++;
      if (this.col > this.colEnd) this.col = this.colStart;
    }
  }

  /**
   * Push the 1-bit GDDRAM buffer to the wokwi-ssd1306 web component.
   *
   * wokwi-ssd1306 API:
   *   - `element.imageData` — a 128×64 ImageData (RGBA, 4 bytes/pixel)
   *   - `element.redraw()` — flushes imageData to the internal canvas
   */
  syncElement(element: HTMLElement): void {
    const el = element as any;
    if (!el) return;

    let imgData: ImageData | undefined = el.imageData;
    if (!imgData || imgData.width !== 128 || imgData.height !== 64) {
      try {
        imgData = new ImageData(128, 64);
      } catch {
        return;
      }
    }

    const px = imgData.data;

    for (let page = 0; page < 8; page++) {
      for (let col = 0; col < 128; col++) {
        const byte = this.buffer[page * 128 + col];
        for (let bit = 0; bit < 8; bit++) {
          const row = page * 8 + bit;
          const lit = (byte >> bit) & 1;
          const idx = (row * 128 + col) * 4;
          px[idx] = lit ? 200 : 0; // R
          px[idx + 1] = lit ? 230 : 0; // G
          px[idx + 2] = lit ? 255 : 0; // B
          px[idx + 3] = 255; // A
        }
      }
    }

    el.imageData = imgData;
    if (typeof el.redraw === 'function') el.redraw();
  }
}

/**
 * VirtualSSD1306 — I2C wrapper around SSD1306Core.
 *
 * Handles the I2C control byte (0x00 = command stream, 0x40 = data stream)
 * and delegates command/data writes to the shared core.
 */
class VirtualSSD1306 implements I2CDevice {
  address: number;
  private readonly core = new SSD1306Core();

  private ctrlByte = true;
  private isData = false;
  private readonly element: HTMLElement;

  constructor(address: number, element: HTMLElement) {
    this.address = address;
    this.element = element;
  }

  /** Expose core buffer for tests. */
  get buffer(): Uint8Array {
    return this.core.buffer;
  }

  writeByte(value: number): boolean {
    if (this.ctrlByte) {
      this.isData = (value & 0x40) !== 0;
      this.ctrlByte = false;
      return true;
    }
    if (this.isData) {
      this.core.writeData(value);
    } else {
      this.core.writeCommand(value);
    }
    return true;
  }

  readByte(): number {
    return 0xff;
  }

  stop(): void {
    this.ctrlByte = true;
    this.core.syncElement(this.element);
  }
}

/**
 * Attach SSD1306 in SPI mode.
 *
 * The panel is a write-only sink on its bus: it has no MISO pin at all, so it
 * answers null and lets the fabric resolve the line, and it takes a whole
 * block in one call when the controller clocks one (DESIGN 11). Chip select
 * is not its business either: the fabric hands it the frames clocked while
 * this panel is selected and nothing else, so a CS strapped to a rail is a
 * selection like any other instead of an edge that never comes.
 *
 * D/C stays a plain pin (F3 rule 6), read from its CURRENT level when the
 * part attaches: a panel that (re)attaches in the middle of a frame, while
 * the sketch holds D/C high, must keep decoding pixels, not commands.
 */
function attachSSD1306SPI(
  element: HTMLElement,
  simulator: any,
  getPin: (name: string) => number | null,
  componentId?: string,
): () => void {
  const pinManager = simulator?.pinManager;
  const core = new SSD1306Core();
  let dcState = false;
  const unsubs: (() => void)[] = [];

  // D/C: LOW = command, HIGH = data.
  const pinDC = getPin('DC');
  if (pinDC !== null && pinDC >= 0 && pinManager) {
    dcState = pinManager.peekPinState?.(pinDC) ?? false;
    unsubs.push(
      pinManager.onPinChange(pinDC, (_: number, s: boolean) => {
        dcState = s;
      }),
    );
  }

  // Throttle rendering to ~60 fps
  let dirty = false;
  let rafId: number | null = null;
  const scheduleSync = () => {
    if (rafId !== null) return;
    rafId = requestAnimationFrame(() => {
      rafId = null;
      if (dirty) {
        core.syncElement(element);
        dirty = false;
      }
    });
  };

  const take = (value: number): void => {
    if (!dcState) {
      core.writeCommand(value);
      return;
    }
    core.writeData(value);
    dirty = true;
    scheduleSync();
  };

  // No deselect(): a real SSD1306 keeps its command parser across chip
  // select. Adafruit_SSD1306 sends a two-byte command as two transactions
  // (ssd1306_command1 per byte, CS toggled in between), and the panel on the
  // bench still reads the second byte as the parameter of the first.
  const device: SpiDevice = {
    transfer: (value: number): number | null => {
      take(value);
      return null;
    },
    transferBlock: (bytes: Uint8Array): void => {
      for (let i = 0; i < bytes.length; i++) take(bytes[i]);
    },
    boardReset: () => core.resetCommand(),
  };

  const handle = attachSpiDevice(
    {
      owner: componentId ?? (element as { id?: string }).id ?? 'ssd1306',
      componentId,
      pins: { sck: 'CLK', mosi: 'DATA', cs: 'CS' },
      // 4-wire SPI: the byte is latched on the rising edge of the clock,
      // which idles either way (datasheet 8.1.3).
      modes: [0, 3],
      // A module wired for SPI whose CS pad is left open is the only chip on
      // the bus: that is how the bench wires it, and how every project
      // migrated from the old ssd1306-spi entry is wired.
      csWhenFloating: 'selected',
    },
    device,
  );

  return () => {
    handle.dispose();
    if (rafId !== null) cancelAnimationFrame(rafId);
    unsubs.forEach((u) => u());
  };
}

/**
 * Internal: SSD1306 attach logic, parameterised over the wire protocol.
 * Called by the single `ssd1306` entry once the protocol has been resolved
 * (auto-detected from the wiring, or read from an explicit `protocol` property).
 */
function attachSSD1306(
  element: HTMLElement,
  simulator: unknown,
  getPin: (n: string) => number | null,
  protocol: 'i2c' | 'spi',
  i2cAddr = 0x3c,
  componentId?: string,
  /** The 8-pin module's I2C mode uses D1 (DATA) as SDA and D0 (CLK) as SCL;
   *  the 4-pin module names them. */
  i2cPins: { scl: string; sda: string } = { scl: 'CLK', sda: 'DATA' },
): () => void {
  if (protocol === 'spi') {
    return attachSSD1306SPI(element, simulator, getPin, componentId);
  }
  const device = new VirtualSSD1306(i2cAddr, element);
  // A QEMU board's worker only ACKs and echoes the writes, and this copy
  // draws from them.
  const part = attachI2cPart({
    simulator,
    componentId,
    device,
    pins: i2cPins,
    worker: {
      type: 'ssd1306',
      echo: (data) => {
        data.forEach((b: number) => device.writeByte(b));
        device.stop();
      },
    },
  });
  return () => part.dispose();
}


/**
 * Which wire protocol did the user build?  A real SSD1306 breakout is ONE board
 * that talks either I2C or SPI depending on how it is wired.  The definitive
 * SPI-only signal is chip-select (CS): I2C never uses it.  (DC deliberately does
 * NOT count — on the 8-pin module DC doubles as the I2C address-select/SA0 line,
 * so many I2C circuits wire it too.)  So CS wired to a GPIO => SPI, otherwise
 * I2C.  This mirrors the physical part — one component, no protocol switch to
 * set, just wire it up.
 *
 * Pure wiring check: the wires decide, never the simulator. Which board is
 * under the part says nothing about which of its two buses the user built.
 */
function detectSSD1306Protocol(getPin: (n: string) => number | null): 'i2c' | 'spi' {
  return getPin('CS') !== null ? 'spi' : 'i2c';
}

/**
 * SSD1306 OLED — a single component that works on every board with an I2C or
 * SPI bus (AVR, RP2040, ESP32, STM32).  New projects just wire it up and the
 * protocol is auto-detected from the wiring like the physical module; a
 * `protocol` property, when present, pins it explicitly (projects migrated from
 * the old ssd1306-i2c / ssd1306-spi entries carry it so their behaviour is
 * preserved exactly).  Consolidates the old three picker entries into one
 * (issues #101 / #215).
 */
PartSimulationRegistry.register('ssd1306', {
  attachEvents: (element, simulator, getPin, componentId) => {
    const { components } = useSimulatorStore.getState();
    const comp = components.find((c) => c.id === componentId);
    const i2cAddr = parseI2cAddress(comp?.properties?.i2cAddress, 0x3c);
    const explicit = comp?.properties?.protocol;
    const protocol: 'i2c' | 'spi' =
      explicit === 'i2c' || explicit === 'spi' ? explicit : detectSSD1306Protocol(getPin);
    return attachSSD1306(element, simulator, getPin, protocol, i2cAddr, componentId);
  },
});

/**
 * SSD1306 OLED (I2C, 4-pin) — the cheap `velxio-ssd1306-i2c` module (GND/VCC/
 * SCL/SDA). Same display core as the 8-pin part but I2C-only by construction,
 * so there's no protocol to detect. Issue #215.
 */
PartSimulationRegistry.register('ssd1306-i2c-4pin', {
  attachEvents: (element, simulator, getPin, componentId) => {
    const { components } = useSimulatorStore.getState();
    const comp = components.find((c) => c.id === componentId);
    const i2cAddr = parseI2cAddress(comp?.properties?.i2cAddress, 0x3c);
    return attachSSD1306(element, simulator, getPin, 'i2c', i2cAddr, componentId, {
      scl: 'SCL',
      sda: 'SDA',
    });
  },
});

// ─── DS1307 RTC ──────────────────────────────────────────────────────────────

/**
 * DS1307 Real-Time Clock — uses the pre-built VirtualDS1307 from I2CBusManager.
 * Returns the browser's current system time in BCD format for registers 0–6.
 */
PartSimulationRegistry.register('ds1307', {
  attachEvents: (_element, simulator, _getPin, componentId) => {
    const part = attachI2cPart({
      simulator,
      componentId,
      device: new VirtualDS1307(),
      worker: { type: 'ds1307' },
    });
    return () => part.dispose();
  },
});

// ─── MPU-6050 IMU ────────────────────────────────────────────────────────────

/**
 * What the MPU-6050 does with a byte written to it, and how it turns motion
 * into counts, as one table. The model below works from it, the tests hold it
 * against the copy in test/fixtures/i2c-vectors/mpu6050.json, and the backend
 * twin (esp32_i2c_slaves.MPU6050Slave) follows the same facts. Sections are
 * those of the register map, RM-MPU-6000A-00 rev 4.2.
 */
export const MPU6050_RULES = {
  /** Every register powers on at 0x00 but these: asleep, and its id (section 3). */
  power_on: { 0x6b: 0x40, 0x75: 0x68 },
  /**
   * Inclusive ranges a write leaves as they are: I2C_MST_STATUS, INT_STATUS,
   * the sample block with the external sensor data behind it, FIFO_COUNT
   * and WHO_AM_I (sections 4.13, 4.16 to 4.20, 4.30 and 4.32).
   */
  read_only: [
    [0x36, 0x36],
    [0x3a, 0x3a],
    [0x3b, 0x60],
    [0x72, 0x73],
    [0x75, 0x75],
  ],
  /**
   * Bits that start something and are never stored, so the next read finds
   * them at 0. All of SIGNAL_PATH_RESET, which is write-only (4.26). The
   * resets of USER_CTRL (4.27) and its bit 3, where i2cdevlib and InvenSense's
   * own driver reset the DMP; i2cdevlib sets one bit at a time with a
   * read-modify-write, so a bit that stuck would fire again on every later
   * write. DEVICE_RESET in PWR_MGMT_1 (4.28).
   */
  self_clearing: { 0x68: 0xff, 0x6a: 0x0f, 0x6b: 0x80 },
  /**
   * Counts per g by AFS_SEL (4.17) and per degree per second by FS_SEL
   * (4.19). The table and not 131 / 2^n, which gives 32.75 and 16.375: the
   * drivers divide by 32.8 and 16.4.
   */
  accel_lsb_per_g: [16384, 8192, 4096, 2048],
  gyro_lsb_per_dps: [131, 65.5, 32.8, 16.4],
  /** TEMP_OUT = (T - 36.53) * 340 (4.18). */
  temp_lsb_per_c: 340,
  temp_offset_c: 36.53,
} as const;

/** Motion and temperature at the chip, under the names of the panel's sliders. */
export interface Mpu6050Inputs {
  /** g */
  accelX: number;
  accelY: number;
  accelZ: number;
  /** degrees per second */
  gyroX: number;
  gyroY: number;
  gyroZ: number;
  /** degrees Celsius */
  temp: number;
}

const MPU6050_INPUT_KEYS = [
  'accelX',
  'accelY',
  'accelZ',
  'gyroX',
  'gyroY',
  'gyroZ',
  'temp',
] as const;

const MPU_GYRO_CONFIG = 0x1b;
const MPU_ACCEL_CONFIG = 0x1c;
/** ACCEL_XOUT_H to GYRO_ZOUT_L: three axes, the die temperature, three axes. */
const MPU_SAMPLE_FIRST = 0x3b;
const MPU_SAMPLE_LAST = 0x48;
const MPU_USER_CTRL = 0x6a;
const MPU_SIG_COND_RESET = 0x01;
const MPU_PWR_MGMT_1 = 0x6b;
const MPU_DEVICE_RESET = 0x80;
const MPU_SLEEP = 0x40;

const MPU_READ_ONLY = new Uint8Array(256);
for (const [first, last] of MPU6050_RULES.read_only) MPU_READ_ONLY.fill(1, first, last + 1);
const MPU_SELF_CLEARING = new Uint8Array(256);
for (const [reg, mask] of Object.entries(MPU6050_RULES.self_clearing)) {
  MPU_SELF_CLEARING[Number(reg)] = mask;
}

/**
 * A physical value as the counts of a 16-bit output register. Half a count
 * rounds away from zero, so a tilt one way and the same tilt the other way
 * read the same size (Math.round sends -65.5 to -65), and what does not fit
 * stays at the end of the scale, as the converter's output does.
 */
function mpuCounts(value: number): number {
  const counts = Math.sign(value) * Math.round(Math.abs(value));
  return Math.max(-32768, Math.min(32767, counts));
}

/**
 * Virtual MPU-6050: the 6-axis IMU, modelled where a driver can tell the
 * difference from the chip.
 *
 *  - It powers on asleep (PWR_MGMT_1 = 0x40), so a sketch that reads without
 *    waking it reads zeros, as it does on the bench.
 *  - DEVICE_RESET puts every register back to its power-on value and is gone
 *    before the next read. Adafruit_MPU6050::reset() polls that bit with no
 *    timeout: while it was stored like any other byte, begin() never returned.
 *  - What the panel sets is the world around the chip, not a register: it
 *    survives a reset, and the sample block 0x3B-0x48 is worked out from it
 *    and from the full-scale ranges the sketch selected, when a read begins.
 *    The whole burst is answered from that one sample (4.17), so a slider
 *    moving while it is read cannot mix two instants.
 *  - Asleep, the block holds what it held when the chip fell asleep.
 */
export class VirtualMPU6050 implements I2CDevice {
  address: number;
  /** The sample block was read while the chip sleeps, for the first time in this run. */
  onAsleepRead: (() => void) | null = null;

  private readonly regs = new Uint8Array(256);
  private readonly inputs: Mpu6050Inputs = {
    accelX: 0,
    accelY: 0,
    accelZ: 1,
    gyroX: 0,
    gyroY: 0,
    gyroZ: 0,
    temp: 24,
  };
  /** The sample the read in progress is answered from. */
  private readonly sample = new Uint8Array(MPU_SAMPLE_LAST - MPU_SAMPLE_FIRST + 1);
  /** What the block held when SLEEP was set; zeros after power-on and reset. */
  private readonly lastAwake = new Uint8Array(MPU_SAMPLE_LAST - MPU_SAMPLE_FIRST + 1);
  /** No START was heard for the read that comes next: latch on its first byte. */
  private latchDue = true;
  private asleepReadSaid = false;
  private regPtr = 0;
  private firstByte = true;

  constructor(address: number) {
    this.address = address;
    this.powerOn();
  }

  /** The panel moved. Only the values it names change. */
  setInputs(values: Record<string, unknown>): void {
    for (const key of MPU6050_INPUT_KEYS) {
      const v = values[key];
      if (typeof v === 'number' && Number.isFinite(v)) this.inputs[key] = v;
    }
  }

  getInputs(): Mpu6050Inputs {
    return { ...this.inputs };
  }

  start(read: boolean): void {
    if (read) this.latch();
  }

  writeByte(value: number): boolean {
    // For a host that does not say where a read begins: after a write, the
    // next byte read is the first of a new read.
    this.latchDue = true;
    if (this.firstByte) {
      this.regPtr = value & 0xff;
      this.firstByte = false;
      return true;
    }
    const reg = this.regPtr;
    this.regPtr = (reg + 1) & 0xff;
    this.writeRegister(reg, value & 0xff);
    return true;
  }

  readByte(): number {
    if (this.latchDue) this.latch();
    const reg = this.regPtr;
    this.regPtr = (reg + 1) & 0xff;
    if (reg < MPU_SAMPLE_FIRST || reg > MPU_SAMPLE_LAST) return this.regs[reg];
    if (this.asleep && !this.asleepReadSaid) {
      this.asleepReadSaid = true;
      this.onAsleepRead?.();
    }
    return this.sample[reg - MPU_SAMPLE_FIRST];
  }

  /**
   * The pointer survives the STOP: i2cdevlib writes it in one transaction and
   * reads in the next, and QEMU ends every write phase this way.
   */
  stop(): void {
    this.firstByte = true;
    this.latchDue = true;
  }

  /** A new run reads a chip that is still asleep: its monitor is told as well. */
  boardReset(): void {
    this.asleepReadSaid = false;
  }

  /**
   * The registers as a read would find them now: the sample block encoded
   * from the panel's values (or what a sleeping chip holds), and no trigger
   * bit. A host that answers its guest from a copy (the Raspberry Pi relay)
   * mirrors this.
   */
  dumpRegisters(): Uint8Array {
    const out = this.regs.slice();
    out.set(this.asleep ? this.lastAwake : this.encode(), MPU_SAMPLE_FIRST);
    return out;
  }

  private get asleep(): boolean {
    return (this.regs[MPU_PWR_MGMT_1] & MPU_SLEEP) !== 0;
  }

  private powerOn(): void {
    this.regs.fill(0);
    for (const [reg, value] of Object.entries(MPU6050_RULES.power_on)) {
      this.regs[Number(reg)] = value;
    }
    this.lastAwake.fill(0);
  }

  private writeRegister(reg: number, value: number): void {
    if (MPU_READ_ONLY[reg]) return;
    if (reg === MPU_PWR_MGMT_1 && (value & MPU_DEVICE_RESET) !== 0) {
      // Nothing of the byte is kept, SLEEP and CLKSEL included: Adafruit's
      // read-modify-write sends 0xC0 and then has to read 0x40.
      this.powerOn();
      return;
    }
    // SIG_COND_RESET clears the sensor registers too (4.27), which shows on a
    // sleeping chip; one that is awake has a new sample by the next read.
    if (reg === MPU_USER_CTRL && (value & MPU_SIG_COND_RESET) !== 0) this.lastAwake.fill(0);
    const stored = value & ~MPU_SELF_CLEARING[reg] & 0xff;
    // The chip sampled until now, so what it holds asleep is this instant.
    if (reg === MPU_PWR_MGMT_1 && (stored & MPU_SLEEP) !== 0 && !this.asleep) {
      this.lastAwake.set(this.encode());
    }
    this.regs[reg] = stored;
  }

  private latch(): void {
    this.sample.set(this.asleep ? this.lastAwake : this.encode());
    this.latchDue = false;
  }

  private encode(): Uint8Array {
    const { inputs, regs } = this;
    const accel = MPU6050_RULES.accel_lsb_per_g[(regs[MPU_ACCEL_CONFIG] >> 3) & 3];
    const gyro = MPU6050_RULES.gyro_lsb_per_dps[(regs[MPU_GYRO_CONFIG] >> 3) & 3];
    const block = [
      inputs.accelX * accel,
      inputs.accelY * accel,
      inputs.accelZ * accel,
      (inputs.temp - MPU6050_RULES.temp_offset_c) * MPU6050_RULES.temp_lsb_per_c,
      inputs.gyroX * gyro,
      inputs.gyroY * gyro,
      inputs.gyroZ * gyro,
    ];
    const out = new Uint8Array(block.length * 2);
    block.forEach((value, i) => {
      const counts = mpuCounts(value);
      out[2 * i] = (counts >> 8) & 0xff;
      out[2 * i + 1] = counts & 0xff;
    });
    return out;
  }
}

PartSimulationRegistry.register('mpu6050', {
  attachEvents: (element, simulator, _getPin, componentId) => {
    const el = element as any;
    // Respect AD0 pin: `el.ad0 = true` → address 0x69, else 0x68
    const addr = el.ad0 === true || el.ad0 === 'true' ? 0x69 : 0x68;
    const device = new VirtualMPU6050(addr);
    // The world starts where the panel's sliders do.
    device.setInputs(getSensorControl('mpu6050')?.defaultValues ?? {});
    const part = attachI2cPart({
      simulator,
      componentId,
      device,
      // The worker's copy starts from the same values, under the names its
      // sensor updates use, and not from defaults of its own.
      worker: { type: 'mpu6050', props: { ...device.getInputs() } },
    });
    device.onAsleepRead = () =>
      part.report(
        'i2c-target-asleep',
        `MPU6050 0x${addr.toString(16)} is in sleep mode: write 0x00 to PWR_MGMT_1 (0x6B) to wake it`,
      );

    registerSensorUpdate(componentId, (values) => {
      // The worker's copy answers a QEMU board; this one answers every board
      // whose firmware runs in the tab, and the Pi relay reads its registers.
      part.updateWorker(values);
      device.setInputs(values);
    });

    return () => {
      part.dispose();
      unregisterSensorUpdate(componentId);
    };
  },
});

// ─── DHT22 Temperature / Humidity Sensor ─────────────────────────────────────

/**
 * DHT22 (AM2302) — a line-owning sensor. The protocol (start signal, 40-bit
 * self-timed reply on the same wire) is the MODEL in simulation/line/models/
 * dht22.ts, hosted by whichever board the part is wired to under the line
 * contract. This part only binds the canvas element to that contract: it says
 * what it is and where it sits, forwards slider changes, and shows the
 * board's answer when the board cannot host it.
 *
 * Default values: 50.0% humidity, 25.0 C. Change via `el.temperature` /
 * `el.humidity`.
 */
PartSimulationRegistry.register('dht22', {
  attachEvents: (element, simulator, getPin, componentId) => {
    // wokwi-dht22 element uses 'SDA' as the data pin name (not 'DATA')
    const pin = getPin('SDA') ?? getPin('DATA');
    if (pin === null) return () => {};

    const el = element as { temperature?: number; humidity?: number };
    const answer = requestLine(
      simulator,
      {
        sensor_type: 'dht22',
        pin,
        temperature: el.temperature ?? 25.0,
        humidity: el.humidity ?? 50.0,
      },
      { componentId },
    );

    // SensorControlPanel: update temperature / humidity on the element, and
    // tell the host when there is one.
    registerSensorUpdate(componentId, (values) => {
      if ('temperature' in values) el.temperature = values.temperature as number;
      if ('humidity' in values) el.humidity = values.humidity as number;
      if (answer.mode !== 'none') {
        answer.update({ temperature: el.temperature ?? 25.0, humidity: el.humidity ?? 50.0 });
      }
    });

    return () => {
      if (answer.mode !== 'none') answer.release();
      // A refusal left a gap recorded; drop it so a rewire cannot keep
      // reporting a sensor the user has already moved.
      else releaseLineGap(componentId);
      unregisterSensorUpdate(componentId);
    };
  },
});

// ─── HX711 Load Cell Amplifier ────────────────────────────────────────────────

/**
 * HX711 — 24-bit ADC for load cells.
 *
 * Protocol:
 *  - DOUT LOW  = conversion ready
 *  - MCU reads 24 rising CLK edges → DOUT sends 24 bits MSB-first
 *  - 1 extra CLK pulse → gain 128 (channel A, default)
 *  - After 25th pulse falling edge: new conversion starts (DOUT → LOW after ~delay)
 *
 * Default weight: 100 g. Change via element.weight (grams).
 * Raw ADC = weight × 1000 (signed 24-bit two's complement).
 *
 * Taring: Arduino sketches typically call tare() first, which reads the
 * zero offset. This simulation always returns weight × 1000 as the raw value;
 * after taring with 0 g the sketch will correctly read any non-zero value.
 */
PartSimulationRegistry.register('hx711', {
  attachEvents: (element, simulator, getPin) => {
    const pinSCK = getPin('SCK');
    const pinDOUT = getPin('DOUT');
    if (pinSCK === null || pinDOUT === null) return () => {};

    let rawValue = rawFromWeight(element);
    let bitCount = 0;
    let finishing = false;

    function rawFromWeight(el: HTMLElement): number {
      const w = (el as any).weight ?? 100; // grams
      const raw = Math.round(w * 1000); // 24-bit fixed-point
      return Math.max(-8_388_608, Math.min(8_388_607, raw)) & 0xff_ffff;
    }

    // DOUT LOW = next conversion ready
    simulator.setPinState(pinDOUT, false);

    const unsub = (simulator as any).pinManager.onPinChange(
      pinSCK,
      (_: number, rising: boolean) => {
        if (rising) {
          // Rising edge: output the current bit (MSB first), then advance
          if (bitCount < 24) {
            const bit = (rawValue >> (23 - bitCount)) & 1;
            simulator.setPinState(pinDOUT, bit === 1);
            bitCount++;
          } else {
            // 25th pulse → gain select. DOUT driven HIGH (end of word)
            simulator.setPinState(pinDOUT, true);
            finishing = true;
          }
        } else {
          // Falling edge after the 25th pulse → conversion complete
          if (finishing) {
            finishing = false;
            bitCount = 0;
            rawValue = rawFromWeight(element);
            // DOUT LOW = new conversion ready (simulate ~10 ms conversion time)
            setTimeout(() => simulator.setPinState(pinDOUT, false), 10);
          }
        }
      },
    );

    return () => {
      unsub();
      simulator.setPinState(pinDOUT, true); // DOUT HIGH = device idle / power down
    };
  },
});

// ─── Infrared ────────────────────────────────────────────────────────────────

/**
 * The two IR parts, and the air between them.
 *
 * WHAT WAS WRONG. Both were dead, each in its own way, and neither failure
 * looked like a failure on the canvas:
 *
 *  - the receiver asked for a pin named `OUT` or `DATA`. `wokwi-ir-receiver`
 *    calls it `DAT`, so the lookup returned null and `attachEvents` returned
 *    an empty cleanup on its second line. Wired or not, the part did nothing.
 *  - the remote asked for a pin too. It is a remote control: it has no pins
 *    and never had. All it did was dispatch an `ir-signal` DOM event that
 *    nothing in the codebase listened for, with a command from a hand-written
 *    key table whose names the element never emits — while the element itself
 *    was already carrying the correct NEC code in `detail.irCode`.
 *  - and the pulse train was built out of `setTimeout(next, 0.562)`. A NEC
 *    mark is 560 us; setTimeout's floor is a millisecond and its nested clamp
 *    is four. No IR library could ever have decoded it.
 *
 * WHAT THEY ARE NOW. The remote transmits into `simulation/ir/irAir` — the
 * medium, with no wire and no geometry — and the receiver is a line-owning
 * model (`simulation/line/models/ir-nec`) that puts the envelope of whatever
 * it hears on its pin, on the guest's own cycle counter, at real NEC timing.
 * Several remotes and several receivers work at once, on any mix of boards,
 * because the air carries microseconds and each receiver converts them to its
 * own board's clock.
 */

/** A component property that may arrive as a number or as a typed string. */
function numProp(el: Record<string, unknown>, keys: string[], dflt: number): number {
  for (const k of keys) {
    const v = el[k];
    if (typeof v === 'number' && Number.isFinite(v)) return v;
    if (typeof v === 'string' && v.trim() !== '') {
      const n = v.trim().toLowerCase().startsWith('0x') ? parseInt(v, 16) : parseFloat(v);
      if (Number.isFinite(n)) return n;
    }
  }
  return dflt;
}

function textProp(el: Record<string, unknown>, key: string): string {
  const v = el[key];
  return typeof v === 'string' ? v : '';
}

/**
 * IR receiver — the demodulator can (a VS1838B, a TSOP38238) behind the lens.
 *
 * It owns its output pin through the line contract, so the board that hosts it
 * answers honestly: the model runs in the browser (AVR, RP2040, RP2350, the
 * esp32*js engines), it runs in the backend worker, or this board cannot host
 * it and the circuit check says why. A click on the part still transmits, as a
 * convenience for a canvas with no remote on it — the part IS the remote then,
 * sending its configured address and command.
 */
PartSimulationRegistry.register('ir-receiver', {
  attachEvents: (element, simulator, getPin, componentId) => {
    // `DAT` is what the element declares. The other two are accepted because
    // modules in the wild silk-screen them, and a saved project may carry
    // either spelling.
    const pin = getPin('DAT') ?? getPin('OUT') ?? getPin('DATA');
    if (pin === null) return () => {};

    const el = element as unknown as Record<string, unknown>;
    const channel = () => textProp(el, 'channel');
    /** Bumped on every transmission: the model fires on the CHANGE, so two
     *  identical presses in a row are still two frames. */
    let seq = 0;

    const answer = requestLine(
      simulator,
      {
        sensor_type: 'ir-nec',
        pin,
        address: numProp(el, ['irAddress', 'address'], 0x00),
        command: numProp(el, ['irCommand', 'command'], 0x45),
        channel: channel(),
      },
      { componentId },
    );

    /** Put a train on the pin. The air already decoded it; this only has to
     *  reproduce the envelope, so the raw pulses go over untouched — a remote
     *  speaking a protocol nothing here parses still reaches the sketch. */
    const deliver = (pulses: readonly IrPulse[]): boolean => {
      if (answer.mode === 'none') return false;
      answer.update({ pulses, seq: ++seq });
      return true;
    };

    const unlisten = listenIr(componentId, channel, (f: IrAirFrame) => deliver(f.pulses));

    // Clicking the receiver transmits its own configured code, so a canvas
    // with no remote on it is still testable. It goes through the air like
    // anything else, which is also what makes a second receiver hear it.
    const onClick = () => {
      emitIr({
        pulses: necEncode(
          numProp(el, ['irAddress', 'address'], 0x00),
          numProp(el, ['irCommand', 'command'], 0x45),
        ),
        sourceId: componentId,
        channel: channel(),
      });
    };
    element.addEventListener('click', onClick);

    // The sensor panel edits the SAME two values the element carries, so a
    // slider and the hex property in the inspector cannot drift apart and the
    // click path below reads whatever was set last.
    registerSensorUpdate(componentId, (values) => {
      if ('address' in values) el.irAddress = values.address;
      if ('command' in values) el.irCommand = values.command;
      // The panel's Send button transmits into the room rather than straight
      // onto the pin, so every OTHER receiver on this channel hears it too.
      if ('send' in values) onClick();
    });

    return () => {
      unlisten();
      element.removeEventListener('click', onClick);
      if (answer.mode !== 'none') answer.release();
      else releaseLineGap(componentId);
      unregisterSensorUpdate(componentId);
    };
  },
});

/**
 * IR remote control — a handset. It has no pins and connects to nothing; it
 * transmits into the room and whatever is listening hears it.
 *
 * The element already computes the NEC code for the button that was pressed
 * (`detail.irCode`) and it is the authority: the codes belong to the artwork
 * printed on its keys. The old hand-written table here disagreed with it on
 * every single key and was keyed on button names the element never emits.
 *
 * Holding a button repeats, exactly as a real remote does — a NEC repeat frame
 * every 108 ms, which is what makes a volume key ramp instead of stepping once.
 */
PartSimulationRegistry.register('ir-remote', {
  attachEvents: (element, _simulator, _getPin, componentId) => {
    const el = element as unknown as Record<string, unknown>;
    const channel = () => textProp(el, 'channel');
    /** The address the handset sends. NEC remotes each have their own; the
     *  element models one physical remote, so the property is on the part. */
    const address = () => numProp(el, ['irAddress', 'address'], 0x00);

    let repeatTimer: ReturnType<typeof setInterval> | null = null;
    const stopRepeat = () => {
      if (repeatTimer !== null) {
        clearInterval(repeatTimer);
        repeatTimer = null;
      }
    };

    const send = (pulses: readonly IrPulse[]) => {
      const taken = emitIr({ pulses, sourceId: componentId, channel: channel() });
      if (taken === 0) {
        // "The button does nothing" and "nothing was listening" look the same
        // on a canvas, and only one of them is the user's mistake.
        console.warn(
          `[ir] ${componentId} transmitted and no receiver took it` +
            (channel() ? ` on channel '${channel()}'` : ''),
        );
      }
    };

    const onButtonPress = (e: Event) => {
      const detail = (e as CustomEvent).detail ?? {};
      const irCode = Number(detail.irCode);
      if (!Number.isFinite(irCode)) return;
      stopRepeat();
      send(necEncode(address(), irCode & 0xff));
      // A held key sends repeat frames, carrying no data, every 108 ms.
      repeatTimer = setInterval(() => send(necEncodeRepeat()), NEC_REPEAT_PERIOD_MS);
    };
    const onButtonRelease = () => stopRepeat();

    element.addEventListener('button-press', onButtonPress);
    element.addEventListener('button-release', onButtonRelease);
    // A button-press with no release (the element focuses and blurs, a pointer
    // leaves the SVG) must not repeat forever.
    element.addEventListener('mouseleave', onButtonRelease);

    return () => {
      stopRepeat();
      element.removeEventListener('button-press', onButtonPress);
      element.removeEventListener('button-release', onButtonRelease);
      element.removeEventListener('mouseleave', onButtonRelease);
    };
  },
});

// ─── MicroSD Card ─────────────────────────────────────────────────────────────

/**
 * MicroSD card: the canvas part of the generic SD-over-SPI card.
 *
 * A responder on the bus fabric: it is on the bus its SCK/DI/DO/CS wires put
 * it on, whatever board and whatever engine, and it only ever sees the frames
 * clocked while its own chip select is active.
 *
 * The protocol is NOT here. One model serves this part, a board's built-in
 * slot and the portable artifact a remote worker runs
 * (`parts/sdSpiCard.ts` and `buses/models/microsd.c`), because three
 * hand-kept copies of it had already drifted apart and each drift was a card
 * that mounted on one engine and not another. What this part owns is the
 * interface: the image the project's files are baked into
 * (`element.sdImageData`), the reader the card panel lists them through, and
 * the pad names the fabric walks the wires from.
 */
// Fixed card capacity (mirrors Wokwi's "no size attribute" model). The backing
// store is SPARSE — only written/loaded blocks allocate — so the advertised
// capacity is free in RAM. Adjustable here; not exposed to the user.
const SD_CARD_BYTES = 64 * 1024 * 1024; // 64 MB

PartSimulationRegistry.register('microsd-card', {
  attachEvents: (element, _simulator, _getPin, componentId) => {
    const el = element as any;

    // The card model is shared with a board's built-in slot and with the
    // portable model a remote worker runs, so a fix lands in one place
    // (project board-buses-2026-09, F4). This part is the INTERFACE: the image
    // the user's files are baked into, the panel that lists them, and the pins
    // the silkscreen prints.
    const card = new SdSpiCard(null, SD_CARD_BYTES);

    // Optional pre-built FAT image (DynamicComponent sets element.sdImageData
    // from the project's files plus the panel's uploads). How big it was is
    // what the dump pads back to, so a FAT parser reading the card sees the
    // whole volume and not just the blocks somebody touched.
    let imageBytes = 0;
    {
      const raw = el.sdImageData;
      const bytes: Uint8Array | null =
        raw instanceof Uint8Array
          ? raw
          : raw instanceof ArrayBuffer
            ? new Uint8Array(raw)
            : Array.isArray(raw)
              ? Uint8Array.from(raw)
              : null;
      if (bytes) {
        imageBytes = bytes.length;
        card.loadImage(bytes);
      }
    }

    const owner = componentId ?? (el?.id as string) ?? 'microsd-card';
    // The panel asks by owner. `fromPart` says this card is a component on the
    // canvas, so a panel opened on the card itself finds it without being told
    // which component it is.
    const unpublish = registerSdImageReader(
      owner,
      () => {
        const image = card.dumpImage(imageBytes);
        // Nothing mounted yet reads as no card, which is what the panel shows
        // for a slot it has never seen a byte from.
        return image.length > 0 ? image : null;
      },
      { fromPart: true },
    );

    // The card is also a RESPONDER on a board whose CPU is in a QEMU worker,
    // and there it has to run beside the guest: the worker reads MISO for a
    // byte before this tab has seen the byte (D-004). Start the fetch of the
    // portable model now; the map is published again when the bytes land.
    loadSdBusChip();

    const handle = attachSpiDevice(
      {
        owner,
        componentId,
        pins: { sck: 'SCK', mosi: 'DI', miso: 'DO', cs: 'CS' },
        // SD cards clock on modes 0 and 3.
        modes: [0, 3],
        // CS (pin 1, DAT3) carries the card's own pull-up, so a card whose CS
        // nothing drives reads as deselected and stays quiet.
        csWhenFloating: 'deselected',
        remoteModel: () => sdCardRemoteModel(card, imageBytes),
        hasRemoteModel: () => sdCardHasRemoteModel(card, imageBytes),
        // What the guest writes on a remote board comes back as spans: the
        // worker no longer relays the bytes of a card transaction no sink can
        // see, and this copy is what the panel lists.
        remoteBlobWrite: sdCardRemoteBlobWrite(card),
      },
      sdSpiFabricDevice(card),
    );

    return () => {
      handle.dispose();
      unpublish();
      card.setCs(false);
    };
  },
});

// ─── BMP280 Barometric Pressure / Temperature Sensor ─────────────────────────

/**
 * BMP280 — I2C barometric pressure + temperature sensor.
 *
 * Addresses:
 *   0x76 (SDO pin pulled LOW, default)
 *   0x77 (SDO pin pulled HIGH — set element.address = '0x77', or
 *         element.i2cAddress, which is the name a Grove brick sets)
 *
 * The element may expose `temperature` (°C) and `pressure` (hPa) properties
 * that are read on attach and forwarded to the virtual device.
 *
 * The virtual device uses the BMP280 datasheet calibration example to compute
 * raw ADC values for any desired temperature/pressure combination, so Arduino
 * sketches using Adafruit_BMP280 or Bosch's reference driver receive correct
 * compensated readings.
 */
PartSimulationRegistry.register('bmp280', {
  attachEvents: (element, simulator, _getPin, componentId) => {
    const el = element as any;
    // SDO selects one of two addresses, and the chip has no other. The Grove
    // BMP280 sets `i2cAddress`: read as `address` only, the model sat at 0x76
    // while Seeed's library asks 0x77 and waits for an answer with no timeout.
    const addr = parseI2cAddress(el.i2cAddress ?? el.address, 0x76) === 0x77 ? 0x77 : 0x76;
    // An unset value starts where the panel and the property dialog say it
    // does (24 C, 1013.25 hPa), not at a literal of this part's own.
    const initTemp =
      el.temperature !== undefined
        ? parseFloat(el.temperature)
        : sensorControlDefault('bmp280', 'temperature', 24);
    const initPressure =
      el.pressure !== undefined
        ? parseFloat(el.pressure)
        : sensorControlDefault('bmp280', 'pressure', 1013.25);

    const dev = new VirtualBMP280(addr);
    dev.temperatureC = initTemp;
    dev.pressureHPa = initPressure;
    const part = attachI2cPart({
      simulator,
      componentId,
      device: dev,
      worker: { type: 'bmp280', props: { temperature: initTemp, pressure: initPressure } },
    });

    registerSensorUpdate(componentId, (values) => {
      part.updateWorker(values);
      if ('temperature' in values) dev.temperatureC = values.temperature as number;
      if ('pressure' in values) dev.pressureHPa = values.pressure as number;
    });

    return () => {
      part.dispose();
      unregisterSensorUpdate(componentId);
    };
  },
});

// ─── DS3231 Real-Time Clock ───────────────────────────────────────────────────

/**
 * DS3231 — I2C RTC with on-chip temperature sensor (address 0x68).
 *
 * Returns the browser's current system time as BCD in registers 0x00–0x06,
 * identical to DS1307 for the time registers. Additionally exposes:
 *   0x0E  Control register
 *   0x0F  Status register (OSF cleared)
 *   0x11  Temperature MSB (integer °C, signed)
 *   0x12  Temperature LSB (fractional, 0.25°C per bit in bits 7:6)
 *
 * Ambient temperature defaults to 25°C; override via `element.temperature`.
 */
PartSimulationRegistry.register('ds3231', {
  attachEvents: (element, simulator, _getPin, componentId) => {
    const el = element as any;
    const initTemp = el.temperature !== undefined ? parseFloat(el.temperature) : 25.0;

    const dev = new VirtualDS3231();
    dev.temperatureC = initTemp;
    const part = attachI2cPart({
      simulator,
      componentId,
      device: dev,
      worker: { type: 'ds3231', props: { temperature: initTemp } },
    });
    registerSensorUpdate(componentId, (values) => {
      part.updateWorker(values);
      if ('temperature' in values) dev.temperatureC = values.temperature as number;
    });
    return () => {
      part.dispose();
      unregisterSensorUpdate(componentId);
    };
  },
});

// ─── PCF8574 I/O Expander ────────────────────────────────────────────────────

/**
 * PCF8574 — I2C 8-bit quasi-bidirectional I/O expander.
 *
 * Default address: 0x27 (all three address pins HIGH — typical LCD backpack).
 * Override with `element.i2cAddress` (e.g. '0x20', '0x3F').
 *
 * `element.portState` (0–255) sets the external input state visible to the
 * Arduino on a read. Defaults to 0xFF (all pins pulled high / floating input).
 *
 * Writes from the Arduino update `dev.outputLatch` and fire `dev.onWrite`
 * which sets `element.value` so wokwi-LCD-I2C or similar elements can render.
 */
PartSimulationRegistry.register('pcf8574', {
  attachEvents: (element, simulator, _getPin, componentId) => {
    const el = element as any;

    // Parse address from element property (accepts '0x27', '39', or numeric)
    let addr = 0x27;
    if (el.i2cAddress !== undefined) {
      const raw = String(el.i2cAddress).trim();
      const parsed =
        raw.startsWith('0x') || raw.startsWith('0X') ? parseInt(raw, 16) : parseInt(raw, 10);
      if (!isNaN(parsed)) addr = parsed;
    }

    const dev = new VirtualPCF8574(addr);
    if (el.portState !== undefined) dev.portState = Number(el.portState) & 0xff;
    dev.onWrite = (value: number) => {
      el.value = value;
    };

    const part = attachI2cPart({
      simulator,
      componentId,
      device: dev,
      worker: {
        type: 'pcf8574',
        echo: (data: number[]) => {
          if (data.length > 0) dev.writeByte(data[0]);
        },
      },
    });
    return () => part.dispose();
  },
});

// ─── LCD1602 / LCD2004 with I2C backpack (PCF8574 + HD44780) ────────────────

/**
 * Common parser for an I2C address property coming from a wokwi-element
 * (the metadata exposes `i2cAddress` as a text control; users type
 * "0x27", "39", or just the raw number).
 */
function parseI2cAddress(raw: unknown, fallback: number): number {
  if (raw === undefined || raw === null) return fallback;
  if (typeof raw === 'number' && !isNaN(raw)) return raw & 0x7f;
  const s = String(raw).trim();
  if (!s) return fallback;
  const parsed = s.toLowerCase().startsWith('0x') ? parseInt(s, 16) : parseInt(s, 10);
  return isNaN(parsed) ? fallback : parsed & 0x7f;
}

/**
 * Build a part attach function for an LCD with an I2C backpack.  The
 * same logic applies to LCD1602 (16×2) and LCD2004 (20×4); only the
 * geometry differs.
 *
 * On attach:
 *  1. Force the underlying `wokwi-lcd1602` / `wokwi-lcd2004` element
 *     into I2C-pinout mode (`pins='i2c'`) so the user sees the
 *     correct 4-pin backpack header.
 *  2. Pre-fill `characters` with spaces so the screen is clean before
 *     the sketch issues its first Clear command.
 *  3. Create a `VirtualPCF8574` at the configured address.
 *  4. Pipe `pcf.onWrite` → `HD44780Decoder.feedPCF8574Byte`.
 *  5. Reflect the decoder's `characters` + `backlight` snapshots back
 *     onto the element's reactive properties.
 *
 * Works on AVR, RP2040, and the ESP32 backend (same trifurcation
 * pattern as other I2C parts above).
 */
function makeI2cLcdAttach(cols: number, rows: number) {
  return (
    element: HTMLElement,
    simulator: unknown,
    _getPin: (name: string) => number | null,
    componentId: string,
  ): (() => void) => {
    const el = element as any;

    const addr = parseI2cAddress(el.i2cAddress ?? el.address, 0x27);

    // Switch the underlying LCD element to I2C pin mode + a clean
    // characters buffer.  The host wokwi element re-renders on
    // attribute change.
    try {
      el.pins = 'i2c';
    } catch {
      /* read-only on some implementations — ignore */
    }
    const blankGrid = new Uint8Array(cols * rows).fill(0x20);
    el.characters = blankGrid;
    if (el.backlight === undefined) el.backlight = true;

    const decoder = new HD44780Decoder({ cols, rows });
    decoder.onCharsChange = (chars) => {
      // wokwi-lcd1602 accepts both number[] and Uint8Array.  Use Uint8Array
      // so Lit's change detection sees a new reference.
      el.characters = Uint8Array.from(chars);
    };
    decoder.onBacklightChange = (on) => {
      el.backlight = on;
    };
    decoder.onCursorChange = (snap) => {
      el.cursorX = snap.cursorCol;
      el.cursorY = snap.cursorRow;
      el.cursor = snap.cursorOn;
      el.blink = snap.cursorBlink;
    };

    const pcf = new VirtualPCF8574(addr);
    pcf.onWrite = (v: number) => decoder.feedPCF8574Byte(v);

    // On a QEMU board the worker's expander echoes each write phase, and the
    // decoder here takes the bytes as the backpack would.
    const part = attachI2cPart({
      simulator,
      componentId,
      device: pcf,
      worker: {
        type: 'pcf8574',
        echo: (data: number[]) => {
          for (const b of data) decoder.feedPCF8574Byte(b);
        },
      },
    });
    return () => {
      part.dispose();
      decoder.reset();
    };
  };
}

/**
 * LCD 16×2 with PCF8574 I2C backpack — the classic "I2C LCD" you buy
 * in a single piece on AliExpress.  Default address 0x27.
 */
PartSimulationRegistry.register('lcd1602-i2c', {
  attachEvents: makeI2cLcdAttach(16, 2),
});

/**
 * LCD 20×4 with PCF8574 I2C backpack.  Same protocol; uses the 2004
 * DDRAM row offsets (0x00, 0x40, 0x14, 0x54).
 */
PartSimulationRegistry.register('lcd2004-i2c', {
  attachEvents: makeI2cLcdAttach(20, 4),
});
