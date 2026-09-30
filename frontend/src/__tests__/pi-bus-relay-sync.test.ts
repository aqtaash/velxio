// @vitest-environment jsdom
/**
 * pi-bus-relay-sync.test.ts — the board side of the backend bus relay.
 *
 * A Linux guest reads its I2C / SPI parts through the backend, which answers
 * what it can without asking the tab (an absent address, a register-file
 * device it holds a copy of) and relays the rest. For that the backend needs
 * the bus map and fresh registers, and only a backend that relays should get
 * them. Pinned here:
 *
 *  - nothing is published until the backend announces `bus_relay`; then the
 *    map goes out once, a register-file device WITH its 256 registers and a
 *    device that must be asked WITHOUT, and the board is marked busRelay;
 *  - afterwards only changes travel: a device whose registers changed pushes
 *    its registers, a device added or removed republishes the map, an
 *    unchanged bus sends nothing (compared byte for byte: the ESP32 proxy's
 *    sampled hash would miss a measurement register);
 *  - a write the backend sends on is answered at once with the device's
 *    registers and the number the backend gave the write, also when the write
 *    left the registers as they were (a bit that clears itself): the backend
 *    holds its own copy back until then, and asks this tab for the reads;
 *  - every later push repeats that number, so the backend can drop a push
 *    that was computed before a guest's write instead of rolling it back;
 *  - a backend that numbers nothing is answered all the same, and nothing is
 *    pushed for the in-browser engine, which has no backend;
 *  - a disconnect stops the sync;
 *  - the real bridge answers a relayed read and stays silent on a `noreply`
 *    write, which the backend holds no request open for.
 *
 * The backend half is pro/backend/tests/unit/test_pi_bus_relay.py.
 */
import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';

const sent = {
  topology: [] as unknown[],
  regs: [] as Array<[bus: number, addr: number, regs: string, seq: number | undefined]>,
};

vi.mock('../simulation/RaspberryPi3Bridge', () => ({
  RaspberryPi3Bridge: class {
    boardId: string;
    boardKind: string;
    connected = false;
    onSerialData: unknown = null;
    onPinChange: unknown = null;
    onPinPull: unknown = null;
    onBusRequest: unknown = null;
    onBusRelay: ((v: number) => void) | null = null;
    onGpioPwm: unknown = null;
    onBooted: unknown = null;
    onDisconnected: (() => void) | null = null;
    onError: unknown = null;
    quietBootDefault = false;
    quietBootLabel = '';
    constructor(id: string, kind: string) {
      this.boardId = id;
      this.boardKind = kind;
    }
    connect() {}
    disconnect() {}
    sendPinEvent() {}
    sendBusTopology(t: unknown) {
      sent.topology.push(t);
    }
    sendBusRegs(bus: number, addr: number, regs: string, seq?: number) {
      sent.regs.push([bus, addr, regs, seq]);
    }
  },
}));

import { useSimulatorStore, getBoardSimulator, getBoardBridge } from '../store/useSimulatorStore';
import type { PiBridgeShim } from '../simulation/PiBridgeShim';
import { VirtualBMP280, VirtualDS3231, type I2CDevice } from '../simulation/I2CBusManager';
import { PartSimulationRegistry } from '../simulation/parts';
import { attachI2cTarget, busRegistry, createStoreNetResolver } from '../simulation/buses';
import type { NetResolver, PinRef, ResolvedPin } from '../simulation/buses/types';
import { i2cTargetOf } from '../simulation/parts/i2cPart';

type MockBridge = {
  onBusRelay: ((v: number) => void) | null;
  onBusRequest: ((rid: number, line: string) => string | null) | null;
  onDisconnected: (() => void) | null;
};

function addPi() {
  const id = useSimulatorStore.getState().addBoard('raspberry-pi-4' as never, 100, 100);
  return {
    id,
    shim: getBoardSimulator(id) as PiBridgeShim,
    bridge: getBoardBridge(id) as unknown as MockBridge,
  };
}

/** Wire a part's SDA/SCL to the Pi's GPIO2/GPIO3 (/dev/i2c-1), as the canvas does. */
function wireI2c1(boardId: string, componentId: string): void {
  useSimulatorStore.setState((st) => ({
    wires: [
      ...st.wires.filter((w) => w.start.componentId !== componentId),
      ...(['SDA', 'SCL'] as const).map((pinName) => ({
        id: `${componentId}-${pinName}`,
        start: { componentId, pinName, x: 0, y: 0 },
        end: { componentId: boardId, pinName: pinName === 'SDA' ? 'GPIO2' : 'GPIO3', x: 0, y: 0 },
        waypoints: [],
        color: '#0a0',
      })),
    ],
  }) as never);
}

let devSeq = 0;
/** A device model on /dev/i2c-1 by its wiring, the way a part registers (board-buses F5). */
function putOnI2c1(boardId: string, device: I2CDevice): () => void {
  const owner = `dev-${device.address.toString(16)}-${++devSeq}`;
  wireI2c1(boardId, owner);
  const h = attachI2cTarget(
    { owner, componentId: owner, pins: { sda: 'SDA', scl: 'SCL' }, addresses: [device.address] },
    i2cTargetOf(device),
  );
  return () => h.dispose();
}

/**
 * A register file with a bit that clears itself, the way a chip's soft reset
 * bit does: writing bit 7 of register 0x10 puts every register back to its
 * power-on value and is never stored. The register reads afterwards what it
 * read before the write, so the write leaves the dump exactly as it was.
 */
class ResettableRegs implements I2CDevice {
  static readonly CTRL = 0x10;
  static readonly CTRL_POWER_ON = 0x40;
  address = 0x29;
  readonly registers = new Uint8Array(256);
  private ptr = 0;
  private first = true;
  constructor() {
    this.powerOn();
  }
  private powerOn(): void {
    this.registers.fill(0);
    this.registers[ResettableRegs.CTRL] = ResettableRegs.CTRL_POWER_ON;
  }
  writeByte(value: number): boolean {
    if (this.first) {
      this.ptr = value;
      this.first = false;
      return true;
    }
    if (this.ptr === ResettableRegs.CTRL && value & 0x80) this.powerOn();
    else this.registers[this.ptr] = value;
    this.ptr = (this.ptr + 1) & 0xff;
    return true;
  }
  readByte(): number {
    const v = this.registers[this.ptr];
    this.ptr = (this.ptr + 1) & 0xff;
    return v;
  }
  stop(): void {
    this.first = true;
  }
  dumpRegisters(): Uint8Array {
    return this.registers.slice();
  }
}

/** One register out of a pushed dump. */
const regOf = (dump: string, reg: number): string => dump.slice(reg * 2, reg * 2 + 2);

beforeEach(() => {
  vi.useFakeTimers();
  sent.topology.length = 0;
  sent.regs.length = 0;
});
afterEach(() => {
  vi.useRealTimers();
});

describe('the bus map goes to a relaying backend, and only then', () => {
  it('nothing is published before bus_relay; once announced, the map and the busRelay flag', () => {
    const { id, shim, bridge } = addPi();
    putOnI2c1(id, new VirtualBMP280(0x76));
    // A device that cannot export its registers (a command-driven sensor):
    // the backend has to ask the tab for it every time.
    putOnI2c1(id, { address: 0x44, writeByte: () => true, readByte: () => 0 });
    // The part is on the bus its wires reach: GPIO2/3, /dev/i2c-1.
    wireI2c1(id, 'mpu-1');
    const cleanup = PartSimulationRegistry.get('mpu6050')!.attachEvents!(
      document.createElement('div'),
      shim as never,
      () => null,
      'mpu-1',
    );
    vi.advanceTimersByTime(1000);
    expect(sent.topology).toEqual([]);

    bridge.onBusRelay!(1);
    expect(sent.topology).toHaveLength(1);
    const topo = sent.topology[0] as { i2c: Array<{ bus: number; addr: number; regs: string | null }>; spi: { attached: boolean } };
    const byAddr = new Map(topo.i2c.map((d) => [d.addr, d]));
    expect(byAddr.get(0x76)?.bus).toBe(1);
    expect(byAddr.get(0x76)?.regs).toMatch(/^[0-9a-f]{512}$/);
    // 0xD0 is the BMP280 chip id register: its copy must say 0x58.
    expect(byAddr.get(0x76)?.regs?.slice(0xd0 * 2, 0xd0 * 2 + 2)).toBe('58');
    // The MPU6050 is a register file too, so it travels with its registers
    // (WHO_AM_I at 0x75 is its own address).
    expect(byAddr.get(0x68)?.regs?.slice(0x75 * 2, 0x75 * 2 + 2)).toBe('68');
    expect(byAddr.get(0x44)?.regs).toBeNull(); // no dumpRegisters: asked, not copied
    expect(topo.spi.attached).toBe(false);
    expect(useSimulatorStore.getState().boards.find((b) => b.id === id)?.busRelay).toBe(true);
    cleanup();
  });
});

describe('afterwards only changes travel', () => {
  it('a register write pushes that device once; an idle bus sends nothing', () => {
    const { id, shim, bridge } = addPi();
    putOnI2c1(id, new VirtualBMP280(0x76));
    bridge.onBusRelay!(1);
    vi.advanceTimersByTime(1000);
    expect(sent.regs).toEqual([]);

    // ctrl_meas (0xF4) is a plain read/write register on the model.
    expect(shim.i2cTransfer(0x76, [0xf4, 0x27], 0)).toEqual([]);
    vi.advanceTimersByTime(260);
    expect(sent.regs).toHaveLength(1);
    expect(sent.regs[0][0]).toBe(1);
    expect(sent.regs[0][1]).toBe(0x76);
    expect(sent.regs[0][2].slice(0xf4 * 2, 0xf4 * 2 + 2)).toBe('27');

    vi.advanceTimersByTime(1000);
    expect(sent.regs).toHaveLength(1);
  });

  it('a device added after the announcement republishes the map', () => {
    const { id, shim, bridge } = addPi();
    putOnI2c1(id, new VirtualBMP280(0x76));
    bridge.onBusRelay!(1);
    expect(sent.topology).toHaveLength(1);
    putOnI2c1(id, new VirtualDS3231());
    vi.advanceTimersByTime(260);
    void shim;
    expect(sent.topology).toHaveLength(2);
    const addrs = (sent.topology[1] as { i2c: Array<{ addr: number }> }).i2c.map((d) => d.addr).sort();
    expect(addrs).toEqual([0x68, 0x76]);
  });

  it('a chip the fabric put on a controller\'s SCK net flips spi.attached in the next map', () => {
    // It used to be enough to assign `shim.spi.onByte`. There is no such
    // channel now: the gate counts devices the fabric placed on SPI0 or SPI1,
    // so the chip is wired to the header pins its model would really sit on
    // (SCK 11, MOSI 10, MISO 9, CE0 8).
    const { id, shim, bridge } = addPi();
    bridge.onBusRelay!(1);
    const wires = new Map<string, ResolvedPin>();
    const resolver: NetResolver = {
      resolve: (ref: PinRef): ResolvedPin =>
        ref.kind === 'board'
          ? { kind: 'board', boardId: ref.boardId, pin: ref.pin }
          : wires.get(`${ref.componentId}:${ref.pinName}`) ?? { kind: 'floating' },
      boardKind: (b: string) => (b === id ? 'raspberry-pi-4' : undefined),
      boards: () => [id],
    };
    for (const [name, pin] of Object.entries({ SCK: 11, MOSI: 10, MISO: 9, CS: 8 })) {
      wires.set(`adc:${name}`, { kind: 'board', boardId: id, pin });
    }
    busRegistry.setResolver(resolver);
    try {
      const h = busRegistry.attachSpi(
        { owner: 'adc', pins: { sck: 'SCK', mosi: 'MOSI', miso: 'MISO', cs: 'CS' } },
        { transfer: () => 0 },
      );
      vi.advanceTimersByTime(260);
      expect((sent.topology.at(-1) as { spi: { attached: boolean } }).spi.attached).toBe(true);
      h.dispose();
    } finally {
      busRegistry.setResolver(createStoreNetResolver(() => useSimulatorStore.getState()));
    }
    void shim;
  });

  it('a disconnect stops the sync', () => {
    const { id, shim, bridge } = addPi();
    putOnI2c1(id, new VirtualBMP280(0x76));
    bridge.onBusRelay!(1);
    bridge.onDisconnected!();
    shim.i2cTransfer(0x76, [0xf4, 0x55], 0);
    putOnI2c1(id, new VirtualDS3231());
    vi.advanceTimersByTime(2000);
    expect(sent.regs).toEqual([]);
    expect(sent.topology).toHaveLength(1);
  });
});

describe('a write the relay sends on is answered with the registers', () => {
  it('a forwarded write whose effect leaves the dump unchanged still pushes, with its seq', () => {
    const { id, bridge } = addPi();
    const chip = new ResettableRegs();
    putOnI2c1(id, chip);
    bridge.onBusRelay!(1);
    const published = (sent.topology[0] as { i2c: Array<{ addr: number; regs: string }> }).i2c.find(
      (d) => d.addr === 0x29,
    )!.regs;
    vi.advanceTimersByTime(1000);
    expect(sent.regs).toEqual([]);

    // The guest sets the reset bit on top of what it read (0x40 | 0x80), as a
    // read-modify-write does. The relay's copy now says 0xc0.
    expect(bridge.onBusRequest!(1, 'I2C 1 29 W 10c0 #3')).toBe('I2C_DATA 1 29');
    expect(chip.registers[ResettableRegs.CTRL]).toBe(0x40);

    // No tick has run: the push is the answer to the write, not the timer's.
    expect(sent.regs).toHaveLength(1);
    const [bus, addr, regs, seq] = sent.regs[0];
    expect([bus, addr, seq]).toEqual([1, 0x29, 3]);
    expect(regs).toBe(published);
    expect(regOf(regs, ResettableRegs.CTRL)).toBe('40');

    // Once. The tick has nothing to add to it.
    vi.advanceTimersByTime(1000);
    expect(sent.regs).toHaveLength(1);
  });

  it('a stale push never rolls back: every push says which write it has seen', () => {
    const { id, bridge } = addPi();
    const chip = new ResettableRegs();
    putOnI2c1(id, chip);
    bridge.onBusRelay!(1);

    // A slider moves before any write arrived: this push knows of none, and a
    // relay whose guest wrote in the meantime drops it.
    chip.registers[0x20] = 0x11;
    vi.advanceTimersByTime(260);
    expect(sent.regs.map((r) => [regOf(r[2], 0x20), r[3]])).toEqual([['11', undefined]]);

    // Write 7 arrives and is answered as 7.
    bridge.onBusRequest!(1, 'I2C 1 29 W 2122 #7');
    expect(sent.regs.map((r) => r[3])).toEqual([undefined, 7]);
    expect(regOf(sent.regs[1][2], 0x21)).toBe('22');

    // From then on the timer's pushes have seen write 7 too...
    chip.registers[0x20] = 0x33;
    vi.advanceTimersByTime(260);
    expect(sent.regs.map((r) => [regOf(r[2], 0x20), r[3]])).toEqual([
      ['11', undefined],
      ['11', 7],
      ['33', 7],
    ]);

    // ...and so has a map republished for another reason.
    putOnI2c1(id, new VirtualBMP280(0x76));
    vi.advanceTimersByTime(260);
    const map = sent.topology.at(-1) as { i2c: Array<{ addr: number; seq?: number }> };
    expect(map.i2c.map((d) => [d.addr, d.seq])).toEqual([
      [0x29, 7],
      [0x76, undefined],
    ]);
  });

  it('the number is not data: the write that lands is the bytes before it', () => {
    const { id, bridge } = addPi();
    const chip = new ResettableRegs();
    putOnI2c1(id, chip);
    bridge.onBusRelay!(1);
    expect(bridge.onBusRequest!(1, 'I2C 1 29 W 3055 #12')).toBe('I2C_DATA 1 29');
    expect(Array.from(chip.registers.slice(0x30, 0x33))).toEqual([0x55, 0, 0]);
    // The relay asks the reads of a device it holds back as `T <ptr> <n>`.
    expect(bridge.onBusRequest!(2, 'I2C 1 29 T 30 2')).toBe('I2C_DATA 1 29 5500');
    // An acknowledged write is numbered the same way.
    expect(bridge.onBusRequest!(3, 'I2C 1 29 T 3166 0 #13')).toBe('I2C_DATA 1 29');
    expect(chip.registers[0x31]).toBe(0x66);
    expect(sent.regs.map((r) => r[3])).toEqual([12, 13]);
  });

  it('a relay that numbers nothing is answered too, without a number', () => {
    const { id, bridge } = addPi();
    putOnI2c1(id, new ResettableRegs());
    bridge.onBusRelay!(1);
    bridge.onBusRequest!(1, 'I2C 1 29 W 10c0');
    expect(sent.regs).toHaveLength(1);
    expect(sent.regs[0][3]).toBeUndefined();
    expect(regOf(sent.regs[0][2], ResettableRegs.CTRL)).toBe('40');
  });

  it('a read, or a write of the register pointer alone, pushes nothing', () => {
    const { id, bridge } = addPi();
    putOnI2c1(id, new ResettableRegs());
    bridge.onBusRelay!(1);
    expect(bridge.onBusRequest!(1, 'I2C 1 29 T 10 1')).toBe('I2C_DATA 1 29 40');
    expect(bridge.onBusRequest!(2, 'I2C 1 29 W 10')).toBe('I2C_DATA 1 29');
    expect(bridge.onBusRequest!(3, 'I2C 1 29 R 1')).toBe('I2C_DATA 1 29 40');
    expect(sent.regs).toEqual([]);
  });

  it('a device that hands over no registers has none to push', () => {
    const { id, bridge } = addPi();
    putOnI2c1(id, { address: 0x44, writeByte: () => true, readByte: () => 0 });
    bridge.onBusRelay!(1);
    expect(bridge.onBusRequest!(1, 'I2C 1 44 W 2400 #1')).toBe('I2C_DATA 1 44');
    expect(sent.regs).toEqual([]);
  });

  it('nothing is pushed while no relay listens', () => {
    const { id, shim, bridge } = addPi();
    const chip = new ResettableRegs();
    putOnI2c1(id, chip);
    // The in-browser engine sends the same lines, to no backend at all.
    expect(shim.answerBusLine('I2C 1 29 W 2077')).toBe('I2C_DATA 1 29');
    expect(chip.registers[0x20]).toBe(0x77);
    expect(sent.regs).toEqual([]);

    bridge.onBusRelay!(1);
    bridge.onDisconnected!();
    shim.answerBusLine('I2C 1 29 W 2078 #4');
    expect(sent.regs).toEqual([]);
  });

  it('a new guest starts from no number', () => {
    const { id, bridge } = addPi();
    const chip = new ResettableRegs();
    putOnI2c1(id, chip);
    bridge.onBusRelay!(1);
    bridge.onBusRequest!(1, 'I2C 1 29 W 2101 #9');
    expect(sent.regs.at(-1)?.[3]).toBe(9);

    // Stop and Run: the next guest's relay counts from zero, and a 9 kept
    // from this one would pass for a write that relay has not made yet.
    bridge.onBusRelay!(1);
    const map = sent.topology.at(-1) as { version: number; i2c: Array<{ addr: number; seq?: number }> };
    expect(map.version).toBe(2);
    expect(map.i2c.map((d) => [d.addr, d.seq])).toEqual([[0x29, undefined]]);
    chip.registers[0x20] = 0x44;
    vi.advanceTimersByTime(260);
    expect(sent.regs.at(-1)?.[3]).toBeUndefined();
  });
});

describe('the real bridge and a noreply write', () => {
  /** The real bridge on a socket that records what it is sent. */
  async function onFakeSocket(
    run: (
      bridge: import('../simulation/RaspberryPi3Bridge').RaspberryPi3Bridge,
      frames: unknown[],
      deliver: (type: string, data: unknown) => void,
    ) => void,
  ): Promise<void> {
    const { RaspberryPi3Bridge } = await vi.importActual<typeof import('../simulation/RaspberryPi3Bridge')>(
      '../simulation/RaspberryPi3Bridge',
    );
    const frames: unknown[] = [];
    class FakeWS {
      static OPEN = 1;
      readyState = 1;
      onopen: (() => void) | null = null;
      onmessage: ((e: { data: string }) => void) | null = null;
      onclose: (() => void) | null = null;
      onerror: (() => void) | null = null;
      constructor(_url: string) {
        sockets.push(this);
      }
      send(s: string) {
        frames.push(JSON.parse(s));
      }
      close() {}
    }
    const sockets: FakeWS[] = [];
    const realWS = globalThis.WebSocket;
    (globalThis as { WebSocket: unknown }).WebSocket = FakeWS;
    try {
      const bridge = new RaspberryPi3Bridge('raspberry-pi-4', 'raspberry-pi-4');
      bridge.connect();
      const ws = sockets[0];
      ws.onopen?.();
      frames.length = 0;
      run(bridge, frames, (type, data) => ws.onmessage?.({ data: JSON.stringify({ type, data }) }));
    } finally {
      (globalThis as { WebSocket: unknown }).WebSocket = realWS;
    }
  }

  it('answers a relayed read, stays silent on a noreply write, announces bus_relay', async () => {
    await onFakeSocket((bridge, frames, deliver) => {
      const relayed: number[] = [];
      bridge.onBusRequest = (_rid, line) => (line.includes(' T ') ? 'I2C_DATA 1 68 68' : null);
      bridge.onBusRelay = (v) => relayed.push(v);
      // What the relay sends for a device it mirrors: the write as the guest
      // made it, numbered, and the read that follows it, asked.
      deliver('pi_bus_request', { rid: 7, line: 'I2C 1 68 W 6b00 #1', noreply: true });
      deliver('pi_bus_request', { rid: 8, line: 'I2C 1 68 T 75 1' });
      deliver('system', { event: 'bus_relay', version: 1 });
      expect(frames).toEqual([{ type: 'pi_bus_reply', data: { rid: 8, line: 'I2C_DATA 1 68 68' } }]);
      expect(relayed).toEqual([1]);
    });
  });

  it('a push names the write it has seen only when there is one', async () => {
    await onFakeSocket((bridge, frames) => {
      bridge.sendBusRegs(1, 0x68, 'ab', 5);
      bridge.sendBusRegs(1, 0x68, 'cd');
      expect(frames).toEqual([
        { type: 'pi_bus_regs', data: { bus: 1, addr: 0x68, regs: 'ab', seq: 5 } },
        { type: 'pi_bus_regs', data: { bus: 1, addr: 0x68, regs: 'cd' } },
      ]);
    });
  });
});
