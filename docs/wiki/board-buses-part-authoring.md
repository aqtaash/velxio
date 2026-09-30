# Board buses: writing a part that talks SPI, I2C or UART

> **Why this exists**: a part used to reach the bus through whatever hook the
> engine it was handed exposed (`simulator.spi.onByte`, `setSPIHandler`,
> `addI2CDevice`, a global UART bus), and behaved differently on each engine
> and each mount order. Since `board-buses-2026-09` there is one way: the
> part names its own pins, registers a device with the bus fabric, and the
> fabric decides which board, which controller and which bytes are its, from
> the wires. This guide is the recipe, with real parts from the tree as the
> worked examples. The model itself is in [board-buses.md](./board-buses.md).

---

## Table of contents

- [The shape of a part](#the-shape-of-a-part)
- [An SPI sink: the ILI9341](#an-spi-sink-the-ili9341)
- [An SPI responder: the microSD](#an-spi-responder-the-microsd)
  - [The portable model](#the-portable-model)
  - [Live inputs](#live-inputs)
- [An I2C target: the MPU-6050](#an-i2c-target)
- [A UART endpoint that only transmits: the GPS](#a-uart-endpoint-that-only-transmits-the-gps)
- [A UART endpoint that answers: the Grove AT modems](#a-uart-endpoint-that-answers-the-grove-at-modems)
- [A board built-in](#a-board-built-in)
- [A custom chip](#a-custom-chip)
- [A module with pull-up resistors](#a-module-with-pull-up-resistors)
- [The tests a part must ship](#the-tests-a-part-must-ship)
- [Checklist](#checklist)

---

## The shape of a part

A part is a `PartSimulationLogic` registered by its element type
(`frontend/src/simulation/parts/PartSimulationRegistry.ts`):

```ts
PartSimulationRegistry.register('my-part', {
  attachEvents: (element, simulator, getPin, componentId) => {
    // register on the bus here, using the part's OWN pin names
    return () => {
      // cleanup: take the device off the bus, by identity
    };
  },
});
```

The four rules that apply to every part on a bus:

1. **Name your own pins.** The descriptor's `pins` are the names printed on
   the module (`'SCK'`, `'DI'`, `'SDA'`, `'TX'`). Never a board pin number,
   never `getPin()`: the fabric walks the wires from those names, and that is
   what puts the part on the right board and the right controller, whatever
   the engine and whatever mounted first (D-002).
2. **Identity is `componentId`.** A part without one has no pins the netlist
   can name and is on no wire. Return an empty cleanup and do nothing.
3. **Dispose by handle.** Keep the `BusHandle` the attach returned and call
   `handle.dispose()` in the cleanup. Never remove by address or by pin.
4. **Guest time, never wall time.** Anything the part paces (a byte rate, a
   sensor cycle, a chip timer) runs on the board's clock:
   `guestMillis(simulator)` from `parts/partUtils.ts` for a part, the fabric's
   `GuestClock` for a decoder, `vx_sim_now_nanos` for a chip. `Date.now()` is
   allowed for one thing only: a date the part reports (the GPS's UTC field),
   advanced in guest seconds afterwards.

Everything the fabric exports comes from one module:

```ts
import { attachSpiDevice, attachI2cTarget, attachUartEndpoint } from '../buses';
// in the pro overlay:
import { attachSpiDevice } from '@velxio/simulation/buses';
```

---

## An SPI sink: the ILI9341

A sink receives bytes and never answers. The panel in
`parts/ComplexParts.ts` (abridged; the decoder is the part's own business):

```ts
const owner = componentId || (el?.id as string) || 'ili9341';
const feed = (value: number) => {
  if (!dcState) processCommand(value);
  else processData(value);
};
const handle = attachSpiDevice(
  { owner, pins: { sck: 'SCK', mosi: 'MOSI', miso: 'MISO', cs: 'CS' } },
  {
    // The panel's SDO leg is real and stays declared, so a wire on it is
    // checked like any other. This model implements no read command, so
    // that leg never leaves high impedance, and saying so is what keeps a
    // TFT sharing its bus with a card from being reported as a responder
    // the QEMU worker cannot host.
    writeOnly: true,
    transfer: (value: number) => {
      feed(value);
      return null;
    },
    transferBlock: (bytes: Uint8Array) => {
      for (let i = 0; i < bytes.length; i++) feed(bytes[i]);
    },
    boardReset: () => {
      // The MCU rebooted: the next byte is a fresh command. The window,
      // the rotation and the picture are the panel's own and stay.
      currentCmd = -1;
      dataBytes = [];
      inRamWrite = false;
      pixelByteCount = 0;
    },
  },
);
return () => {
  handle.dispose();
  // ...the part's own listeners
};
```

What to copy:

- `transfer` returns `null`: the panel leaves MISO in high impedance and the
  fabric resolves the line. Never answer `0xFF` "before passing the byte on";
  there is no chain to pass it to.
- `writeOnly: true` because the **model** never answers, whatever the
  silkscreen says. Drop it the day the model grows a read command, and then
  give it a portable model for the remote lanes.
- `transferBlock` is the fast path for DMA and W-buffer transactions: it
  must behave exactly as byte-by-byte `transfer` with a `null` answer.
- The fabric hands the panel a frame only while its own CS is active. The
  panel does not look at CS; it does not need to. D/C, RST and BUSY are
  ordinary pins the part reads through the PinManager, not bus business.
- `boardReset` clears protocol state, not the picture: a board whose reset
  button is pressed keeps what the panel shows.
- No `deselect()` here on purpose: on this chip the RAM write survives CSX
  (Adafruit drivers send `setAddrWindow` and the pixels in separate
  transactions). Model the chip, not a generic state machine.

The SSD1306 in `parts/ProtocolParts.ts` is the same shape with two extras
worth knowing: `modes: [0, 3]` (the datasheet's) and
`csWhenFloating: 'selected'`, because the bench wires that module with its CS
pad open and it is the only chip on the bus.

---

## An SPI responder: the microSD

A responder drives MISO. The card in `parts/ProtocolParts.ts` (`microsd-card`)
keeps its model in `parts/sdSpiCard.ts` and adapts it to the bus:

```ts
// parts/sdSpiCard.ts
export function sdSpiFabricDevice(card: SdSpiCard): SpiDevice {
  return {
    select: () => card.setCs(true),
    deselect: () => card.setCs(false),
    transfer: (mosi: number) => card.transfer(mosi & 0xff),
    peekMiso: () => card.peekResponse(),
    boardReset: () => card.setCs(false),
  };
}
```

```ts
// parts/ProtocolParts.ts, the part's attachEvents
loadSdBusChip();   // start the fetch of the portable model now (see below)

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
    remoteBlobWrite: sdCardRemoteBlobWrite(card),
  },
  sdSpiFabricDevice(card),
);

return () => {
  handle.dispose();
  unpublish();
  card.setCs(false);
};
```

What to copy:

- `select` and `deselect` are the CS edges. A real card ends its command
  frame when the host lets go of CS; that is where the model resets its
  frame state, never per byte.
- `peekMiso` returns what the card **will** shift out on the next frame
  without consuming it. A bit-banged master (`shiftIn`) reads MISO bit by bit
  before the byte is complete, and the software decoder needs the answer
  ahead of the sampling edge. Every responder implements it; a sink leaves it
  out.
- `boardReset` drops the transfer in flight and nothing else: a Stop/Run is a
  reset of the MCU, and the card keeps its image (decisions, open question 3).
- `modes` and `csWhenFloating` are datasheet facts about the chip, stated
  once. The fabric reports a controller in another mode (`spi-mode`) instead
  of quietly working.

### The portable model

On a board whose processor runs in a backend worker (ESP32 or STM32 on QEMU,
the Raspberry Pi guest) the tab's copy answers nobody: the worker reads MISO
for a byte before the tab has seen the byte. A responder therefore carries the
same chip as a portable WASM model, and the fabric ships it in the bus map
(D-004). Without one, a selected responder on such a board is reported
(`bus-remote-responder-missing`) rather than left half working.

The steps, as the microSD does them:

1. **Write the model in C against `backend/sdk/velxio-chip.h`**, the same
   header custom chips use: `buses/models/microsd.c` for the OSS card;
   the pro responders keep theirs under their part folder
   (`pro/frontend/src/pro/components/xptouch/chips/`, `maxim/chips/`, ...).
   A chip that must answer inside the byte it is receiving (the MCP3008)
   sets `on_exchange` in `vx_spi_config` (D-013).
2. **Build it with the folder's `build.sh`** (wasi-sdk 22, the same clang
   flags as the production chip compiler) into `public/bus-chips/<name>.wasm`
   (OSS `frontend/public/bus-chips/`, pro `pro/frontend/public/bus-chips/`).
   The script rewrites `manifest.json` with the source's sha256, and the
   suites fail while the `.wasm` is older than the `.c`: editing the C
   without rebuilding is caught instead of silently testing the old binary.
   In velxio-prod, `Dockerfile.prod` copies the pro folder into the image.
3. **Fetch it on attach** (`loadBusChip(name)`, idempotent) and hand it over
   **synchronously** from `remoteModel()` through `busChipB64(name)`: `null`
   while the bytes are not there yet. A model that lands after the first map
   was published makes the registry publish again on its own.
4. **Name the chip's pads** when the part registers under other names
   (`chipPads`): a card inside a shield registers the shield's pads
   (`D8`, `D10`, `D9`, `D2`) because that is what the circuit resolves, while
   the model watches its select under its own name (`CS`).
5. **Storage goes as blobs.** `sdCardRemoteModel` ships the card's current
   image under `blobs.card` and names it in `blobIds.card`; the id changes
   when the part loads another image, never on a guest write, which is how a
   host tells "the tab is behind" from "the user swapped the card"
   (`hosted_model_identity`). What the hosted model writes comes back as
   spans, and `remoteBlobWrite` puts them into the tab's copy, which is what
   the card panel lists and what the next map ships back.

The model is the artifact every host runs: the tab (`ChipRuntime.ts`), the
QEMU workers and the Pi host (`wasm_chip_runtime.py`). The cross-host parity
table (`__tests__/board-buses/fixtures/chips-abi-parity/`) is what holds them
to one answer, so a responder written once behaves the same wherever it ends
up.

### Live inputs

A map goes once per membership change and carries the whole artifact; a
finger on a touch glass moves at pointer rate. The XPT2046
(`pro/frontend/src/pro/components/xptouch/attachXptTouch.ts`) shows the pair
that keeps them apart:

```ts
handle = attachSpiDevice(
  {
    owner: `${componentId}:touch`,
    componentId,
    pins: { sck: 'T_CLK', mosi: 'T_DIN', miso: 'T_DO', cs: 'T_CS' },
    modes: [0],
    bitOrder: 'msb',
    remoteModel: () => {
      const wasmB64 = busChipB64(MODEL);
      return wasmB64 ? { wasmB64 } : null;
    },
    remoteAttrs: liveAttrs,     // { x, y, touched } as they are NOW, under the model's attribute names
  },
  {
    select: () => device.select(),
    deselect: () => device.deselect(),
    transfer: (mosi: number) => device.transfer(mosi),
    peekMiso: () => device.peekMiso(),
    boardReset: () => device.deselect(),
  },
);
```

and, wherever the input changes, `handle.attrsChanged()`: on pointer down,
move and up here, **before** the part drives T_IRQ, so the coordinates reach
the worker on the same socket ahead of the interrupt. The registry sends the
values only when the device is on a remote board, has a model and the values
changed, so the call is cheap enough for every pointer move and every circuit
solve (the MCP3008 calls it on each solve of the electrical store).

---

## An I2C target

Most I2C parts are register files: a pointer, a byte array, a `stop`. That
shape is `I2CDevice` (`simulation/I2CBusManager.ts`), and `attachI2cPart`
(`parts/i2cPart.ts`) puts one on the fabric and, on a QEMU board, files the
worker record the guest is answered from. The MPU-6050 in
`parts/ProtocolParts.ts`:

```ts
PartSimulationRegistry.register('mpu6050', {
  attachEvents: (element, simulator, _getPin, componentId) => {
    const el = element as any;
    const addr = el.ad0 === true || el.ad0 === 'true' ? 0x69 : 0x68;
    const device = new VirtualMPU6050(addr);
    device.setInputs(getSensorControl('mpu6050')?.defaultValues ?? {});
    const part = attachI2cPart({
      simulator,
      componentId,
      device,
      worker: { type: 'mpu6050', props: { ...device.getInputs() } },
    });
    device.onAsleepRead = () => part.report('i2c-target-asleep', '...');
    // ...live values into device.setInputs(values), and part.updateWorker(values) for the worker's copy
    return () => {
      // ...
      part.dispose();
    };
  },
});
```

What `attachI2cPart` does with it:

- registers `attachI2cTarget({ owner: componentId, pins: { scl: 'SCL', sda: 'SDA' }, addresses: [addr], remoteModel: 'mpu6050' }, i2cTargetOf(device))`.
  Pass `pins` when the module names them otherwise (the 8-pin SSD1306 uses
  `CLK` and `DATA`);
- `i2cTargetOf` maps the fabric's `start` / `write` / `read` / `stop` onto
  `writeByte` / `readByte` / `stop`, and clears the "waiting for a pointer"
  flag on a repeated START for writing (M5Unified reads the BMI270's id that
  way) as well as on STOP;
- a model that defines `start(read)` hears every START and repeated START
  before its first byte, and one that defines `boardReset()` hears the MCU
  reset. The MPU-6050 latches its sample block at the START of a read, so a
  burst is one sample. Both are optional, and a host that cannot say where a
  read begins does not call `start`: a model that latches must also do it on
  the first `readByte` after a `writeByte` or a `stop`;
- on a board whose guest runs in a worker, sends
  `registerSensor(type, chipVirtualPin(componentId), { ...props, addr, owner })`
  so the worker's own copy answers the guest, starting from the values in
  `props`, and `updateWorker(values)` keeps that copy fed. The worker only
  has models for the types in `WORKER_I2C_MODELS`
  (`buses/workerI2cModels.ts`); a part of another type is reported on a
  remote bus (`bus-remote-responder-missing`), not silently absent;
- `report(code, message)` puts a note about the part's own state in the
  monitor of the board whose bus it is on. The part decides when it is due;
- `dispose()` takes the target off by identity and the worker record with it.

A model stores what the guest wrote and what the panel set as two things. The
panel's values are the world around the chip: they survive the chip's own
soft reset, and the output registers are worked out from them and from the
configuration the guest selected when they are read. A bit that triggers
something (a reset, a one-shot conversion) is never stored, so a driver that
polls it reads 0 at once. `test/fixtures/i2c-vectors` holds the bus vectors a
model and its backend twin both replay.

A write sink (a display, an expander) uses the same call with
`worker: { type: 'ssd1306', echo: (data) => ... }`: the worker ACKs and
echoes the writes, and the tab's copy draws from them.

When the shape does not fit, call `attachI2cTarget` directly. The Pimoroni
encoder wheel (`pro/frontend/src/pro/components/pimoroni/register.ts`) has two
chips behind one module and registers two owners on one pair of pins:

```ts
attachI2cTarget({ owner: `${componentId}:ioe`, componentId, pins, addresses: [ioe.address] }, i2cTargetOf(ioe)),
attachI2cTarget({ owner: `${componentId}:led`, componentId, pins, addresses: [led.address] }, i2cTargetOf(led)),
```

and disposes both handles in its cleanup. A chip that answers several
addresses puts them all in one `addresses` array: they leave together. A
model that may refuse a byte while present (`start` or `write` returning
`false`) sets `mayNak: true`, so the Raspberry Pi relay asks the tab instead
of ACKing on its behalf.

What not to do: `addI2CDevice` and `removeI2CDevice` no longer exist on any
simulator. There is no "bus 0"; the target lands on the bus its SDA is wired
to, `Wire1` included, and on the XIAO RP2040 that is I2C1.

---

## A UART endpoint that only transmits: the GPS

The NEO-6M (`parts/GpsParts.ts`, aliased by the Grove SIM28 and Air530Z)
prints an NMEA cycle once a second at 9600 baud and never listens:

```ts
if (!componentId) return () => {};

// Only a TX leg: a receiver never listens, so the fabric never places an RX
// leg for it, whatever its RX pad is wired to. receive() exists because the
// endpoint contract has it; with no RX leg on any wire nothing reaches it.
const handle = attachUartEndpoint(
  { owner: componentId, pins: { tx: 'TX' }, baud: GPS_BAUD },
  { receive: () => {} },
);

// ...one character every GPS_BYTE_MS of GUEST time, at most BURST_MAX per
// poll, a new cycle every GPS_CYCLE_MS, all from guestMillis(simulator):
while (pending.length > 0 && t >= byteDueAt && sent < BURST_MAX) {
  handle.transmit(pending.shift()!);
  byteDueAt += GPS_BYTE_MS;
  sent++;
}
```

What to copy:

- Declare only the legs the device has. The descriptor is the datasheet: a
  receiver has a TX and no RX.
- `baud` is the rate the module ships at. It is what the fabric compares
  against the controller's `Serial.begin()` (garbage plus
  `uart-baud-mismatch` when they differ beyond 3 %, as on hardware) and the
  bit time a byte to a plain GPIO is clocked at.
- `handle.transmit(byte)` is the wire. The fabric decides where it goes: a
  hardware UART's RX (the Uno's D0, the Mega's RX1, the ESP32's U2RXD), a
  plain GPIO the sketch samples through SoftwareSerial (the fabric's emitter
  puts the frame on the pin as edges at exact guest instants), the board's
  own TX (`uart-tx-contention`, and the board hears nothing), or nothing.
- Pace on the guest clock, and cap a burst: the ESP32's 128-byte FIFO never
  receives a whole cycle in one instant on the bench. A stopped board is an
  unpowered module: nothing leaves, and the first fix after Run comes a
  second later.

---

## A UART endpoint that answers: the Grove AT modems

A command-and-response module has both legs. The AT family
(`pro/frontend/src/pro/components/grove/at/register.ts`) hosts one modem
model per part id, all through one function:

```ts
function atPart(make: ModemFactory, baud: number): PartSimulationLogic {
  return {
    attachEvents: (element, _simulator, _getPin, componentId) => {
      const el = element as ValueSink;

      let modem: AtModem | null = null;
      const handle = attachUartEndpoint(
        { owner: componentId, pins: { rx: 'RX', tx: 'TX' }, baud },
        { receive: (b) => modem?.feed(b) },
      );

      modem = make({
        send: (b) => handle.transmit(b),
        onState: (s) => el.setValues?.(s),
      });

      registerSensorUpdate(componentId, (values) => {
        modem?.update(values as Record<string, number | boolean>);
      });

      return () => {
        unregisterSensorUpdate(componentId);
        modem?.dispose();
        modem = null;
        handle.dispose();
      };
    },
  };
}

PartSimulationRegistry.register('grove-at-hm11', atPart((opts) => new Hm11Modem({ ... }), 9600));
```

`RX` and `TX` are the module's own pad names: its TX lands on a board RX pin
and vice versa, and the fabric decides from the nets which controller, if
any, is on each. The same code serves a modem on `Serial1` of a Mega, on
SoftwareSerial pins of an Uno, on the header UART of a Pi (through
`PiUartPort`, both Pi engines) and on `Serial2` of an ESP32 in QEMU, and an
unwired one hears nothing. Nothing here picks a UART, classifies a pin or
falls back to the console.

---

## A board built-in

A peripheral soldered to a board names the board pins instead of pin names,
through the pro overlay's helpers. The Badger 2350's e-paper
(`pro/frontend/src/pro/boards/rp2350/badgerBuiltins.ts`):

```ts
cleanups.push(
  attachBuiltinSpiDevice(
    boardIdOf(el),
    'epd',
    { sck: EPD_SCK, mosi: EPD_MOSI, cs: EPD_CS },
    {
      transfer: (mosi: number) => {
        decoder.feed(mosi, sim.pinOutputValue(EPD_DC));
        return null;
      },
      transferBlock: (bytes: Uint8Array) => {
        const dc = sim.pinOutputValue(EPD_DC);
        for (let i = 0; i < bytes.length; i++) decoder.feed(bytes[i], dc);
      },
      // A Stop/Run resets the MCU, not the panel: an e-paper holds its image
      // with the power off. Only a falling edge on RST clears the controller.
    },
  ),
);
```

and its RTC (`pro/frontend/src/pro/boards/index.ts`):

```ts
const off = attachBuiltinI2cTarget(boardId, 'rtc', { sda: 4, scl: 5 }, [0x51], new BuiltinI2cDevice(rtc));
```

`BuiltinI2cDevice` wraps a register-file model as a target, with an optional
`powered` predicate: a chip whose load switch is off does not ACK its
address, which is how a sketch that forgets to raise the IMU's enable pin
finds nothing (the XIAO boards' LSM6DS3 in `boards/seeed/boardImu.ts`). The
owner is `builtin:<boardId>:<name>`; the helper returns a no-op cleanup when
the board element has no id yet, so a caller can always push it on its
cleanup list. A built-in with an answer (the M5Stack panel's id) carries a
portable model like any responder (`boards/m5stack/peripherals.ts`).

---

## A custom chip

A WASM chip joins the bus when it calls `vx_spi_attach`, `vx_i2c_attach` or
`vx_uart_attach` with the pins of its config; the runtime registers it with
the same three functions this guide uses, under `<id>:spi<n>`, `<id>` and
`<id>:uart<n>`. Its select, mode, addresses and rate come from the config
struct; its clock is the board's. An analog pad reads the voltage the
circuit solve holds on its net through `vx_pin_read_analog`, the same number
in every host, and `vx_pin_wired` says whether a wire reaches the pad at all
(the Grove ADS1115 keeps its slider only for a pad nothing is wired to).
Write it as
[custom-chips-api-reference.md](./custom-chips-api-reference.md) says and it
gets everything above for free, in the tab, in a QEMU worker and on the Pi.

---

## A module with pull-up resistors

If the module's schematic has a resistor from a data line to VCC (or GND),
declare it; the simulator does not guess. The Grove 4-Digit Display has
2 x 10k on CLK and DIO, and without them avishorp's `TM1637Display`, which
sends a 1 by releasing the line with `pinMode(INPUT)`, leaves the display
dark. Where to say it:

| The part is | Declaration |
|---|---|
| a custom chip (`chip.json`) | `"pulls": { "CLK": "up", "DIO": "up" }`, by pin name |
| a host that creates a `ChipInstance` | `pulls: { CLK: 'up', DIO: 'up' }` in the options, by chip pin (a key of `wires`) |
| a Grove chip module (velxio-prod) | `sim: { kind: 'chip', ..., pulls: { CLK: 'up', DIO: 'up' } }`, by pad |
| any other part with the PinManager at hand | `setBoardPinPull(pm, pin, '<componentId>::<pad>~pull', 'up', sink)`, and `null` on cleanup; `sink` puts the level into the guest (the simulator's `setPinState`) |

The rule the net applies is in board-buses.md, "Pull resistors on a line":
any strong drive beats the pull with no warning, the pull beats the MCU's
internal pull the other way, and the pulled level reaches both the sketch
and the parts on the pin. Do not declare one for SDA and SCL: the I2C fabric
already models the bus pull-up, on hardware and software I2C alike. Do not
declare one to fix a line your own model drives: a 1-Wire model that owns
its line answers a read slot with its own rise.

Tests: drive the library's exact pin sequence with `pinMode(INPUT)`
releases and no pull on the pad (never an injected HIGH standing in for the
resistor), on a pad-reporting host and on an ESP32-shaped one (direction and
pull channels, a level only when the latch moves), and keep a negative
control with the declaration removed. The Grove display's
`grove/__tests__/tm1637.test.ts` is the template.

---

## The tests a part must ship

Three kinds, and each one has to **fail when the part is broken**. A test
that passes before and after a change proves nothing and is rewritten (that
is how the old e-paper and microSD cleanup tests were caught passing under
the behaviour they were meant to forbid).

### 1. The real-firmware row

Drive the real engine with a sketch built by the production toolchain and a
real library, through the store's own lifecycle, and read the result the way
a user would (the serial monitor, a pin level, a pixel). The GPS on an Uno
(`frontend/src/__tests__/board-buses/board-buses-f6-gps-avr.test.ts`) is the
template:

```ts
class Board {
  constructor(kind: 'arduino-uno' | 'arduino-mega', hex: string) {
    const st = useSimulatorStore.getState();
    st.addBoard(kind, 0, 0, this.id);
    this.sim.onSerialData = (ch: string) => { this.out += ch; };
    st.compileBoardProgram(this.id, hex);
    st.startBoard(this.id);
  }
  /** Wire a component pin to one of this board's pins, as the canvas does. */
  wire(componentId: string, pinName: string, boardPin: number): void {
    useSimulatorStore.getState().addWire({
      id: `${this.id}-w${++this.wireSeq}`,
      start: { componentId, pinName, x: 0, y: 0 },
      end: { componentId: this.id, pinName: String(boardPin), x: 0, y: 0 },
      waypoints: [],
      color: '#0a0',
    } as never);
  }
  run(ms: number, done = () => false): void { /* step the CPU for `ms` of GUEST time */ }
}

it('avr-gps-softserial: on the Uno D4 through SoftwareSerial(4, 3), the fix prints and every checksum passes', () => {
  // TinyGPS++ on a real SoftwareSerial, the module's TX wired to D4
});
it('avr-gps-serial1: on the Mega RX1 (19), Serial1 carries the fix through USART1', () => { /* ... */ });
```

The fixture lives next to the test (`fixtures/avr-gps-softserial/`: the
`.ino`, the `.hex`, `compile.log`), built with
`project/board-buses-2026-09/harness/compile-fixture.mjs --fqbn arduino:avr:uno --libs TinyGPSPlus`
in the velxio-prod checkout; the sketch's header comment repeats the command
so the next person can rebuild it. Real library, real wiring (the one every
tutorial uses), real output parsed from the monitor.

For a part with no wire-level path in node (a QEMU board), the row is a
backend test on the worker rig (`test/backend/unit/test_board_buses_*.py`, a
real `esp32_worker.py` with a fake libqemu) and, before a merge, a cell of the
in-app matrix (`project/board-buses-2026-09/harness/bus-matrix.mjs`) run
against a local build and against velxio.dev.

### 2. The negative control

A row that proves the assertion has teeth. Two forms:

- **In the tree**: a sibling row that breaks the circuit on purpose and
  asserts the failure the bench would show. `port-conformance-rp2040.test.ts`
  keeps one next to its microSD row: `microsd-r1-without-ncr-fill control:
  with the card DO leg unwired nothing drives MISO, and SD.begin() reports
  failure`. If the positive row ever passes without the card, this one says
  so.
- **Recorded**: mutate the product code, one change at a time, re-run the
  suite, note how many rows fell, revert and check the md5. The project's
  `STATUS.md` records every control per phase, including the ones that
  caught nothing; a mutation nothing catches is either an equivalent mutant
  (say why) or a missing test (write it). Snapshot the file before
  mutating, run the script in the foreground, and never end a session with
  a mutation applied.

A part's F0-style row (`board-buses-repro-*.test.ts`) is written as `it.fails`
while the defect is open and flipped to `it` by the fix, which is the same
control in the other direction.

### 3. The unwired case

A part with a leg wired to nothing must be on no wire: nothing heard,
nothing transmitted anywhere, and no diagnostic for a part that is simply not
wired yet. The GPS rows:

```ts
it('TX on the Uno D1 (its TX): reported as uart-tx-contention, and Serial never sees a character', () => { /* ... */ });
it('TX wired to nothing: on no wire, no diagnostic, and Serial never sees a character (no UART0 fallback)', () => { /* ... */ });
```

The second row is the one that guards against every "default bus" that ever
crept in (bus 0, `boards[0]`, UART0, `CHIP_UART`). Collect diagnostics with
`busRegistry.onDiagnostic` in `beforeEach` and `busRegistry.resetDiagnostics()`
in `afterEach`, and assert the list, not only the bytes: the wrong-pin row
must show its code, the unwired row must show none.

### Running them

```bash
cd velxio/frontend && npx vitest run src/__tests__/board-buses/board-buses-f6-gps-avr.test.ts
```

Run targeted suites while you work: each vitest fork can take an 8 GB heap.
The full suite is the closing agent's, with `--maxWorkers=3`, reading the
`Errors` line as well as the counts. A `-t` filter that matches nothing exits
0, so count the rows that ran.

---

## Checklist

- [ ] The descriptor names the part's own pins, and only the legs the chip has.
- [ ] `owner` is `componentId` (or `<componentId>:<chip>` per chip in a module, or `builtin:<boardId>:<name>`), and the cleanup calls `handle.dispose()`.
- [ ] Datasheet facts stated once: `modes`, `bitOrder`, `csActive`, `csWhenFloating`, `addresses`, `baud`, `frame`.
- [ ] A sink returns `null` and says `writeOnly: true`; a responder implements `peekMiso`, resets its frame in `deselect`, and ships a portable model (`remoteModel`, `loadBusChip`, `build.sh`, `manifest.json`).
- [ ] Live inputs go through `remoteAttrs` and `handle.attrsChanged()`; storage through blobs and `remoteBlobWrite`.
- [ ] `boardReset` clears protocol state and keeps data.
- [ ] Every pull resistor on the module's schematic that sits on a line the sketch can release is declared (`pulls`), and none is invented.
- [ ] Nothing is paced on `Date.now()` or `performance.now()`.
- [ ] Three tests: the real-firmware row with its fixture, the negative control, the unwired case.
- [ ] No `simulator.spi`, `setSPIHandler`, `addI2CDevice`, `classifyPin`, `feedUart` or `serialWrite`: none of them exist on a bus part any more.
