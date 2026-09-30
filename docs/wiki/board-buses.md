# Board buses: SPI, I2C and UART as wires on a board

> **Why this exists**: until 2026-09 a bus did not exist as an object in
> Velxio. Each engine had a callback (`spi.onByte`, `setSPIHandler`,
> `onTransmit`, `addI2CDevice`, a global UART bus), each part hooked itself to
> whichever callback it was handed, and what a sketch read back depended on
> the order React mounted the parts. Issue #355 (a touch controller gone deaf
> because a Grove module shared the board) was one of 94 findings of that
> family. Project `board-buses-2026-09` replaced all of it with one model,
> the one a wire has: a device is on a bus because its pins are on that bus's
> nets, chip select, address and TX/RX decide who talks, and every engine
> exposes the same controller port. This page is the model, the engine
> contract, the device contract and the remote-host design. The part author
> guide is [board-buses-part-authoring.md](./board-buses-part-authoring.md).

Every rule below names the decision it comes from (`D-nnn`) when it comes
from one. The decisions, their alternatives and the measurements are in the
velxio-prod checkout under `project/board-buses-2026-09/` (`DESIGN.md`,
`decisions.md`, `STATUS.md`, `evidence/`).

---

## Table of contents

- [The model](#the-model)
  - [A bus is a net on a board](#a-bus-is-a-net-on-a-board)
  - [Membership by wiring](#membership-by-wiring)
  - [Removal by identity](#removal-by-identity)
  - [Arbitration, as the wire does it](#arbitration-as-the-wire-does-it)
  - [Software buses](#software-buses)
  - [Pull resistors on a line](#pull-resistors-on-a-line)
  - [Lifecycle](#lifecycle)
  - [Diagnostics](#diagnostics)
- [The engine contract](#the-engine-contract)
  - [getBusBinding](#getbusbinding)
  - [The controller ports](#the-controller-ports)
  - [Routing](#routing)
  - [The guest clock](#the-guest-clock)
  - [The ports each engine exposes](#the-ports-each-engine-exposes)
  - [The conformance kits](#the-conformance-kits)
- [The device contract](#the-device-contract)
  - [attachSpiDevice](#attachspidevice)
  - [attachI2cTarget](#attachi2ctarget)
  - [attachUartEndpoint](#attachuartendpoint)
  - [Board built-ins](#board-built-ins)
  - [Custom chips](#custom-chips)
  - [The bus maps sent to a remote host](#the-bus-maps-sent-to-a-remote-host)
- [Remote hosts: the responder runs beside the master](#remote-hosts-the-responder-runs-beside-the-master)
  - [The blob ABI](#the-blob-abi)
  - [Live attributes](#live-attributes)
  - [Sinks stay in the tab](#sinks-stay-in-the-tab)
  - [What the microSD costs on a QEMU board](#what-the-microsd-costs-on-a-qemu-board)
  - [I2C and UART on a remote lane](#i2c-and-uart-on-a-remote-lane)
- [Decisions by number](#decisions-by-number)
- [Files and tests](#files-and-tests)

---

## The model

Everything lives in `frontend/src/simulation/buses/`. Three roles, one
contract each (`types.ts`):

```
  engine (SoC) --> ControllerPort, one per serial peripheral (SPI0, I2C1, UART2...)
                        |
                 BoardBusFabric, one per board on the canvas
                   membership from the circuit's nets, arbitration, diagnostics
                        |
          SpiDevice   I2cTarget   UartEndpoint
        (canvas parts, the board's own peripherals, custom chips, the software decoders)
```

Nothing in the fabric knows a simulator object, and nothing in a device
touches an engine hook. The registry (`busRegistry`, one per page) is the only
way to reach a fabric, and the store keys it by board id, never by simulator
instance (D-001, D-003).

### A bus is a net on a board

- An **SPI bus** is the net of a board pin that carries SCK. A hardware
  controller feeds the bus of the pin its SCK is routed to right now; the
  software decoder feeds the same bus from the edges on that pin. Hardware
  SPI, bit-banged SPI and a controller the sketch moved to other pins share
  one arbitration (DESIGN 5.0).
- An **I2C bus** is the net of a board pin that carries SDA; its clock is the
  SCL of the controller routed to that SDA, or, on a software bus, the SCL its
  members share. `Wire` and `Wire1` are two buses exactly when their pins are
  two nets.
- **UART has no bus, only wires**: one net per board pin, with whoever
  transmits on it and whoever listens. A controller's TX feeds the net of its
  TX pin and its RX reads the net of its RX pin; an endpoint's RX and TX legs
  each land on the net of their own pin. One TX to several RX is legal, as on
  the bench.

A board's fabric lives as long as the board is on the canvas, not as long as
an engine instance lives. The store binds and unbinds the engine
(`busRegistry.bindBoard(id, sim)` / `unbindBoard(id)`), and the board's
devices never notice a rebind.

### Membership by wiring

A device names its **own** pins, by the pin names printed on it (`'SCK'`,
`'SDA'`, `'TX'`), and the registry walks the circuit's nets (wires,
breadboard strips, sockets, passives) to find the board pin each one lands on
(D-002). The walk is the store's own trace (`createStoreNetResolver`, on the
same net union `PinTrace` and the chip nets use), through a `NetResolver`:

| Where a pin lands (`ResolvedPin`) | Meaning |
|---|---|
| `board` (`boardId`, `pin`) | a GPIO of that board |
| `chip` | a synthetic net a custom chip drives, not a board |
| `rail` (`gnd` or `vcc`) | tied to ground or a supply |
| `floating` | wired to nothing that drives it, or not wired |

An SPI device belongs to the bus of the board pin its SCK reaches. An I2C
target belongs to the bus of the board pin its SDA reaches, on **every** board
both its SDA and SCL reach (`resolveAll`: a net wired from one board's I2C
header to another's carries the chip to both masters). A UART leg belongs to
the net of the board pin it reaches; a leg on a rail, a chip net or nothing is
on no wire at all, and there is no default UART to fall back to.

Consequences:

- A device is on the board it is wired to, never on `boards[0]` or the active
  board. Multi-board projects work by construction.
- Editing a wire re-places the device (`busRegistry.netlistChanged()` from
  the store, coalesced in a microtask). Nothing re-attaches, so attach order
  cannot matter. The unit suite proves it with every permutation of four
  devices' registration order.
- A part dropped on the canvas and not yet wired is on no bus, silently: SPI
  stays quiet until SCK reaches a board, I2C until both lines reach one.
  Half wired (one I2C line on a board, the other not) is reported, because
  silence with no reason reads as a dead chip.

### Removal by identity

Every attach returns a handle whose `dispose()` takes the device off, by
identity. Registering an owner that already exists replaces it, so a remount
that skipped its cleanup can never leave a stale twin listening. Nothing is
ever removed by address: `removeDevice(0x68)` used to take off whichever
device held that address, the part's own replacement included (finding
`i2c-remove-by-address-evicts-other-part`), and that call no longer exists.

Identity is the descriptor's `owner`: the component id for a part, or
`builtin:<boardId>:<name>` for a peripheral soldered to a board. A module
with two chips registers two owners (`<id>:touch` and `<id>:rtc` on the Seeed
Round Display). A custom chip is `<id>:spi<n>` per SPI handle and `<id>`
(then `<id>:uart<n>`) per UART handle.

### Arbitration, as the wire does it

**SPI** (`spiBus.ts`). The bus keeps the set of selected members updated on
chip-select edges, never per byte, and every frame is resolved from that set:

| Selected members that drive MISO | MISO the master reads | Diagnostic |
|---|---|---|
| 0 | the idle line, `0xFF` | none |
| 1 | that device's answer | none |
| 2 or more | the bit-wise AND of their answers | `spi-contention` |

- Every selected member receives the frame (`transfer(mosi, bits)`), a
  write-only panel included; only the ones that answer with a byte count as
  drivers. A device returns `null` to leave MISO in high impedance.
- A device counts as a driver only when its MISO leg resolves onto the bus
  and the model is not `writeOnly`. A device with no MISO wired is heard,
  never answered.
- Two members selected at once is `spi-multiple-selected` whether or not both
  drive: each receives the other's bytes, as on a real board.
- `select()` and `deselect()` are electrical events on the CS edge, not
  inferences per byte: the chip resets its frame state on deselect like the
  real one (SD, XPT2046, MCP3008).
- Bit order: if the controller shifts LSB first and the device expects MSB,
  the device sees every byte bit-reversed, as on hardware, and
  `spi-bit-order` is reported. Mode: a controller in a mode the device does
  not accept is reported (`spi-mode`); the shifted bits are not emulated
  (D-011).
- Frames of 4 to 32 bits carry `bits` with them; the device decides.

Where the chip select comes from (`registry.ts`, `csSource`):

| `pins.cs` resolves to | Selection |
|---|---|
| a GPIO of the same board | the pin's level, `csActive` (`'low'` by default) |
| a hardware chip-select output the engine reports | the port's `setHardwareCsHandler` events |
| a chip net | that net's level |
| `gnd` or `vcc` | constant: selected when the rail matches the active level |
| omitted | no select line at all (a 74HC595 latch): always selected |
| a pin of another board | `spi-cross-board`, treated as unconnected |
| nothing | `csWhenFloating`: `'selected'` means the chip pulls it active itself; anything else is deselected plus `spi-cs-floating` |

The idle level of MISO is `0xFF` on every host (D-012). D-012 also foresaw
`0x00` when the canvas has a pull-down on that net; that half is not in the
code today.

**I2C** (`i2cBus.ts`). The address phase is an ACK vote: every clocked target
that has the address sees the START, and the controller reads an ACK if any
of them ACKs. Two targets at one address on one bus is what it is on the
bench: both ACK, a read returns the wired-AND of their bytes, and
`i2c-address-conflict` names them. A repeated START ends the previous address
phase without a STOP, and the STOP reaches every target the transaction
touched. A data byte the target NAKs ends the transfer, as in hardware (the
six in-browser ESP32 engines used to count it as an ACK; they no longer do).

**UART** (`uartBus.ts`). A byte a controller transmits reaches every listener
on the net of its TX pin, in owner order, and, through a wire between boards,
the RX of the peer board's controller. A byte an endpoint transmits reaches
the controller whose RX pin is on that net. Two transmitters on one wire is
`uart-tx-contention`, checked when membership or routing changes, never per
byte. The rate is compared when both sides know theirs: within 3 % they match
(the AVR's real 117647 against a module at 115200 is not a mismatch), and
otherwise the receiver gets the garbage the silicon would read
(`resampleUartFrame`: a byte with a parity error is dropped as HardwareSerial
drops it, one with only a framing error is delivered) plus
`uart-baud-mismatch`, once. A side with no rate takes bytes as they come.

### Software buses

If a device's pins land on board pins no controller is routed to, the device
is on a **software bus** and the fabric puts a pin-level decoder in front of
it that feeds the same device contract (D-005):

- `SoftSpiDecoder`: SPI from SCK edges, the four modes, MSB or LSB first. A
  real chip shifts its answer out while the master is still clocking the
  byte in, so the decoder asks the selected device what it will send next
  (`peekMiso`) and puts each bit on the MISO pin ahead of the sampling edge.
- `SoftI2cDecoder`: START, STOP, repeated START and ACK from SDA and SCL
  edges. The ATtiny85's USI in two-wire mode is served this way on PB0/PB2.
- `SoftUartDecoder` and `SoftUartEmitter`: a bit-banged TX decoded from the
  guest's edge timestamps (the stop bit's deadline comes from `clock.at`,
  because a byte whose last bits are ones ends with no edge at all), and a
  byte to a plain GPIO put on the pin as edges at exact guest instants
  (`clock.scheduleEdge`). A `SoftwareSerial(4, 3)` on an Uno and a
  `Serial1.begin(9600, SERIAL_8N1, 25, 26)` on an ESP32 (a hardware UART the
  matrix moved) reach the same module model.

The same model of a microSD, a MAX6675 or a custom chip therefore works with
`SPI.transfer()` and with `shiftIn`/`shiftOut`, as the chip does. The one rule
this rests on, tested for every engine: **a pad routed to a peripheral never
produces GPIO edges**, so the hardware and the software path can never both
deliver the same frame.

### Pull resistors on a line

A module carries resistors on its data lines: the Grove 4-Digit Display has
2 x 10k from CLK and DIO to VCC, a DS18B20 module 4.7k on DQ, every I2C
breakout 4.7k or 10k on SDA and SCL. An open-drain driver releases a line
with `pinMode(INPUT)` and reads HIGH only through them: avishorp's
`TM1637Display` sends every 1 bit that way, SoftwareWire does with
`pullups = false`, OneWire always does. Until 2026-09-28 nothing put that
resistor on the wire, so a released line kept the LOW the MCU last drove, in
the guest and on the level channel, and the Grove display stayed dark with
avishorp's library on every board (finding `module-pullup-not-modelled`).

The resistor is a driver of the board pin's net (`customChips/busNets.ts`,
`setBoardPinPull`), of PULL strength, on the net for as long as the part is
on the canvas. The net resolves in tiers, as the wire does:

| On the line | The level | Why |
|---|---|---|
| any strong drive: the MCU's output, a chip pulling low, a part's injection (a button, a line model, the SPICE connector) | the strong drive's; two opposite strong drives are contention (`[chipbus] contention` warning) | a 10k loses to any push-pull stage; no warning for a pull under a drive |
| no strong drive, one or more module pulls | the modules' | the smaller resistor sets the divider: 4.7k-10k beats the MCU's internal 20k-100k, so a module pull-up wins over `INPUT_PULLDOWN` |
| a module pull-up and a module pull-down, nothing driving | kept where it was, and said once (`[chipbus] ... pull-up and a pull-down from two modules`) | a divider of equal resistors sits mid-rail, which is no logic level |
| no strong drive, no module pull | the pad's own pull (`INPUT_PULLUP`), else the level it had | as before |

What follows from it:

- The pulled level reaches the guest's input register through the part
  host's digital-write door (the same sink a chip's own drive uses), and the
  level channel, so the sketch's `digitalRead` and every part watching the
  pin agree.
- A part's injection is a drive: it beats a pull, the module's included. The
  latch of an input pad (the AVR's PORT bit under `INPUT_PULLUP`) is not; it
  reaches the net as the source `'pull'` (`PinManager.LevelSource`) and the
  resolution answers it.
- A release is heard on the channel the engine reports it on: the pad
  channel on the AVR, RP2040, RP2350 and XIAO ARM engines, the direction and
  pull channels (`PinManager.onPinConfigChange`) on the ESP32 bridges, whose
  outputs the resolution treats as strong drives. `setBoardType` and
  `initSimulator` now wire `onPinDir` as `addBoard` always did.
- An engine that reports neither pads nor directions (the Pi and STM32
  bridges) cannot say when the guest lets go of a pin it drove. There a
  module pull sets the level of a pin the guest has never driven, and
  nothing else.
- An MCU drive that moves only the direction register (the AVR's
  `pinMode(OUTPUT)` over a latch already at 0) is put on the level channel by
  the net when the channel disagrees: the engine reports it on the pad
  channel only, and on a pulled line the parts would otherwise never see the
  line go LOW.
- A board reset does not cut the module's supply: `hardResetPinStates` fires
  the config listeners and the pulled level goes back on the wire.
- A chip's own `VX_INPUT_PULLUP` is still not put on a board pin: chips use
  it for their own reads of a select or an IRQ, and the worker models none.
  The PCB's resistor is declared by the part: `ChipInstanceOptions.pulls` by
  chip pin, `"pulls": { "DIO": "up" }` in a custom chip's `chip.json`, and
  `ChipSim.pulls` by pad on a Grove module in velxio-prod.
- A **software I2C bus** carries its members' pull-ups itself
  (`softI2c.ts`): on a board whose pins come with the PinManager
  (`BoardPins.pinHost`) it puts a pull-up on SDA and SCL and the target's
  low as a strong driver of SDA. An I2C part declares no pull of its own.
  Hardware I2C is served by the controller port and never reads a line.
- A 1-Wire model that owns its line (the DS18B20 of velxio-prod, a DHT line
  model) already answers a released read slot with a timed rise, so it needs
  no declared pull.

Remote hosts. On the QEMU ESP32 and STM32 boards the guest's input register
is the worker's, and QEMU's pin injection carries a level and no float state:
a line the guest or a chip hosted there released kept the last level anybody
wrote. The worker has a pad model for the pads a module pull is on
(`backend/app/services/pad_model.py`, shared by `esp32_worker.py` and
`stm32_worker.py`), with the same tiers as the table above:

- the tab sends every module pull on the board's pins as the `pulls` half of
  the bus map (`[{pin, pull, owner}]`, `busNets.boardPinPulls`, sent by
  `buses/remotePulls.ts` with `start_*` and again when a part's resistors
  come, move or go);
- strong drivers are the guest's output (its latch while the pad is an
  output), a chip hosted in the worker between its `vx_pin_write` and its
  release (`vx_pin_set_mode` to an input mode, or the chip leaving), and a
  level the tab injected (`set_pin`), which holds until the guest drives the
  pad as an output, as the store's own release does;
- with none of them the module pulls decide, and the level goes into the pad
  inside the direction write that released it (`picsimlab_dir_pin`), so the
  sketch's next `digitalRead` already sees it; an up and a down from two
  modules leave the pad and are said once;
- what reads pads in the worker hears the resolved level: a chip's
  `vx_pin_read` and its `vx_pin_watch` callbacks, and a GPIO chip select;
- a pad with no module pull is not the model's: it keeps the old behaviour,
  the last level written, and the MCU's own internal pull is not modelled
  there (the ABI parity row `board_pin:DIR:worker` still says so).

A custom chip the worker hosts puts its `chip.json` `"pulls"` on the tab's net
from `CustomChipPart` (no `ChipInstance` in the tab does it on that path), so
the level channel and the worker agree.

The Raspberry Pi needs no pad model on the host: the guest's input reads are
answered from the levels the tab injected (`qemu_manager.set_pin_state`), and
the host forgets a pin's output level when the guest turns it into an input.
Its tab-hosted chips run through this same net, with the no-direction rule
above: the Pi bridge reports the direction in `gpio_setup`, and the store
reads only its pull.

### Lifecycle

| Event | What happens |
|---|---|
| Part mounts | `attachSpiDevice` / `attachI2cTarget` / `attachUartEndpoint` from its `attachEvents`; a handle by identity |
| A wire is edited | the registry re-places the device; nobody re-attaches |
| Stop then Run, Reset, firmware reload, MicroPython reset | the engine adapter re-binds its ports to the rebuilt SoC; every device gets `boardReset()`: protocol state, not data. An SD card keeps its image and an e-paper its picture, as on a board whose reset button is pressed (decisions, open question 3) |
| Engine swap (the in-browser ESP32 falling back to QEMU, a rebuilt bridge) | the fabric does not change; the store binds the new simulator |
| Active board changes | nothing |
| Part removed | `handle.dispose()` |
| Board removed | its fabric goes with it; the registry re-links what was wired to it across boards |

### Diagnostics

Diagnostics are part of the model (D-006): a professional simulator says why a
circuit does not work instead of not working. Each one names the code, the
bus, the board and the owners involved, with a message written for the
person wiring the circuit. The full list (`BusDiagnosticCode` in `types.ts`):

| Code | When | What it tells the user |
|---|---|---|
| `spi-contention` | two selected devices drive MISO in one frame | which two, and that the master reads their wired-AND |
| `spi-multiple-selected` | two or more devices are selected at once, even if one only listens | each receives the other's bytes; check the CS lines |
| `spi-cs-floating` | a chip's CS is wired to nothing and the chip declares no pull | the chip never answers; wire CS to a GPIO, or to GND if it is alone on the bus |
| `spi-wiring` | MOSI and MISO crossed, MOSI or MISO on a pin the controller does not use, a responder's MISO wired to nothing, or two controllers routed to one SCK pin | which leg is where and what to swap or move |
| `spi-cross-board` | SCK on one board and CS on another | the CS is treated as unconnected |
| `spi-mode` | the controller runs a mode the chip does not accept | the data would be shifted on hardware (D-011) |
| `spi-bit-order` | LSB-first controller, MSB-first chip (or the reverse) | the chip receives every byte bit-reversed |
| `spi-no-controller` | declared in the code type, never emitted: a bus whose pins no controller is routed to is served by the software decoder and stays silent | |
| `i2c-address-conflict` | two targets answer at one address on one bus | every one ACKs and a read is the wired-AND; change an address or move one to another bus |
| `i2c-wiring` | SDA and SCL crossed against a controller; a target whose SCL is not the bus's clock; one line on a board and the other not; SDA on one board and SCL on another | which line goes where; wire both to the board's I2C pins or to two GPIOs for software I2C |
| `i2c-target-asleep` | a chip that powers on asleep was read for data before the sketch woke it (the MPU-6050's sample block with SLEEP set) | the chip answers zeros, as on the bench, and which register to write to wake it |
| `uart-baud-mismatch` | receiver and sender rates or frames differ beyond 3 % | what each side runs at, and that what arrives is garbage, as on hardware |
| `uart-tx-contention` | two transmitters on one wire (a module's TX on the board's TX pin, two modules on one RX, two boards' TX wired together) | who transmits, and which pin to move to |
| `uart-wiring` | RX wired to RX (nobody transmits), or two controllers routed to one TX pin | wire the module's RX to the board's TX pin |
| `uart-no-clock` | a module on a plain GPIO of a board whose emulator does not time its pads | wire the module to a hardware UART pin |
| `uart-no-baud` | a module on a plain GPIO that declares no rate, so no bit time exists | give the part a rate, or use a hardware UART pin |
| `bus-remote-responder-missing` | on a board whose processor runs in the backend, a selected SPI responder (or a clocked I2C target) has no portable model the worker could run | the board reads an idle bus or finds nothing at the address; run the board on an in-browser engine, or use a part that carries a model |

Rules:

- A diagnostic is said **once** per (code, board, owners); one with no owners
  once per message. The registry forgets them on `resetDiagnostics()`. The
  suites call it between cases; the app does not call it on Run today, so in
  a page each one is said once.
- The exception is what a chip says about its own state
  (`i2c-target-asleep`). The chip raises it through its part's handle
  (`I2cPartHandle.report`, `busRegistry.reportI2cTarget`) when it is due,
  once per power-on and per MCU reset, and the registry passes it on as it
  comes: a second Run hears it again.
- They reach the user through the simulator notes of the board's monitor
  (`appendSimulatorNote` in `useSimulatorStore.ts`, wired to
  `busRegistry.onDiagnostic`). The part inspector does not show them yet.
- A QEMU worker emits `bus_diag` for the contention only it can see; the tab
  does not surface that event yet.

---

## The engine contract

### getBusBinding

A simulator exposes its buses by implementing one method
(`BusCapableSimulator`, `isBusCapable(sim)`):

```ts
getBusBinding(): EngineBinding | null;

interface EngineBinding {
  pins: BoardPins;              // onPinChange, peekPinState, peekPad?, onPadChange?, driveInput?
  spi: SpiControllerPort[];     // every SPI controller of the SoC
  i2c?: I2cControllerPort[];    // every I2C controller
  uart?: UartControllerPort[];  // every UART controller
  clock?: GuestClock;           // the guest's clock, for the software UART
  setResetHandler?(handler: (() => void) | null): void;
}
```

`boardPinsFromPinManager(pm)` builds `pins` from a board's `PinManager`. The
store calls `busRegistry.bindBoard(id, sim)` when a simulator is created for a
board and `unbindBoard(id)` when it goes; an engine with no binding leaves its
board with no buses, and its devices wait for the next bind.

### The controller ports

A port is the engine's face for one peripheral. It is created **once per
board** and kept across every rebuild of the SoC (D-003): the adapter
re-points it at the new `AVRSPI` that `loadHex` or `reset` built, at the new
`RP2040` after a MicroPython reset, at the engine the `DelegatingEsp32Bridge`
switched to. No device is told.

```ts
interface SpiControllerPort {
  readonly bus: 'spi';
  readonly unit: number;                 // the SoC's index, as in the pin function table
  readonly name: string;                 // 'SPI0', 'VSPI', 'SPIM3', for diagnostics
  readonly remote?: boolean;             // the master runs in a backend worker
  setFrameHandler(handler: ((mosi: number, bits: number) => number) | null): void;
  setBlockHandler?(handler: ((mosi: Uint8Array, miso: Uint8Array | null) => void) | null): void;
  config(): SpiControllerConfig;         // enabled, mode?, bitOrder?, bits?, hz?
  routing(): SpiRouting | 'static';      // sck, mosi, miso, cs[] right now
  setHardwareCsHandler?(handler: ((index: number, active: boolean) => void) | null): void;
  setRoutingChangeHandler?(handler: (() => void) | null): void;
}
```

The rules every adapter is held to (DESIGN section 4, checked by the
conformance kits):

1. **Exactly one answer per frame, synchronous.** The adapter calls the frame
   handler once per frame the controller clocks and hands the returned MISO
   to the engine once, for that frame, however the engine wants it
   (overwriting SPDR on AVR, one `completeTransmit` on RP2040/RP2350, a
   return value on XIAO and the ESP32 engines). Answering zero times (the
   RP2350 and AVR hangs) or twice (the RP2350's shifted FIFO) is impossible
   by construction.
2. **Stable identity** across reset, Stop/Run, reload and engine swap.
3. **Every controller the SoC has**, not only the first: SPI1 on the RP2040
   and RP2350, both GP-SPI on the ESP32, each SPIM/SERCOM/RSPI/EUSART of the
   XIAO ARM boards, USART0 to USART3 on the Mega.
4. **The real route when the engine knows it** (funcsel on RP2040, the GPIO
   matrix on ESP32, PSEL on nRF); otherwise the board's static table.
5. **A pad routed to a peripheral produces no GPIO edges.**

The I2C port hands the fabric every bus event and takes the answer back
synchronously:

```ts
interface I2cControllerPort {
  readonly bus: 'i2c'; readonly unit: number; readonly name: string; readonly remote?: boolean;
  setTransactionHandler(handler: I2cTransactionHandler | null): void;
  routing(): I2cRouting | 'static';
  setRoutingChangeHandler?(handler: (() => void) | null): void;
}
interface I2cTransactionHandler {
  start(address: number, read: boolean): boolean;  // START or repeated START: the address ACK
  write(byte: number): boolean;                     // the data ACK
  read(): number;                                   // 0xFF when nobody drives SDA
  stop(): void;
}
```

The UART port transmits through a TX handler and takes bytes in through
`receive`:

```ts
interface UartControllerPort {
  readonly bus: 'uart'; readonly unit: number; readonly name: string; readonly remote?: boolean;
  setTxHandler(handler: ((byte: number) => void) | null): void;  // once per byte the guest shifts out
  receive(byte: number): void;                                    // lands in the guest's RX once
  config(): UartConfig;                                           // baud and frame, undefined until the guest set them
  routing(): UartRouting | 'static';
  setRoutingChangeHandler?(handler: (() => void) | null): void;
}
```

`config()` reports what the guest configured, never an invented default: a
port whose guest has not called `begin()` reports no rate, and the fabric
checks nothing against it.

### Routing

Which board pin carries which signal comes from two sources, and the live one
wins:

1. **Pin function tables** (`pinFunctions.ts`, one per board kind, registered
   with `registerBoardPinFunctions`). A table lists the controllers of the SoC
   (`ControllerDef`: bus, unit, datasheet name, the Arduino objects bound to
   it, the pins `begin()` uses without arguments) and, per board pin, every
   function it can take. `routing` says how far the table can be trusted:
   `'fixed'` (ATmega328P), `'mux'` (RP2040 funcsel, SAMD21 PMUX: every
   alternate is listed, the engine says which is live), `'matrix'` (ESP32 GPIO
   matrix, nRF PSEL: any GPIO can carry any signal, the table holds the IO_MUX
   pins and the Arduino defaults). OSS tables are under `boardPinTables/`;
   the pro overlay registers its own boards.
2. **`routing()` of the port**, when the engine reports it. A sketch that
   calls `SPI.begin(38, 39, 37)` or `Serial1.begin(9600, SERIAL_8N1, 16, 17)`
   moves the controller and the fabric follows, through
   `setRoutingChangeHandler`.

One entry of the classic ESP32 table is worth knowing because the silkscreen
disagrees with it: on arduino-esp32 3.x, `Serial2.begin()` with no pins is
**GPIO4 (RX2) and GPIO25 (TX2)**, not the 16/17 printed on the DevKit and used
in the 1.0.x-era tutorials (`HardwareSerial.h`, and `boardPinTables/esp32.ts`
says so). A module wired to 16/17 answers only a sketch that names the pins.

### The guest clock

Parts and decoders run on the **guest's** clock, never the browser's: an
emulated board under load runs slower than real time, and a bit time measured
on the wall clock is garbage to a sketch that samples on `millis()`. An engine
that can time its pads hands the fabric a `GuestClock`:

```ts
interface GuestClock {
  now(): number;                                   // guest cycles now (in a pin callback: the edge's cycle)
  clockHz(): number;
  scheduleEdge(pin: number, level: boolean, atCycle: number): void;  // in order, at the cycle, never skipped by idle skip
  at(atCycle: number, cb: () => void): () => void; // run cb when the guest gets there
}
```

An engine without one cannot host a software UART: an endpoint on plain
GPIOs of that board is reported (`uart-no-clock`) instead of silently hearing
nothing. Parts that pace themselves (the GPS) read the same clock through
`guestMillis(simulator)` (`parts/partUtils.ts`); custom chips read it through
`vx_sim_now_nanos` and their timers.

### The ports each engine exposes

| Engine | SPI | I2C | UART | Routing |
|---|---|---|---|---|
| AVR (Uno, Nano, Mega): `AVRSimulator.ts` | SPI (the ATmega's one controller) | TWI (`I2CBusManager` is the port) | USART0; USART0 to USART3 on the Mega | fixed table |
| ATtiny85 | none (avr8js gives no byte callback for the USI): `shiftOut` reaches devices through the software decoder | the USI on PB0/PB2 through the software decoder | none | fixed table |
| RP2040: `RP2040Simulator.ts` | SPI0, SPI1 | I2C0, I2C1 | UART0, UART1 | live, funcsel |
| RP2350 (pro): `RP2350Simulator.ts` | SPI0, SPI1 | I2C0, I2C1 | UART0, UART1 | live, funcsel over 48 GPIO |
| XIAO ARM x5 (pro): `XiaoArmSimulator.ts` | SPIM (nRF52840, nRF54L15), SERCOM (SAMD21), RSPI (RA4M1), EUSART (MG24) | one per controller of the SoC | UARTE, SERCOM, SCI, (E)USART | live, from the engine's registers (PSEL, PMUX, PFS, ROUTE) |
| ESP32 in-browser x6 (pro): `esp32sim/partBus.ts`, owned by the `DelegatingEsp32Bridge` | the GP-SPI controllers each SoC has | two masters on the classic, S3 and P4; one on C3, C6, C5 | UART0 to UART2 (classic, S3), 0 to 1 (C3, C6, C5), 0 to 4 (P4) | live, GPIO matrix and IO_MUX |
| Raspberry Pi, both engines: `PiBridgeShim.ts` | SPI0 and SPI1, the CE lines as hardware chip selects | I2C0 (GPIO0/1), I2C1 (GPIO2/3) | UART0 (the PL011 on GPIO14/15) | static |
| ESP32 QEMU, STM32 QEMU: `remotePort.ts`, `remoteI2c.ts`, `remoteUart.ts` | `RemoteSpiPort`, fed by the worker's relayed bytes | `RemoteI2cPort` | `RemoteUartPort` | ESP32: the worker reads the live matrix; STM32: table |

The ports of the in-browser ESP32 survive Stop/Run, reset, reload and the
fall-back to QEMU and back, because they belong to the delegating bridge and
not to the engine instance it holds.

### The conformance kits

One suite per bus that every adapter must pass, run against the **real**
engine with a guest that really clocks transactions: a firmware fixture
compiled with the production toolchain, or the engine's own registers written
the way the core's driver writes them. The kit owns the expectations; the
engine test supplies a rig that knows how to make its guest talk.

```ts
// frontend/src/simulation/buses/conformance/
defineSpiPortConformance(title, makeRig: () => Promise<SpiConformanceRig | null>, opts?)
defineI2cPortConformance(title, makeRig: () => Promise<I2cConformanceRig | null>, opts?)
defineUartPortConformance(title, makeRig: () => Promise<UartConformanceRig | null>, opts?)
```

An `SpiConformanceRig` names the controller `units`, a chip-select GPIO per
unit, returns the `binding()` (called again after rebuilds), runs a list of
`GuestTransaction`s and reports what the guest read back, and drives the
product's own `reset()`, `stopRun()` and `reload()` paths. The cases, per
engine and per controller:

| Case | What it proves |
|---|---|
| echo | the MISO returned is what the firmware reads for that byte (the probe answers a function of the byte and its position, so a stream shifted by one cannot match by accident) |
| one answer | no hang without an answer, no shifted FIFO with two |
| idle | with nobody selected the firmware reads the line's idle level |
| identity | the same device still hears after reset, Stop/Run, reload, MicroPython reset, engine swap |
| every controller | SPI0 and SPI1, both GP-SPI, each SPIM, SERCOM, RSPI, EUSART |
| real route | after `SPI.begin(pins)` or a funcsel write, `routing()` matches the sketch |
| no GPIO echo | a pad routed to the peripheral produces no edge in the PinManager |
| CS at the instant | a GPIO chip select written just before the byte is seen by that byte |
| block equals byte | a DMA or W-buffer transaction gives the same result as byte by byte |

The I2C kit checks the ACK of an address nobody has, a data NACK, a repeated
START, every controller and identity; the UART kit checks that a 40-byte
pattern is transmitted once and in order, received in the guest's RX,
survives a rebuild, keeps directions apart, reports a `config()` within
`baudsMatch` and echoes nothing on routed pads.

**How a new engine proves itself**: implement `getBusBinding()`; write one
test file per bus that calls the kit with a rig for the real engine (the
existing ones are the templates: `__tests__/board-buses/port-conformance-avr.test.ts`
and its `-i2c` and `-uart` siblings, `port-conformance-rp2040*.test.ts`,
`port-conformance-pi*.test.ts`; in the pro overlay `port-conformance-esp32`,
`-rp2350*`, `-xiao*`); compile the guest with
`project/board-buses-2026-09/harness/compile-fixture.mjs` and commit the
`.ino`, the `.hex` (or `.bin`) and the `compile.log` under `fixtures/`, with
the rebuild command in the sketch's header comment; then break the adapter on
purpose, one mutation at a time, and record how many rows fall. A row that
passes before and after the fix proves nothing and is rewritten.

---

## The device contract

The three entry points are exported from `frontend/src/simulation/buses`
(`index.ts`). A part calls one of them from its `attachEvents` and returns the
handle's `dispose()` from its cleanup. The part receives no new context: it
names its pins, the registry resolves the nets and the board.

### attachSpiDevice

```ts
attachSpiDevice(desc: SpiDeviceDescriptor, device: SpiDevice): BusHandle
```

| Descriptor field | Meaning |
|---|---|
| `owner` | identity, unique (the component id, or `builtin:<boardId>:<name>`) |
| `componentId?` | the component whose pin names `pins` refers to; defaults to `owner` |
| `pins.sck`, `pins.mosi?`, `pins.miso?`, `pins.cs?` | the device's own pin names, or an explicit `PinRef` (`{ kind: 'board', boardId, pin }` for a built-in). Omit a leg the chip does not have; omit `cs` only for a chip with no select line |
| `csActive?` | `'low'` (default) or `'high'` |
| `csWhenFloating?` | what an undriven CS means for this chip: `'selected'` for a module with its own pull to the active level; default deselected plus a diagnostic |
| `modes?` | the SPI modes the chip accepts; default any |
| `bitOrder?` | `'msb'` (default) or `'lsb'` |
| `remoteModel?()` | the chip as a portable model (`RemoteSpiModel`: `wasmB64`, `pinMap?`, `chipPads?`, `attrs?`, `blobs?`, `blobIds?`), or `null` while it has nothing to send. See the remote-host section |
| `remoteAttrs?()` | the model's live inputs, read now |
| `remoteBlobWrite?(name, offset, data, blobId?)` | the hosted model wrote into a named blob; keep the tab's copy in step |
| `remoteKeepsTabCopy?` | the tab still decodes the bytes clocked under this device's select while a worker answers its MISO (a panel that answers its id) |

```ts
interface SpiDevice {
  readonly writeOnly?: boolean;                 // this model never drives MISO, in any state
  select?(): void;                              // CS went active: a transaction starts
  deselect?(): void;                            // CS went inactive: frame state resets
  transfer(mosi: number, bits: number): number | null;  // the MISO for this frame, or null (high impedance)
  peekMiso?(): number | null;                   // what the NEXT frame will shift out, without consuming
  transferBlock?(mosi: Uint8Array): void;       // fast path for sinks: a whole block, as if byte by byte with null
  boardReset?(): void;                          // the MCU was reset: protocol state, not data
}
```

**Responders and sinks.** Whether a device answers is a fact about the model,
not the silkscreen. A display that implements no read command leaves its SDO
leg in high impedance for the whole run: it returns `null` from `transfer`,
sets `writeOnly: true`, and may still declare `miso` so a wire on that leg is
checked like any other. The `writeOnly` flag is what keeps a TFT sharing its
bus with a card from being reported as a responder the QEMU worker cannot
host. A responder (a card, a touch controller, an ADC) returns bytes,
implements `peekMiso` for a bit-banged master, resets its frame state in
`deselect`, and carries a portable model for the remote lanes.

The handle's `attrsChanged()` is the one method beyond `dispose()`: call it
whenever the device's live inputs move (a finger, a solved voltage, a slider).
It is cheap enough for every pointer move and every circuit solve; the
registry sends the new values only when the device is on a remote board, has
a model, and the values changed.

### attachI2cTarget

```ts
attachI2cTarget(desc: I2cTargetDescriptor, target: I2cTarget): BusHandle
```

| Descriptor field | Meaning |
|---|---|
| `owner`, `componentId?` | as for SPI |
| `pins.scl`, `pins.sda` | the device's own pin names or explicit refs |
| `addresses` | every 7-bit address the chip answers; they all belong to this registration and all leave with its handle |
| `remoteModel?` | the record type a QEMU worker has a model for (`'mpu6050'`, `'ssd1306'`, `'custom-chip'`...; the set is `WORKER_I2C_MODELS`). Leave it out for a chip the worker cannot host; on a remote bus it is then reported instead of silently absent |

```ts
interface I2cTarget {
  start(address: number, read: boolean): boolean;  // one of its addresses: return the ACK
  write(byte: number): boolean;                     // a data byte: return the ACK
  read(): number;                                   // the next byte the controller clocks out
  stop(): void;                                     // once per STOP the target took part in
  boardReset?(): void;
  readonly mayNak?: boolean;                        // start() or write() may return false while present
}
```

The fabric only calls a target for traffic addressed to it. `mayNak` exists
for the Raspberry Pi relay, which ACKs writes itself for every target that
does not say it may refuse them, and asks the tab only for the ones that do
(a user's custom chip).

Most parts do not implement `I2cTarget` by hand: `attachI2cPart`
(`parts/i2cPart.ts`) adapts the register-file `I2CDevice` shape the parts
already have (`writeByte`, `readByte`, `stop`, and the optional `start` and
`boardReset`) and, on a QEMU board, also
files the worker record the guest is answered from. The
[part author guide](./board-buses-part-authoring.md#an-i2c-target) shows it.

### attachUartEndpoint

```ts
attachUartEndpoint(desc: UartEndpointDescriptor, endpoint: UartEndpoint): UartHandle
```

| Descriptor field | Meaning |
|---|---|
| `owner`, `componentId?` | as above |
| `pins.rx?`, `pins.tx?` | the device's own legs, each placed on its own net. A GPS has only a TX, a display that takes commands only an RX, a modem both |
| `baud?` | the rate the device talks at. It is what the fabric checks a controller against, the rate a byte to a plain GPIO is clocked at, and the bit time a bit-banged byte to it is decoded with. Leave it out for a device that takes any rate (a terminal): nothing is reported for it, and it cannot sit on a software UART |
| `frame?` | `'8N1'` (default), `'7E1'`, `'8N2'`, Arduino style |

```ts
interface UartEndpoint { receive(byte: number): void; boardReset?(): void; }
interface UartHandle extends BusHandle { transmit(byte: number): void; }  // out of the TX leg; dropped when that leg reaches no board pin
```

There is no default UART. An endpoint whose leg is wired to nothing is on no
wire, hears nothing, and is heard by nobody, without a diagnostic: that is
what an unwired module does on the bench. The old `classifyPin` route, which
put a module on an unknown pin onto UART0 (the console), is gone.

### Board built-ins

A board's own panel, card slot, IMU or RTC is a device of the fabric like any
part; the only difference is where its pins come from. A part names its own
pin names and the fabric walks the wires; a built-in names the **board pins**
it is soldered to. The pro overlay's helpers
(`pro/frontend/src/pro/boards/builtinSpiDevice.ts`, `builtinI2cTarget.ts`):

```ts
attachBuiltinSpiDevice(boardId, name, { sck, mosi?, miso?, cs? }, device, extra?): () => void
attachBuiltinI2cTarget(boardId, name, { sda, scl }, addresses, target): () => void
```

The owner is `builtin:<boardId>:<name>`, so two boards of one kind on a
canvas are two devices and a remount replaces the device instead of adding a
twin. A built-in gates on its real chip select: the M5Stack panel and the
microSD that share VSPI behave as they do on the board, and the per-engine
hole that only one built-in could sit in (`setSPIHandler`) is gone with the
CS-keyed routers that were stacked in front of the byte path (F3).

### Custom chips

A chip enters a bus **when it calls** `vx_spi_attach`, `vx_i2c_attach` or
`vx_uart_attach`, with the pins of its config, not when the part mounts
(`customChips/ChipRuntime.ts`, `_joinSpiBus`, `_joinI2cBus`, `_joinUartBus`).
A Grove UART model no longer touches SPI. `vx_spi_config.cs` is honoured (a
chip receives bytes only while selected; `cs = NO_PIN` means always
selected), `mode` is compared with the controller's, every address of a
multi-address chip goes into one descriptor and leaves with it, and
`vx_sim_now_nanos` and the timers run on the board's guest clock in every
host (F7, with a cross-host parity table under
`__tests__/board-buses/fixtures/chips-abi-parity/`). The chip-side details are
in [custom-chips-api-reference.md](./custom-chips-api-reference.md).

An analog read is the circuit's too (F8): `vx_pin_read_analog` answers the
voltage the solve publishes for the pad's net in every host, and
`vx_pin_wired` tells a wired pad from one in the air, so a model with a
slider per channel reads the slider only where no wire arrives. A chip in a
worker or beside the Pi guest is handed those numbers as `pad_volts`, sent
with its record and again on every solve that moves one of its pads. The
level another part injects on a board pin (`setPinState`) reaches the
PinManager's level channel on every engine, not only the AVR, and on a pin a
chip drives the channel carries the net's resolution: the PORT latch of an
input pin and an injection are proposals the board net answers
(`PinManager.claimLevel`), never a glitch.

### The bus maps sent to a remote host

For a board whose processor runs in a backend worker, the tab computes
membership as for any board and sends the result:

- **SPI** (`busRegistry.remoteSpiPublication(boardId)`): one
  `RemoteSpiMapEntry` per responder with a portable model (`owner`,
  `bus_id` or `null` when the tab cannot tell, `cs` as `pin` / `hw` / `const`
  / `none`, and the `model` with `wasm_b64`, `pin_map`, `attrs`, `blobs`,
  `blob_ids`), followed by one `sinks` entry: the chip selects of every device
  the tab keeps, so the worker relays a byte only while one of them could be
  selected, or `all` when it cannot follow a select.
- **I2C** (the `i2c` half of the same map): per placed target, its `owner`,
  the controller (`bus_id`) or the SDA/SCL board pins when only the live
  routing can say, and its `addresses`; plus `unplaced`, the owners the worker
  holds a record for that the fabric put on no wire, so it keeps them silent.
- **UART** (`busRegistry.uartMap(boardId)`): per endpoint with a leg on that
  board, `rx_uart` (the controller whose TX feeds its RX), `tx_uart`, the
  pins, `baud` and `frame`, or `null` for a leg on no controller.
- **Pulls** (`busNets.boardPinPulls(pinManager)`, `buses/remotePulls.ts`):
  every module resistor on the board's pins as `{pin, pull: 'up'|'down',
  owner}`, for the worker's pad model (see "Pull resistors on a line").
- **Raspberry Pi**: `PiBridgeShim.busTopology()` carries the SPI responders
  as `spi.responders` (the same entries, the CS as a GPIO the relay maps to a
  CE) inside `pi_bus_topology`.

The maps go with `start_*` and again on every membership change
(`onSpiMapChange`, `onI2cMapChange`, `onUartMapChange`,
`onBoardPinPullsChange`, coalesced per task). A half that is absent from a
live map leaves the worker's copy of it as it was.
Field names are the wire's, which is Python's.

---

## Remote hosts: the responder runs beside the master

On the ESP32 and STM32 QEMU boards the CPU is in a backend worker, and QEMU
asks for the MISO of a byte, the ACK of an address and every I2C byte
**synchronously**, in its own callback. Nothing in the browser can answer in
time: the old `esp32_spi_response` path sent one socket message per MISO byte
and the worker applied it to some later byte, which is why a touch controller
answered with the coordinates of an earlier command. On the Raspberry Pi the
guest blocks on every `ioctl` and waits for the tab: correct, and tens of
milliseconds per transaction, an SD sector being dozens of them.

So whatever drives MISO or SDA runs in the same host as the CPU that reads it
(D-004), as one portable model: a chip compiled against `velxio-chip.h` to
WASM, run by the same runtime everywhere (`ChipRuntime.ts` in the tab,
`wasm_chip_runtime.py` in the ESP32 and STM32 workers and on the Pi host). The
JS part stays as the interface: it paints, it collects the user's input, and
it ships the model with its inputs. Seven responders travel this way: the
microSD (`buses/models/microsd.c`, the one source for the canvas part, every
board's built-in slot and the QEMU lanes), XPT2046, MCP3008, MAX6675,
MAX31856, MAX31865 and the PAG7661, plus the M5Stack panel's id
(`lcd-panel-id.wasm`) and the Round Display's card slot. The artifacts sit
under `public/bus-chips/` (OSS `frontend/public/bus-chips/microsd.wasm`, the
pro ones under `pro/frontend/public/bus-chips/`), fetched once per page by
`loadBusChip(name)` and handed to `remoteModel()` synchronously through
`busChipB64(name)`; an artifact that lands after the first map was published
makes the registry publish again (`spiModelsChanged`).

Where the pieces live:

| Host | Map in | Per-byte arbitration | Live inputs | Written blobs out |
|---|---|---|---|---|
| ESP32 QEMU worker (`app/services/esp32_worker.py`) | `bus_map` command (`esp32_bus_map` on the socket, `EspLibManager.set_bus_map`) | `_on_spi_event` and `_on_spi_batch`: selected = the responders whose CS is active (GPIO state, the last hardware-CS event, constants); one answers, several are ANDed with a `bus_diag`, none is `0xFF` | `bus_attrs` (`esp32_bus_attrs`, `set_bus_attrs`) | `bus_blob` event |
| STM32 QEMU worker (`pro/backend/app/pro_boards/stm32_worker.py`) | `stm32_bus_map` | the same, by GPIO chip select only (the STM32 bridge reports no hardware CS events) | `stm32_bus_attrs` | none: the tab's card follows the relayed bytes |
| Raspberry Pi host (`pi_bus_relay.py`, `pi_spi_responders.py`) | `pi_bus_topology`, `spi.responders` | per transaction: a CE with a hosted model answers the whole `SPI_IOC_MESSAGE` here; the select is replayed as an edge on a pseudo GPIO the model watches | `pi_bus_attrs` | `bus_blob` |

A host keeps a running model across maps while its identity holds
(`hosted_model_identity` in `wasm_chip_runtime.py`: the wasm bytes, the
select, `bus_id`, `pin_map` and the blob **ids**, never the blob contents or
the attributes). The reason is a race: the map carries the blobs as the tab
last knew them, and what the guest wrote comes back later; rebuilding the card
from a republished map (a wire moved while a file was being written) would
undo the write. Identity by id, not content, is what tells "the tab is
behind" from "the user swapped the card".

### The blob ABI

A model with storage (the card image) uses the header's named blobs:

```c
uint32_t vx_blob_size (const char* name);
uint32_t vx_blob_read (const char* name, uint32_t offset, uint8_t* dst, uint32_t len);
uint32_t vx_blob_write(const char* name, uint32_t offset, const uint8_t* src, uint32_t len);
```

A blob exists because the host declared it for the instance and never grows.
On the wire it travels base64 per name in `model.blobs`, with `model.blob_ids`
naming which image each one is (the microSD mints a new id in its
constructor and in `loadImage`, never on a guest write). What the hosted
model writes comes back as a span (`bus_blob`: owner, blob name, offset,
bytes, the blob id it belongs to); the registry hands it to the device's
`remoteBlobWrite` (`applyRemoteBlob`), and the tab discards a span whose id is
not the image it holds now. A responder whose model carries blobs but takes
no spans back is kept as a sink so its bytes keep being relayed: without
either path its copy would fall silently behind the guest.

Decision D-013 belongs here too: `vx_spi_config` gained `on_exchange(user_data,
mosi) -> miso` so a chip can answer the byte it is receiving. The MCP3008 puts
its result bits in the same byte as the configuration bits that decide them,
and the pre-armed buffer of `vx_spi_start` cannot express that (2.00 V read as
0.35 V). Both runtimes implement it; an old chip leaves the field at 0.

### Live attributes

The map is static and goes once per membership change; a finger on a touch
glass moves at pointer rate. The two travel apart:

- The descriptor's `remoteAttrs()` returns the inputs as they are now, under
  the model's attribute names (`vx_attr_read`); the same names a map entry
  carries in `model.attrs`, and they win over those.
- The part calls `handle.attrsChanged()` where its input changes: the XPT2046
  on pointer down, move and up, before it drives T_IRQ (same socket, in
  order); the MCP3008 on every solve of the electrical store; the MAX chips
  from the sensor panel and the thermal-zone follower; the PAG7661 from its
  panel and camera.
- The registry sends `bus_attrs {owner, attrs}` only if the device is on a
  bus, has a model and the values changed; a published map counts as sent.
  The host applies them to the running model (`update_attrs`), rebuilding
  nothing. With the socket closed the bridge patches its saved map, so the
  next `start_*` carries what the user sees.

Measured (STATUS, F4): in the ESP32 worker rig a new value is read by the next
guest transaction (p50 1.8 ms from the command); on a real Pi 4 guest the
hosted MCP3008 read 620 counts (2.00 V) in 10 to 13 ms per `xfer2`, against
74.8 ms through a tab 60 ms away, and a moved input was seen by the first
read after it.

### Sinks stay in the tab

Displays, e-paper and LED drivers are heavy to paint and never answer, so
they stay JS in the tab and receive the bytes the worker relays
(`spi_batch`, framed by the chip-select and D/C edges around them) through
the `RemoteSpiPort`. The `sinks` entry at the end of every map lists their
chip selects so the worker keeps to itself a transaction nothing in the tab
could see (a card read), which is what brought the streaming-read cost down.
A device that is hosted for its answer and decoded in the tab for its writes
(the M5Stack panel: hosted for the id M5GFX probes, a sink for the pixels)
says `remoteKeepsTabCopy: true`, and stays a sink as well.

### What the microSD costs on a QEMU board

The single model has a price inside the worker, measured and accepted by the
operator on 2026-09-24 (D-014): a streaming read (CMD18) costs 2.7 to 3.2 ms
per sector against 2.1 to 2.4 ms for the Python card it replaced, a single
sector up to 1.5 ms more, and 1 MB read by CMD18 takes 0.6 to 1.8 s longer.
The cost is the model's own: about 14 calls into wasm per SD command from the
Python runtime, after the runtime's per-byte path was already cut 7x (561 to
83 us per byte) and the model taught to deliver a block in one burst. What it
buys is one card that answers the same on every engine: the three copies it
replaced had already drifted (ACMD51 missing in one, the real CRC16 of CMD59
in another, the N_CR byte that kept cards from mounting on the Pico). Displays
saw no regression; five benches are faster than the F0 baseline. If a real
case shows the cost (high-rate logging, audio from the card), D-014 is
reopened.

### I2C and UART on a remote lane

- **I2C.** The worker answers every event from its own models, so a target
  exists for the guest only if the worker has a model of it. A part sends its
  record (`registerSensor(type, chipVirtualPin(componentId), { addr, owner })`),
  the worker keeps it in an `I2cBusTable` keyed by (controller, address) and
  by identity, and the tab's `i2c` map says which controller the part's SDA is
  on. A target whose `remoteModel` is not in `WORKER_I2C_MODELS` is reported
  (`bus-remote-responder-missing`). Write sinks (a display) are hosted as an
  echo: the worker ACKs and relays the writes, and the tab's copy draws from
  them.
- **UART.** A chip the worker hosts is on the unit the `uart_bus_table`
  resolves, in this order: the live routing of the pad its RX leg is on (the
  ESP32 matrix), then the tab's `uart` map, then nothing. A chip on no unit
  hears nothing and is heard by nobody. The worker has no bit-timed pins, so
  there is no software UART there, and Serial1 is not a place a wire leads.
  On the Pi the host runs no UART chips: the Grove UART modules run in the
  tab and answer through `PiUartPort`.
- What a remote host cannot do, said in the code: the STM32 bridge reports
  no hardware CS events (a chip there learns its select from a GPIO only);
  the Pi relay registers one I2C address per chip; the Pi host runs no chip
  timers; QEMU (C) is not changed (custom-chips decision D6).

---

## Decisions by number

| Decision | Rule in this page |
|---|---|
| D-001 | one fabric per board with electrical arbitration; the listener chain is gone |
| D-002 | membership from the netlist and the pin function tables |
| D-003 | engine port with stable identity and one answer per frame |
| D-004 | the responder runs beside the master, as a portable WASM model |
| D-005 | software buses through pin-level decoders, one device contract |
| D-006 | diagnostics are part of the model |
| D-007 | no performance regression; block APIs for block traffic |
| D-008 | what the engine does not expose is added to the engine (hardware CS on the ESP32 engines, live routes on SAMD21, RA4M1, EFR32, the Mega's USART1 to 3), never invented by the adapter |
| D-009 | migration on one branch, no permanent double paths |
| D-010 | the demo I2C devices at 0x48, 0x50 and 0x68 are gone; a bus with nothing on it answers nothing |
| D-011 | wrong SPI mode is a diagnostic, not bit emulation |
| D-012 | idle MISO is the net's level, `0xFF` |
| D-013 | `on_exchange`: a chip may answer the byte it receives |
| D-014 | the microSD's cost in the QEMU worker is accepted |

---

## Files and tests

| File | Role |
|---|---|
| `frontend/src/simulation/buses/types.ts` | every contract in this page |
| `buses/registry.ts` | `busRegistry`: attach, placement, chip-select sources, the remote maps, diagnostics |
| `buses/fabric.ts` | `BoardBusFabric`: ports to buses, routing, software decoders, wiring checks |
| `buses/spiBus.ts`, `i2cBus.ts`, `uartBus.ts` | arbitration per bus kind |
| `buses/softSpi.ts`, `softI2c.ts`, `softUart.ts`, `uartFrame.ts` | the software buses and the UART frame maths |
| `buses/pinFunctions.ts`, `boardPinTables/` | the static routing tables |
| `buses/storeResolver.ts`, `boardPins.ts` | the store's net resolver and the PinManager adapter |
| `customChips/busNets.ts` | a board pin as a net with drivers: chips, the MCU's pad, module pulls (`setBoardPinPull`) |
| `buses/remotePort.ts`, `remoteI2c.ts`, `remoteUart.ts`, `remoteLane.ts`, `remotePulls.ts` | the ports and maps of a board whose CPU is in a worker |
| `buses/busChips.ts`, `models/` | portable model artifacts; `models/microsd.c` with `build.sh` and `manifest.json` (source sha, checked by the suites) |
| `buses/conformance/` | the three kits |
| `backend/app/services/wasm_chip_runtime.py`, `i2c_bus_table.py`, `uart_bus_table.py`, `pad_model.py`, `esp32_worker.py` | the worker side |
| `pro/backend/app/pro_boards/stm32_worker.py`, `pi_bus_relay.py`, `pi_spi_responders.py` | the STM32 and Pi hosts (velxio-prod) |
| `buses/__tests__/` | layer 1: arbitration tables, order permutations, decoders, cross-board I2C and UART |
| `frontend/src/__tests__/board-buses/` | conformance suites, F0 reproduction rows (`board-buses-repro-*`), the F4 to F7 suites, `board-buses-module-pulls.test.ts`, fixtures |
| `buses/__tests__/soft-i2c-bus-pullups.test.ts` | SoftwareWire with its pull-ups off, on a real PinManager |
| `test/backend/unit/test_board_buses_*.py`, `test_wasm_chip_*.py`, `test_microsd_wasm_model.py`, `test_pad_model.py` | the worker, the runtime, the microSD golden table, the pad model |
| `test/backend/integration/test_esp32_qemu_pads.py`, `test_esp32_qemu_sd_card.py` | the ESP32 and S3 workers on the real libqemu-xtensa with compiled sketches |
| `project/board-buses-2026-09/harness/` (velxio-prod) | the in-app matrix, the perf bench, the fixture compiler, the cost and latency harnesses |
