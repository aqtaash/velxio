/**
 * What a register-file model hears of the bus through i2cTargetOf.
 *
 * The models were written against writeByte / readByte / stop, and that is
 * still all most of them need. Two things could not be said that way:
 *
 *   - where a read begins. The MPU-6050 answers a whole burst from one
 *     sampling instant, so it has to take its sample at the START of the
 *     read and not byte by byte;
 *   - that the MCU was reset while the chip kept its supply, which is when a
 *     note a model gave to the run that ended is due again.
 *
 * Both are optional on I2CDevice. A model that defines neither is driven
 * exactly as before.
 */
import { describe, it, expect } from 'vitest';
import { i2cTargetOf } from '../simulation/parts/i2cPart';
import type { I2CDevice } from '../simulation/I2CBusManager';

const ADDR = 0x68;

/** A model that writes down what it is told. */
function listener(hears: { start?: boolean; boardReset?: boolean } = {}) {
  const log: string[] = [];
  const device: I2CDevice = {
    address: ADDR,
    writeByte: (v) => {
      log.push(`w${v.toString(16)}`);
      return true;
    },
    readByte: () => {
      log.push('r');
      return 0xa5;
    },
    stop: () => {
      log.push('stop');
    },
  };
  if (hears.start) device.start = (read) => void log.push(read ? 'START r' : 'START w');
  if (hears.boardReset) device.boardReset = () => void log.push('reset');
  return { device, log, target: i2cTargetOf(device) };
}

describe('i2cTargetOf: START reaches the model', () => {
  it('every START and repeated START, with its direction, before the first byte of the phase', () => {
    const { target, log } = listener({ start: true });
    expect(target.start(ADDR, false)).toBe(true);
    target.write(0x3b);
    expect(target.start(ADDR, true)).toBe(true);
    target.read();
    target.read();
    target.stop();
    expect(log).toEqual(['START w', 'w3b', 'START r', 'r', 'r', 'stop']);
  });

  it('a repeated START for writing still ends the pointer phase first', () => {
    const { target, log } = listener({ start: true });
    target.start(ADDR, false);
    target.write(0x75);
    target.start(ADDR, true);
    target.read();
    target.start(ADDR, false);
    target.write(0x6b);
    target.stop();
    expect(log).toEqual(['START w', 'w75', 'START r', 'r', 'stop', 'START w', 'w6b', 'stop']);
  });

  it('a STOP and then a START, which is how QEMU delivers a repeated START', () => {
    const { target, log } = listener({ start: true });
    target.start(ADDR, false);
    target.write(0x3b);
    target.stop();
    target.start(ADDR, true);
    target.read();
    target.stop();
    expect(log).toEqual(['START w', 'w3b', 'stop', 'START r', 'r', 'stop']);
  });

  it('a model that does not ask for it is driven as it always was', () => {
    const { target, log } = listener();
    target.start(ADDR, false);
    target.write(0x3b);
    target.start(ADDR, true);
    target.read();
    target.stop();
    target.boardReset?.();
    expect(log).toEqual(['w3b', 'r', 'stop']);
  });
});

describe('i2cTargetOf: the MCU reset reaches the model', () => {
  it('between transactions: the model hears the reset and no STOP', () => {
    const { target, log } = listener({ boardReset: true });
    target.start(ADDR, false);
    target.write(0x6b);
    target.stop();
    target.boardReset?.();
    expect(log).toEqual(['w6b', 'stop', 'reset']);
  });

  it('in the middle of one: the STOP it will never get, then the reset', () => {
    const { target, log } = listener({ boardReset: true });
    target.start(ADDR, false);
    target.write(0x6b);
    target.boardReset?.();
    expect(log).toEqual(['w6b', 'stop', 'reset']);
    // The transaction went with the old run: the next START opens a new one.
    target.start(ADDR, false);
    target.write(0x75);
    expect(log.slice(3)).toEqual(['w75']);
  });
});
