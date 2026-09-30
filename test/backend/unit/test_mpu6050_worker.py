"""
test_mpu6050_worker.py: the MPU-6050 a QEMU worker hosts (project
i2c-model-fidelity-2026-09).

The model itself is held to the shared bus vectors in test_i2c_slaves.py. This
file is about what only the worker does with it, on the rig of the board-buses
suite (app/services/esp32_worker.py running unmodified, libqemu replaced at the
ctypes boundary, the guest played by calling the worker's own callbacks):

  - the worker's copy starts from the values its sensor record carries, which
    are where the panel's sliders are, and a later update changes only what it
    names. The copy used to start from values of its own and to take the
    record's at the first slider move: the gallery sketch printed 25.0 C
    beside a panel that said 24;
  - answering an I2C event logs nothing and emits nothing unless
    VELXIO_I2C_TRACE is set, and with it set the trace is what it always was.
"""
from __future__ import annotations

import time

import pytest

# The rig loads custom chips, so its module skips itself where wasmtime is
# missing; that skip is handed to the fixture, as the F5 suite does.
try:
    from .test_board_buses_repro_worker import (  # noqa: F401  (worker fixture)
        I2C_FINISH,
        I2C_READ,
        I2C_START_RECV,
        I2C_START_SEND,
        I2C_WRITE,
        worker,
    )
except pytest.skip.Exception as _no_rig:
    _RIG_MISSING = str(_no_rig)
    I2C_START_RECV, I2C_START_SEND, I2C_FINISH, I2C_WRITE, I2C_READ = 0x00, 0x01, 0x03, 0x05, 0x06

    @pytest.fixture
    def worker():
        pytest.skip(_RIG_MISSING)

ADDR = 0x68
PIN = 200 + ADDR
PWR_MGMT_1, ACCEL_XOUT_H = 0x6B, 0x3B

# Where a project left the sliders: nothing here is the value the panel, the
# tab model or the twin starts from.
PANEL = {'accelX': 0.5, 'accelY': -0.25, 'accelZ': 0.75,
         'gyroX': 100, 'gyroY': -50, 'gyroZ': 10, 'temp': 30}
# The same, as counts at the power-on ranges (16384 per g, 131 per deg/s) and
# (T - 36.53) * 340.
PANEL_COUNTS = [8192, -4096, 12288, -2220, 13100, -6550, 1310]
REST_COUNTS = [0, 0, 16384, -4260, 0, 0, 0]


def record(**values) -> dict:
    """The record the tab's part files for the worker (parts/i2cPart.ts)."""
    return {'sensor_type': 'mpu6050', 'pin': PIN, 'addr': ADDR, 'owner': 'imu1', **values}


def write_reg(w, reg: int, *values: int) -> None:
    ret = w.i2c(0, ADDR, [I2C_START_SEND, I2C_WRITE | (reg << 8),
                          *[I2C_WRITE | (v << 8) for v in values], I2C_FINISH])
    assert ret[:-1] == [0] * (len(ret) - 1), 'the chip acknowledges its address and every byte'


def read_sample(w) -> list[int]:
    """The 14-byte burst of getEvent(), as seven signed counts."""
    ret = w.i2c(0, ADDR, [I2C_START_SEND, I2C_WRITE | (ACCEL_XOUT_H << 8), I2C_FINISH,
                          I2C_START_RECV, *[I2C_READ] * 14, I2C_FINISH])
    block = ret[4:18]
    words = [(block[i] << 8) | block[i + 1] for i in range(0, 14, 2)]
    return [v - 0x10000 if v & 0x8000 else v for v in words]


def stderr_after(w, marker: str, timeout: float = 5.0) -> list[str]:
    """The worker's log up to the line that holds `marker`. The log is one
    pipe written in order, so whatever came before the marker is in."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        lines = list(w.stderr)
        for i, line in enumerate(lines):
            if marker in line:
                return lines[:i]
        time.sleep(0.02)
    raise AssertionError(f'"{marker}" never reached the log:\n' + ''.join(w.stderr[-20:]))


def event_lines(log: list[str]) -> list[str]:
    """The lines the worker logs per I2C event, without its prefix."""
    lines = [line.rstrip('\n').split('] ', 1)[-1] for line in log]
    return [line for line in lines if line.startswith(('I2C #', 'I2C bus='))]


class TestSeededFromTheRecord:
    def test_the_start_config_record_sets_the_first_read(self, worker):
        w = worker(sensors=[record(**PANEL)])
        write_reg(w, PWR_MGMT_1, 0x00)
        assert read_sample(w) == PANEL_COUNTS

    def test_a_record_attached_while_running_sets_the_first_read(self, worker):
        w = worker()
        w.send({'cmd': 'sensor_attach', **record(**PANEL)})
        w.sync()
        write_reg(w, PWR_MGMT_1, 0x00)
        assert read_sample(w) == PANEL_COUNTS

    def test_a_record_with_no_values_reads_the_panels_rest(self, worker):
        """A tab that sends no values gets 1 g on Z and 24 C, which is where
        its panel starts."""
        w = worker(sensors=[record()])
        write_reg(w, PWR_MGMT_1, 0x00)
        assert read_sample(w) == REST_COUNTS

    def test_an_update_changes_what_it_names_and_nothing_else(self, worker):
        """One slider moves. The temperature used to fall back to 25 C with
        it, and every value the record never carried to the twin's own."""
        w = worker(sensors=[record(**PANEL)])
        write_reg(w, PWR_MGMT_1, 0x00)
        w.send({'cmd': 'sensor_update', 'pin': PIN, 'accelX': -1})
        w.sync()
        assert read_sample(w) == [-16384, *PANEL_COUNTS[1:]]

    def test_the_chip_is_asleep_until_the_sketch_wakes_it(self, worker):
        w = worker(sensors=[record(**PANEL)])
        assert w.read_reg(0, ADDR, PWR_MGMT_1) == (0, 0x40)
        assert read_sample(w) == [0] * 7


class TestI2cTrace:
    def _traffic(self, w) -> None:
        write_reg(w, PWR_MGMT_1, 0x00)
        assert w.read_reg(0, ADDR, PWR_MGMT_1) == (0, 0x00)

    def test_silent_unless_asked(self, worker, monkeypatch):
        monkeypatch.delenv('VELXIO_I2C_TRACE', raising=False)
        w = worker(sensors=[record()])
        self._traffic(w)
        # Both are behind the traffic now: an event emitted after it, and a
        # log line written after it.
        w.flush()
        w.send({'cmd': 'sensor_attach', 'sensor_type': 'trace-marker', 'pin': 77})
        log = stderr_after(w, 'Sensor trace-marker attached')
        assert w.events('i2c_trace') == []
        assert event_lines(log) == []

    def test_the_trace_is_what_it_was_when_asked_for(self, worker, monkeypatch):
        monkeypatch.setenv('VELXIO_I2C_TRACE', '1')
        w = worker(sensors=[record()])
        self._traffic(w)
        w.flush()
        w.send({'cmd': 'sensor_attach', 'sensor_type': 'trace-marker', 'pin': 77})
        log = stderr_after(w, 'Sensor trace-marker attached')

        assert w.events('i2c_trace') == [
            {'type': 'i2c_trace', 'bus': 0, 'addr': ADDR, 'event': ev, 'op': op,
             'result': result, 'reg_ptr': ptr}
            for ev, op, result, ptr in [
                (I2C_START_SEND, 'START_SEND', 0, 0x00),
                (I2C_WRITE | (PWR_MGMT_1 << 8), 'WRITE', 0, 0x6B),
                (I2C_WRITE, 'WRITE', 0, 0x6C),
                (I2C_FINISH, 'FINISH', 0, 0x6C),
                (I2C_START_SEND, 'START_SEND', 0, 0x6C),
                (I2C_WRITE | (PWR_MGMT_1 << 8), 'WRITE', 0, 0x6B),
                (I2C_FINISH, 'FINISH', 0, 0x6B),
                (I2C_START_RECV, 'START_RECV', 0, 0x6B),
                (I2C_READ, 'READ', 0x00, 0x6C),
                (I2C_FINISH, 'FINISH', 0, 0x6C),
            ]
        ]
        assert event_lines(log) == [
            'I2C #001 bus=0 addr=0x68 START_SEND → reg_ptr=0x00',
            'I2C #002 bus=0 addr=0x68 WRITE byte=0x6b → reg_ptr=0x6b',
            'I2C #003 bus=0 addr=0x68 WRITE byte=0x00 → reg_ptr=0x6c',
            'I2C #004 bus=0 addr=0x68 FINISH ',
            'I2C #005 bus=0 addr=0x68 START_SEND → reg_ptr=0x6c',
            'I2C #006 bus=0 addr=0x68 WRITE byte=0x6b → reg_ptr=0x6b',
            'I2C #007 bus=0 addr=0x68 FINISH ',
            'I2C #008 bus=0 addr=0x68 START_RECV → reg_ptr=0x6b',
            'I2C #009 bus=0 addr=0x68 READ → PWR_MGMT1=0x00',
            'I2C #010 bus=0 addr=0x68 FINISH ',
        ]
