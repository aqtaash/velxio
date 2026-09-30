"""
The end-to-end script of the MPU-6050 (test/backend/e2e/test_mpu6050_simulation.mjs)
against a backend that is not there: its diagnostics have to work on the day
they are needed.

The script runs against a live backend and a real QEMU, so nothing runs it in
a test suite, and the branches a passing run never takes are never executed.
The one for an I2C address no model answers is the one that says why a run
failed (the sensor record was not registered, or the sketch scans the bus).
Here a backend of a few lines answers the compile request, sends the events
of that case over the WebSocket and closes; the script has to print them and
reach its verdict.

The script opens its socket with the global WebSocket, as the other e2e
scripts do, and Node has that from version 22. This file sits in the unit
suite, which the deploy gate and CI run whole with whatever Node the machine
has, so the test skips on a Node that cannot run the script.
"""

import json
import shutil
import socket
import subprocess
import threading
import time
from pathlib import Path

import pytest
import uvicorn
from fastapi import FastAPI, WebSocket

SCRIPT = Path(__file__).parent.parent / 'e2e' / 'test_mpu6050_simulation.mjs'

# What the worker reports for an address with no model, as esp32_worker.py
# encodes it: the operation in the low byte, the data byte above it.
UNANSWERED = [
    (0x0001, 'START_SEND'),
    (0x7505, 'WRITE'),
    (0x0003, 'FINISH'),
    (0x0000, 'START_RECV'),
    (0x0006, 'READ'),
    (0x0004, 'NACK'),
    (0x0002, 'START_ASYNC'),
]


def _backend() -> FastAPI:
    app = FastAPI()

    @app.post('/api/compile/')
    async def compile_():
        return {'success': True, 'binary_content': 'AAAA', 'has_wifi': False}

    @app.websocket('/api/simulation/ws/{session}')
    async def simulation(ws: WebSocket, session: str):
        await ws.accept()
        await ws.receive_text()                       # start_esp32
        for event, _name in UNANSWERED:
            await ws.send_text(json.dumps({'type': 'i2c_event', 'data': {
                'bus': 0, 'addr': 0x68, 'event': event, 'response': 1}}))
        await ws.send_text(json.dumps({'type': 'serial_output', 'data': {
            'data': 'MPU6050 not found! Check wiring.\r\n'}}))
        await ws.close()

    return app


@pytest.fixture
def backend_url():
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(_backend(), log_level='error', lifespan='off'))
    thread = threading.Thread(target=server.run, kwargs={'sockets': [sock]}, daemon=True)
    thread.start()
    deadline = time.monotonic() + 10
    while not server.started and time.monotonic() < deadline:
        time.sleep(0.02)
    assert server.started, 'the stand-in backend did not start'
    yield f'http://127.0.0.1:{port}'
    server.should_exit = True
    thread.join(timeout=10)
    sock.close()


def _node_for_the_script() -> str:
    """The node on PATH, or a skip when it cannot run the script.

    The probe asks for the global itself and not for the version: Node 21 has
    it behind a flag, and NODE_OPTIONS can take it away from a later one.
    """
    node = shutil.which('node')
    if node is None:  # pragma: no cover - the e2e scripts need node anyway
        pytest.skip('node not available')
    try:
        probe = subprocess.run([node, '-p', 'typeof WebSocket'],
                               capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as exc:
        pytest.skip(f'node did not answer the probe: {exc}')
    if probe.returncode != 0 or probe.stdout.strip() != 'function':
        pytest.skip('the e2e script needs node 22 or later (global WebSocket)')
    return node


def test_skips_on_a_node_without_the_global_websocket(monkeypatch):
    if shutil.which('node') is None:  # pragma: no cover
        pytest.skip('node not available')
    # What Node 20 looks like to the script, on any Node: no global WebSocket.
    monkeypatch.setenv('NODE_OPTIONS', '--no-experimental-websocket')
    with pytest.raises(pytest.skip.Exception, match='node 22 or later'):
        _node_for_the_script()


def test_an_address_no_model_answers_is_reported_not_a_crash(backend_url):
    node = _node_for_the_script()
    run = subprocess.run(
        [node, str(SCRIPT), f'--backend={backend_url}', '--timeout=20'],
        capture_output=True, text=True, timeout=60)
    out = run.stdout + run.stderr

    assert 'ReferenceError' not in out, out
    for event, name in UNANSWERED:
        assert f'event=0x{event:04x} op={name} ' in run.stdout, (name, out)
    # The verdict is the script's, not an uncaught exception's.
    assert 'I2C events (no slave registered): 7' in run.stdout, out
    assert 'mpu.begin() returned false' in run.stdout, out
    assert run.returncode == 1
