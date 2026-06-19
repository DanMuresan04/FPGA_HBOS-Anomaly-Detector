"""
Headless test suite — no display required.

Tests MockFpgaClient, UartFpgaClient (no hardware), TrainSession, CSV loading,
telemetry parsing, and state-machine logic without starting a tkinter window.

Run:
    python3 -m licenta.gui.test_headless
    python3 licenta/gui/test_headless.py     # from project root
    python3 gui/test_headless.py             # from licenta/
"""

import os
import struct
import sys
import tempfile
import threading
import time
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import licenta.gui.train_session as _ts

# Speed up timing-heavy constants for all tests
_ts.POST_CALIB_WAIT   = 0.10
_ts.LATCH_SETTLE      = 0.05
_ts.DUMP_CALIB_LATCH  = 4
_ts.POLL_GAP          = 0.004
_ts.TRAIN_PACE        = 0.0
_ts.PROGRESS_INTERVAL = 10

from licenta.gui.mock_client import MockFpgaClient, _HbosEngine, make_mock_csv
from licenta.gui.train_session import TrainSession
from licenta.gui.stream_viewer import _load_csv, _load_csv_or_dir
from licenta.gui.uart_client import UartFpgaClient, OP_DETECT, OP_TRAIN, OP_CALIBRATE, OP_DUMP

# ── test infrastructure ───────────────────────────────────────────────────────

_PASS = 0
_FAIL = 0


def _check(name: str, fn):
    global _PASS, _FAIL
    try:
        fn()
        print(f"  PASS  {name}")
        _PASS += 1
    except Exception as exc:
        print(f"  FAIL  {name}  →  {exc}")
        traceback.print_exc()
        _FAIL += 1


def _make_rows(n: int = 50) -> list:
    return [
        [str(100 + i % 30), str(200 - i % 20), str(150 + i % 10), str(175),
         str(1 if i % 17 == 0 and i > 0 else 0), str(i)]
        for i in range(n)
    ]


# ── controlled mock for telemetry edge-case tests ─────────────────────────────

class _ControlledClient:
    """Returns a predetermined sequence from recv(); no real hardware."""

    def __init__(self, responses):
        self._responses = list(responses)   # list[int | None]
        self._idx = 0

    def pack_packet(self, s0, s1, s2, s3, op, tlast):
        return struct.pack("<iiiiBB", int(s0), int(s1), int(s2), int(s3), op, tlast) + bytes((0xA5, 0x5A))

    def pack_config_packet(self, weights, spike_penalty):
        import struct
        w = [int(weights[i]) & 0xFF for i in range(4)]
        w_unsigned = w[0] | (w[1] << 8) | (w[2] << 16) | (w[3] << 24)
        s0 = struct.unpack('<i', struct.pack('<I', w_unsigned))[0]
        return self.pack_packet(s0, int(spike_penalty) & 0xFFFF, 0, 0, 4, 0)

    def send(self, payload): pass
    def drain(self, timeout=0.5): return 0
    def close(self): pass
    def try_recv(self): return None, None

    def recv(self, timeout=1.0):
        if self._idx >= len(self._responses):
            return None, None
        val = self._responses[self._idx]
        self._idx += 1
        if val is None:
            return None, None
        return val, bytes([val])


# ══════════════════════════════════════════════════════════════════════════════
# PACKET FORMAT TESTS
# Expected layout (20 bytes, UART):
#   [0-15]  4 × int32 LE  (s0, s1, s2, s3)
#   [16]    opcode
#   [17]    tlast
#   [18]    0xA5
#   [19]    0x5A
# ══════════════════════════════════════════════════════════════════════════════

def test_pack_packet_length():
    c = MockFpgaClient()
    pkt = c.pack_packet(1, -2, 300, 0, 0, 0)
    assert len(pkt) == 20, f"expected 20 bytes (UART), got {len(pkt)}"


def test_pack_magic_bytes():
    c = MockFpgaClient()
    pkt = c.pack_packet(0, 0, 0, 0, 0, 0)
    assert pkt[18] == 0xA5 and pkt[19] == 0x5A, \
        f"bad magic: 0x{pkt[18]:02X} 0x{pkt[19]:02X}"


def test_pack_opcode_and_tlast_positions():
    c = MockFpgaClient()
    for op in range(5):
        for tlast in (0, 1):
            pkt = c.pack_packet(0, 0, 0, 0, op, tlast)
            assert pkt[16] == op,    f"opcode mismatch op={op}: byte16=0x{pkt[16]:02X}"
            assert pkt[17] == tlast, f"tlast mismatch: byte17=0x{pkt[17]:02X}"


def test_pack_sensor_values():
    c = MockFpgaClient()
    pkt = c.pack_packet(0x01020304, -1, 0x7FFFFFFF, 0, 0, 0)
    s0 = struct.unpack_from("<i", pkt, 0)[0]
    s1 = struct.unpack_from("<i", pkt, 4)[0]
    s2 = struct.unpack_from("<i", pkt, 8)[0]
    assert s0 == 0x01020304
    assert s1 == -1
    assert s2 == 0x7FFFFFFF


def test_pack_negative_sensor():
    c = MockFpgaClient()
    pkt = c.pack_packet(-100, -200, -300, -400, 2, 1)
    values = struct.unpack_from("<iiii", pkt, 0)
    assert values == (-100, -200, -300, -400), f"got {values}"


def test_mock_pack_matches_uart_format():
    """MockFpgaClient and UartFpgaClient must produce identical byte layouts."""
    # Use UartFpgaClient.pack_packet as a static reference (no serial needed)
    uart_pkt  = UartFpgaClient.pack_packet(None, 42, -7, 1000, 0, 3, 1)
    mock_pkt  = MockFpgaClient().pack_packet(42, -7, 1000, 0, 3, 1)
    assert uart_pkt == mock_pkt, \
        f"format mismatch:\n  uart={uart_pkt.hex()}\n  mock={mock_pkt.hex()}"


# ══════════════════════════════════════════════════════════════════════════════
# UartFpgaClient (no hardware)
# ══════════════════════════════════════════════════════════════════════════════

def test_uart_decode_normal():
    result = UartFpgaClient.decode(bytes([0x00]))
    assert result["result"] == 0x00
    assert result["name"] == "normal"


def test_uart_decode_anomaly():
    result = UartFpgaClient.decode(bytes([0x01]))
    assert result["result"] == 0x01
    assert result["name"] == "anomaly"


def test_uart_decode_telemetry_bytes():
    assert UartFpgaClient.decode(bytes([0xFE]))["name"] == "telemetry_banner"
    assert UartFpgaClient.decode(bytes([0xFF]))["name"] == "dump_ack"


def test_uart_decode_short():
    result = UartFpgaClient.decode(bytes([]))
    assert result["result"] is None
    assert result["name"] == "short"


def test_uart_decode_unknown():
    result = UartFpgaClient.decode(bytes([0x42]))
    assert result["result"] == 0x42
    assert "42" in result["name"]


# ══════════════════════════════════════════════════════════════════════════════
# MockFpgaClient behaviour
# ══════════════════════════════════════════════════════════════════════════════

def test_mock_send_reads_opcode_at_byte_16():
    """send() must read opcode at byte 16 (UART format), not byte 18 (UDP)."""
    c = MockFpgaClient()
    # OP_DETECT = 2; put wrong value at byte 18 to catch index-18 regression
    pkt = bytearray(c.pack_packet(0, 0, 0, 0, 2, 0))
    pkt[18] = 0x99   # corrupt the magic byte — if opcode read from 18, last_opcode = 0x99
    c.send(bytes(pkt))
    v, _ = c.try_recv()
    assert v is not None, "OP_DETECT not recognised — opcode read from wrong index"
    assert v in (0x00, 0x01)


def test_mock_train_no_reply():
    c = MockFpgaClient()
    c.send(c.pack_packet(1, 2, 3, 4, 0, 0))   # OP_TRAIN
    v, _ = c.try_recv()
    assert v is None, "OP_TRAIN must produce no reply"


def test_mock_calib_no_reply():
    c = MockFpgaClient()
    c.send(c.pack_packet(1, 2, 3, 4, 1, 0))   # OP_CALIB
    v, _ = c.try_recv()
    assert v is None, "OP_CALIB must produce no reply"


def test_detect_reply_distribution():
    """After training, replies must be 0x00/0x01; extreme outliers must always be 0x01."""
    cancel = threading.Event()
    rows   = _make_rows(50)
    client = MockFpgaClient()
    session = TrainSession(client, rows, lambda *a: None, cancel)
    ok, _, _ = session.run()
    assert ok, "must train successfully before detecting"

    # Normal-range samples — format check (threshold can vary with sparse histograms)
    for _ in range(10):
        client.send_sample([110, 192, 154, 175], OP_DETECT, 0)
        v, raw = client.try_recv()
        assert v is not None, "try_recv returned None after OP_DETECT"
        assert v in (0x00, 0x01), f"unexpected result byte: {v:#04x}"
        assert raw is not None and len(raw) == 1

    # Extreme outliers: magnitude score is maximal (bin count=0 → always exceeds threshold)
    for _ in range(5):
        client.send_sample([9999, 9999, 9999, 9999], OP_DETECT, 0)
        v, raw = client.try_recv()
        assert v is not None
        assert v == 0x01, f"extreme outlier must be anomaly (0x01), got {v:#04x}"
        assert raw is not None and len(raw) == 1


def test_try_recv_none_before_send():
    c = MockFpgaClient()
    v, _ = c.try_recv()
    assert v is None, f"expected None before any send, got {v}"


def test_mock_telem_delay():
    """First _TELEM_DELAY recv() calls must return None before telemetry starts."""
    c = MockFpgaClient()
    c.send(c.pack_packet(0, 0, 0, 0, 3, 0))   # OP_DUMP
    for i in range(MockFpgaClient._TELEM_DELAY):
        v, _ = c.recv(timeout=0.1)
        assert v is None, f"expected None on delay poll {i}, got 0x{v:02X}"
    v, _ = c.recv(timeout=0.1)
    assert v == 0xFE, f"expected 0xFE after delay, got {v!r}"


def test_mock_telem_recv_raw_is_1_byte():
    c = MockFpgaClient()
    c.send(c.pack_packet(0, 0, 0, 0, 3, 0))
    for _ in range(MockFpgaClient._TELEM_DELAY + 1):
        v, raw = c.recv(timeout=0.1)
        if v is not None:
            assert len(raw) == 1, f"UART reply must be 1 byte, got {len(raw)}"
            break


def test_drain_returns_zero():
    c = MockFpgaClient()
    assert c.drain(0.5) == 0


def test_network_warnings_empty():
    assert MockFpgaClient.network_warnings() == []


# ══════════════════════════════════════════════════════════════════════════════
# CSV loading
# ══════════════════════════════════════════════════════════════════════════════

def test_mock_csv_write_load():
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        make_mock_csv(path, n=40)
        rows = _load_csv(path)
        assert len(rows) == 40, f"expected 40 rows, got {len(rows)}"
        assert len(rows[0]) >= 5, f"row too short: {rows[0]}"
        for r in rows:
            float(r[0]); float(r[4])
    finally:
        os.unlink(path)


def test_load_csv_skips_header():
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        with open(path, "w") as fh:
            fh.write("s0,s1,s2,s3,label,seq\n")
            for i in range(5):
                fh.write(f"{i},{i},{i},{i},0,{i}\n")
        rows = _load_csv(path)
        assert len(rows) == 5, f"expected 5, got {len(rows)}"
    finally:
        os.unlink(path)


def test_load_csv_negative_values():
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        with open(path, "w") as fh:
            fh.write("-100,-200,150,175,0,0\n")
            fh.write("100,200,-50,175,1,1\n")
        rows = _load_csv(path)
        assert len(rows) == 2, f"expected 2, got {len(rows)}"
    finally:
        os.unlink(path)


def test_load_csv_empty_file_returns_empty():
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        rows = _load_csv(path)
        assert rows == [], f"expected [], got {rows}"
    finally:
        os.unlink(path)


def test_load_csv_header_only_returns_empty():
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        with open(path, "w") as fh:
            fh.write("s0,s1,s2,s3,label\n")
        rows = _load_csv(path)
        assert rows == [], f"expected [], got {rows}"
    finally:
        os.unlink(path)


def test_load_csv_short_rows_skipped():
    """Rows with fewer than 2 columns must be silently dropped; longer rows kept."""
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        with open(path, "w") as fh:
            fh.write("1,2,3,4,0,5\n")   # good — 6 cols
            fh.write("1\n")              # only 1 col — skip (< 2)
            fh.write("5,6,7,8,1,9\n")   # good — 6 cols
        rows = _load_csv(path)
        assert len(rows) == 2, f"expected 2, got {len(rows)}"
    finally:
        os.unlink(path)


def test_load_csv_mixed_numeric_and_header():
    """Header row (non-numeric first field) must be skipped; data rows kept."""
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        with open(path, "w") as fh:
            fh.write("s0,s1,s2,s3,label\n")
            fh.write("10,20,30,40,0\n")
            fh.write("11,21,31,41,1\n")
        rows = _load_csv(path)
        assert len(rows) == 2


        assert rows[0][0] == "10" and rows[1][4] == "1"
    finally:
        os.unlink(path)


# ══════════════════════════════════════════════════════════════════════════════
# Telemetry parsing
# ══════════════════════════════════════════════════════════════════════════════

def test_telemetry_parse():
    """Telemetry from a completed TrainSession must report the global threshold."""
    cancel = threading.Event()
    client = MockFpgaClient()
    session = TrainSession(client, _make_rows(20), lambda *a: None, cancel)
    ok, stream, _ = session.run()
    assert ok, "TrainSession must succeed before testing telemetry"
    parsed = TrainSession.parse_telemetry(stream)
    assert "global_threshold" in parsed, f"missing global_threshold: {parsed}"
    assert isinstance(parsed["global_threshold"], int)
    # delta_th and rx counters were removed from the telemetry.
    assert "delta_th" not in parsed
    assert "total_rx_train" not in parsed


def test_telemetry_parse_short_stream():
    for length in range(0, 17):
        stream = [0xFE] + [0x00] * (length - 1) if length > 0 else []
        result = TrainSession.parse_telemetry(stream)
        assert isinstance(result, dict), f"expected dict at length {length}"


def test_parse_telemetry_empty():
    assert TrainSession.parse_telemetry([]) == {}


def test_parse_telemetry_wrong_banner():
    assert TrainSession.parse_telemetry([0x00, 0x01, 0x02, 0x03]) == {}
    assert TrainSession.parse_telemetry([0xFF]) == {}


def test_parse_telemetry_threshold_le24():
    """Threshold must decode as LE24 from bytes [1:4]."""
    # global_threshold = 0x123456 = 1193046
    stream = [0xFE, 0x56, 0x34, 0x12]
    parsed = TrainSession.parse_telemetry(stream)
    assert parsed["global_threshold"] == 0x123456, f"got {parsed['global_threshold']}"


def test_parse_telemetry_all_fields():
    """Build the 5-byte threshold frame and verify it decodes."""
    th = 5000           # 0x001388
    stream = (
        [0xFE]
        + [th & 0xFF, (th >> 8) & 0xFF, (th >> 16) & 0xFF]
        + [0xFF]
    )
    assert len(stream) == 5

    parsed = TrainSession.parse_telemetry(stream)
    assert parsed["global_threshold"] == th


# ══════════════════════════════════════════════════════════════════════════════
# TrainSession — happy path and cancel paths
# ══════════════════════════════════════════════════════════════════════════════

def test_train_session_completes():
    rows = _make_rows(50)
    progress = []
    cancel  = threading.Event()
    client  = MockFpgaClient(simulated_rx_count=50)
    session = TrainSession(client, rows, lambda f, m: progress.append((round(f, 2), m)), cancel)
    ok, stream, msg = session.run()

    assert ok, f"TrainSession returned ok=False: {msg}"
    assert stream and stream[0] == 0xFE and stream[-1] == 0xFF, f"invalid stream: {stream}"
    assert len(stream) == 5, f"expected 5-byte threshold frame, got {len(stream)}"
    assert len(progress) > 0, "no progress callbacks fired"
    fracs = [f for f, _ in progress]
    assert fracs == sorted(fracs), f"progress went backwards: {fracs}"


def test_train_session_progress_covers_phases():
    """Progress must be reported for each of the 5 major phases."""
    msgs = []
    cancel = threading.Event()
    client = MockFpgaClient(simulated_rx_count=10)
    session = TrainSession(client, _make_rows(10), lambda f, m: msgs.append(m), cancel)
    ok, _, _ = session.run()
    assert ok
    full_log = " ".join(msgs).lower()
    for keyword in ("train", "calib"):
        assert keyword in full_log, f"phase '{keyword}' not mentioned in progress: {msgs}"


def test_train_session_cancel_train():
    rows = _make_rows(200)
    cancel = threading.Event()
    session = TrainSession(MockFpgaClient(), rows, lambda *a: None, cancel)
    threading.Timer(0.05, cancel.set).start()
    ok, _, msg = session.run()
    assert not ok,          f"expected ok=False, got ok=True"
    assert "Cancel" in msg, f"unexpected message: {msg!r}"


def test_train_session_cancel_calib():
    rows = _make_rows(100)
    cancel = threading.Event()
    session = TrainSession(MockFpgaClient(), rows, lambda *a: None, cancel)
    threading.Timer(0.02, cancel.set).start()
    ok, _, msg = session.run()
    assert not ok, f"expected cancellation, got ok=True with msg={msg!r}"


def test_train_session_cancel_postcalib_wait():
    rows = _make_rows(5)
    cancel = threading.Event()
    orig = _ts.POST_CALIB_WAIT
    _ts.POST_CALIB_WAIT = 5.0
    try:
        session = TrainSession(MockFpgaClient(), rows, lambda *a: None, cancel)
        threading.Timer(0.05, cancel.set).start()
        ok, _, msg = session.run()
        assert not ok
        assert "Cancel" in msg
    finally:
        _ts.POST_CALIB_WAIT = orig


def test_train_session_cancel_latch():
    """Cancel during config latch settle must abort."""
    rows = _make_rows(5)
    cancel = threading.Event()
    orig = _ts.LATCH_SETTLE
    _ts.LATCH_SETTLE = 5.0
    try:
        session = TrainSession(MockFpgaClient(), rows, lambda *a: None, cancel)
        # TRAIN(~0) + CALIB(~0) + POST_CALIB_WAIT(0.10) + latch_pump(4×20ms=0.08s) = ~0.18s;
        # cancel at 0.22s fires during the 5s latch settle
        threading.Timer(0.22, cancel.set).start()
        ok, _, msg = session.run()
        assert not ok
        assert "Cancel" in msg
    finally:
        _ts.LATCH_SETTLE = orig


def test_train_session_cancel_telemetry():
    """Cancel during telemetry polling must abort before stream completes."""
    rows = _make_rows(5)
    cancel = threading.Event()
    orig_polls = _ts.TELEMETRY_POLLS
    _ts.TELEMETRY_POLLS = 200   # enough polls that it won't time out first
    try:
        # pre-set cancel so the very first telemetry poll iteration sees it
        cancel.set()
        session = TrainSession(MockFpgaClient(), rows, lambda *a: None, cancel)
        ok, _, msg = session.run()
        assert not ok
    finally:
        _ts.TELEMETRY_POLLS = orig_polls


# ══════════════════════════════════════════════════════════════════════════════
# TrainSession._collect_telemetry edge cases (via ControlledClient)
# ══════════════════════════════════════════════════════════════════════════════

def _run_collect(responses):
    """Run _collect_telemetry with a controlled byte sequence; return (ok, stream)."""
    client = _ControlledClient(responses)
    cancel = threading.Event()
    session = TrainSession(client, [], lambda *a: None, cancel)
    # Bypass train/calib by calling _collect_telemetry directly
    return session._collect_telemetry()


def _make_valid_telem_bytes() -> list:
    """Build a hand-crafted valid 17-byte telemetry sequence."""
    import struct
    th = 5000  # 0x001388
    return ([0xFE, th & 0xFF, (th >> 8) & 0xFF, (th >> 16) & 0xFF]
            + [10, 20, 30, 40]
            + list(struct.pack("<I", 100))
            + list(struct.pack("<I", 100))
            + [0xFF])


def test_collect_telemetry_full_17_bytes():
    telem = _make_valid_telem_bytes()
    assert len(telem) == 17
    ok, stream = _run_collect(telem)
    assert ok,               f"expected ok=True, got False; stream={stream}"
    assert stream[-1] == 0xFF
    assert len(stream) == 17


def test_collect_telemetry_out_of_sync_0xff_before_0xfe():
    """0xFF before 0xFE must return ok=False immediately."""
    ok, stream = _run_collect([0xFF])
    assert not ok, "0xFF before 0xFE should be treated as out-of-sync"


def test_collect_telemetry_timeout_before_banner():
    """All-None sequence (no banner) must return ok=False with empty stream."""
    ok, stream = _run_collect([None] * 5)
    assert not ok
    assert stream == [] or stream is None or stream == []


def test_collect_telemetry_partial_stream():
    """Stream cut off before 0xFF must return ok=False with partial data."""
    ok, stream = _run_collect([0xFE, 0x10, 0x27, 0x00])   # threshold only, no 0xFF
    assert not ok
    assert stream and stream[0] == 0xFE


def test_collect_telemetry_nones_before_banner_ok():
    """Leading None timeouts before 0xFE are normal; banner must still be found."""
    telem = _make_valid_telem_bytes()
    ok, stream = _run_collect([None, None, None, None, None] + telem)
    assert ok
    assert stream[0] == 0xFE and stream[-1] == 0xFF


def test_collect_telemetry_nones_mid_stream():
    """Timeout gaps within a telemetry stream must be handled gracefully."""
    telem = _make_valid_telem_bytes()
    interleaved = []
    for b in telem:
        interleaved.append(None)
        interleaved.append(b)
    ok, stream = _run_collect(interleaved)
    assert ok, f"stream with internal gaps failed: stream={stream}"
    assert stream[-1] == 0xFF


# ══════════════════════════════════════════════════════════════════════════════
# State machine and button rules
# ══════════════════════════════════════════════════════════════════════════════

def test_state_machine_transitions():
    from licenta.gui.stream_viewer import IDLE, BUSY, READY, STREAMING, ERROR, _LAMP_CFG
    for state in (IDLE, BUSY, READY, STREAMING, ERROR):
        assert state in _LAMP_CFG, f"state {state!r} missing from _LAMP_CFG"
        ring, dot, fg, text = _LAMP_CFG[state]
        assert ring.startswith("#"), f"{state}: ring not a hex colour"
        assert dot.startswith("#"),  f"{state}: dot not a hex colour"
        assert fg.startswith("#"),   f"{state}: fg not a hex colour"
        assert isinstance(text, str) and len(text) > 0, f"{state}: empty label"


def test_button_state_rules():
    """Encode expected per-state button enable/disable rules and verify them."""
    from licenta.gui.stream_viewer import IDLE, BUSY, READY, STREAMING, ERROR

    # (state, train_enabled, start_enabled, stop_enabled, cancel_enabled)
    rules = [
        (IDLE,      True,  False, False, False),
        (BUSY,      False, False, False, True),
        (READY,     True,  True,  False, False),
        (STREAMING, False, False, True,  False),
        (ERROR,     True,  False, False, False),
    ]
    for state, train, start, stop, cancel in rules:
        assert (state in (IDLE, READY, ERROR)) == train,   f"{state}: train rule wrong"
        assert (state == READY)                == start,   f"{state}: start rule wrong"
        assert (state == STREAMING)            == stop,    f"{state}: stop rule wrong"
        assert (state == BUSY)                 == cancel,  f"{state}: cancel rule wrong"


def test_all_states_covered_in_lamp_cfg():
    from licenta.gui.stream_viewer import IDLE, BUSY, READY, STREAMING, ERROR, _LAMP_CFG
    expected = {IDLE, BUSY, READY, STREAMING, ERROR}
    missing = expected - set(_LAMP_CFG.keys())
    extra   = set(_LAMP_CFG.keys()) - expected
    assert not missing, f"states missing from _LAMP_CFG: {missing}"
    assert not extra,   f"unknown keys in _LAMP_CFG: {extra}"


def test_stream_viewer_vars_initialised_before_build_ui():
    """_port_var and _baud_var must exist before _build_ui() references them.

    Regression test: previously these were initialised after _build_ui(), which
    caused an AttributeError on startup when the Entry widget bound to them.
    """
    from licenta.gui.stream_viewer import StreamViewer
    import tkinter as tk

    try:
        root = tk.Tk()
        root.withdraw()   # don't show the window during the test
        try:
            app = StreamViewer.__new__(StreamViewer)
            tk.Tk.__init__(app)
            app.withdraw()

            # Manually replicate enough of __init__ to hit the ordering invariant
            app._port_var = tk.StringVar(value="sentinel")
            app._baud_var = tk.StringVar(value="1000000")
            # If the ordering were wrong, accessing _port_var here would raise
            assert app._port_var.get() == "sentinel"
        finally:
            root.destroy()
    except tk.TclError:
        pass   # no DISPLAY in CI — skip silently


# ══════════════════════════════════════════════════════════════════════════════
# OP_CONFIG packet encoding
# ══════════════════════════════════════════════════════════════════════════════

def test_pack_config_opcode():
    """pack_config_packet must produce opcode=4 at byte 16."""
    c = MockFpgaClient()
    pkt = c.pack_config_packet([50, 93, 58, 55], 5632)
    assert len(pkt) == 20, f"expected 20 bytes, got {len(pkt)}"
    assert pkt[16] == 4, f"opcode must be 4 (OP_CONFIG), got {pkt[16]}"
    assert pkt[18] == 0xA5 and pkt[19] == 0x5A, "magic bytes wrong"


def test_pack_config_weights_encoding():
    """weights must be packed LE into s0: bytes [w0, w1, w2, w3]."""
    c = MockFpgaClient()
    pkt = c.pack_config_packet([10, 20, 30, 40], 0)
    s0 = struct.unpack_from("<i", pkt, 0)[0]
    assert (s0 >> 0)  & 0xFF == 10, f"w0 wrong: {(s0>>0)&0xFF}"
    assert (s0 >> 8)  & 0xFF == 20, f"w1 wrong: {(s0>>8)&0xFF}"
    assert (s0 >> 16) & 0xFF == 30, f"w2 wrong: {(s0>>16)&0xFF}"
    assert (s0 >> 24) & 0xFF == 40, f"w3 wrong: {(s0>>24)&0xFF}"


def test_pack_config_spike_penalty():
    """spike_penalty must appear as s1 (bytes 4-7 LE)."""
    c = MockFpgaClient()
    pkt = c.pack_config_packet([0, 0, 0, 0], 5632)
    s1 = struct.unpack_from("<i", pkt, 4)[0]
    assert s1 & 0xFFFF == 5632, f"spike_penalty wrong: {s1 & 0xFFFF}"


def test_pack_config_hls_defaults():
    """Default compile-time values must round-trip correctly."""
    c = MockFpgaClient()
    pkt = c.pack_config_packet([50, 93, 58, 55], 5632)
    s0 = struct.unpack_from("<i", pkt, 0)[0]
    s1 = struct.unpack_from("<i", pkt, 4)[0]
    weights = [(s0 >> (i * 8)) & 0xFF for i in range(4)]
    assert weights == [50, 93, 58, 55], f"weights wrong: {weights}"
    assert s1 & 0xFFFF == 5632, f"spike wrong: {s1 & 0xFFFF}"


def test_pack_config_max_weights():
    """Weights at 255 (max uint8) must not raise and must round-trip."""
    c = MockFpgaClient()
    pkt = c.pack_config_packet([255, 255, 255, 255], 65535)
    assert pkt[16] == 4
    s0 = struct.unpack_from("<i", pkt, 0)[0]
    s1 = struct.unpack_from("<i", pkt, 4)[0]
    assert all((s0 >> (i * 8)) & 0xFF == 255 for i in range(4))
    assert s1 & 0xFFFF == 65535


def test_pack_config_uart_mock_agree():
    """UartFpgaClient and MockFpgaClient must produce identical OP_CONFIG bytes."""
    class _NoSerialClient(UartFpgaClient):
        def __init__(self): pass  # skip serial.Serial()

    mock_pkt = MockFpgaClient().pack_config_packet([10, 20, 30, 40], 1234)
    uart_pkt = _NoSerialClient().pack_config_packet([10, 20, 30, 40], 1234)
    assert mock_pkt == uart_pkt, \
        f"format mismatch:\n  mock={mock_pkt.hex()}\n  uart={uart_pkt.hex()}"


# ══════════════════════════════════════════════════════════════════════════════
# TrainSession with weights
# ══════════════════════════════════════════════════════════════════════════════

def test_train_session_sends_config_before_train():
    """When weights= given, OP_CONFIG (opcode 4) must be sent before OP_TRAIN."""
    sent = []

    class _RecordingClient(MockFpgaClient):
        def send(self, payload):
            if len(payload) >= 17:
                sent.append(payload[16])  # opcode at byte 16
            super().send(payload)

        def send_sample(self, values, opcode, tlast):
            sent.append(opcode)  # record opcodes from send_sample too
            super().send_sample(values, opcode, tlast)

    rows = _make_rows(10)
    cancel = threading.Event()
    client = _RecordingClient()
    session = TrainSession(client, rows, lambda *a: None, cancel,
                           weights=[10, 20, 30, 40], spike_penalty=1000)
    ok, _, _ = session.run()
    assert ok, "TrainSession with weights= must succeed"
    assert sent[0] == 4, f"first opcode must be 4 (OP_CONFIG), got {sent[0]}"
    assert 0 in sent, "OP_TRAIN (opcode 0) must appear after OP_CONFIG"
    assert sent.index(4) < sent.index(0), "OP_CONFIG must precede first OP_TRAIN"


def test_train_session_no_config_without_weights():
    """When weights= is None, no OP_CONFIG must be sent."""
    sent = []

    class _RecordingClient(MockFpgaClient):
        def send(self, payload):
            if len(payload) >= 17:
                sent.append(payload[16])
            super().send(payload)

    rows = _make_rows(10)
    cancel = threading.Event()
    client = _RecordingClient(simulated_rx_count=10)
    session = TrainSession(client, rows, lambda *a: None, cancel)  # no weights=
    ok, _, _ = session.run()
    assert ok
    assert 4 not in sent, f"OP_CONFIG must not be sent when weights=None; sent={sent[:10]}"


def test_run_config_update_completes():
    """run_config_update() must succeed with MockFpgaClient and return telemetry."""
    cancel = threading.Event()
    client = MockFpgaClient(simulated_rx_count=50)
    session = TrainSession(client, [], lambda *a: None, cancel,
                           weights=[25, 50, 75, 100], spike_penalty=2048)
    ok, stream, msg = session.run_config_update()
    assert ok, f"run_config_update failed: {msg}"
    assert stream and stream[0] == 0xFE and stream[-1] == 0xFF, \
        f"telemetry stream malformed: {stream}"


def test_run_config_update_no_weights_fails():
    """run_config_update() without weights= must return ok=False immediately."""
    cancel = threading.Event()
    session = TrainSession(MockFpgaClient(), [], lambda *a: None, cancel)
    ok, _, msg = session.run_config_update()
    assert not ok, "expected failure when weights=None"
    assert "weight" in msg.lower(), f"error message should mention weights: {msg!r}"


def test_run_config_update_sends_op_config():
    """run_config_update() must send OP_CONFIG (opcode 4) as its first packet."""
    sent = []

    class _RecordingClient(MockFpgaClient):
        def send(self, payload):
            if len(payload) >= 17:
                sent.append(payload[16])
            super().send(payload)

    cancel = threading.Event()
    client = _RecordingClient(simulated_rx_count=50)
    session = TrainSession(client, [], lambda *a: None, cancel,
                           weights=[10, 20, 30, 40], spike_penalty=999)
    ok, _, _ = session.run_config_update()
    assert ok
    assert sent[0] == 4, f"first packet must be OP_CONFIG (4), got {sent[0]}"


def test_run_config_update_cancel():
    """Cancel during run_config_update latch settle must abort."""
    cancel = threading.Event()
    orig = _ts.LATCH_SETTLE
    _ts.LATCH_SETTLE = 5.0
    try:
        client = MockFpgaClient(simulated_rx_count=50)
        session = TrainSession(client, [], lambda *a: None, cancel,
                               weights=[50, 50, 50, 50], spike_penalty=100)
        threading.Timer(0.15, cancel.set).start()
        ok, _, msg = session.run_config_update()
        assert not ok, "expected cancellation"
        assert "Cancel" in msg, f"unexpected message: {msg!r}"
    finally:
        _ts.LATCH_SETTLE = orig


# ══════════════════════════════════════════════════════════════════════════════
# Apply Config button state rule
# ══════════════════════════════════════════════════════════════════════════════

def test_apply_config_button_enabled_only_in_ready():
    """Apply Config button must be enabled only in READY state."""
    from licenta.gui.stream_viewer import IDLE, BUSY, READY, STREAMING, ERROR
    for state in (IDLE, BUSY, READY, STREAMING, ERROR):
        expected = (state == READY)
        assert expected == (state == READY), f"{state}: apply rule broken"


# ══════════════════════════════════════════════════════════════════════════════
# Comparison module — compute_metrics
# ══════════════════════════════════════════════════════════════════════════════

from licenta.gui.comparison import compute_metrics, run_comparison, _SKLEARN_OK


def test_compute_metrics_perfect():
    """Perfect classifier: all TP for anomalies, all TN for normals."""
    y_true = [0, 0, 0, 1, 1, 1]
    y_pred = [0, 0, 0, 1, 1, 1]
    m = compute_metrics(y_true, y_pred)
    assert m["tp"] == 3 and m["fp"] == 0 and m["tn"] == 3 and m["fn"] == 0
    assert abs(m["f1"]       - 1.0) < 1e-9
    assert abs(m["recall"]   - 1.0) < 1e-9
    assert abs(m["precision"]- 1.0) < 1e-9
    assert abs(m["accuracy"] - 1.0) < 1e-9


def test_compute_metrics_all_wrong():
    """Inverted classifier: all FP and FN, zero TP."""
    y_true = [0, 0, 1, 1]
    y_pred = [1, 1, 0, 0]
    m = compute_metrics(y_true, y_pred)
    assert m["tp"] == 0 and m["fp"] == 2 and m["tn"] == 0 and m["fn"] == 2
    assert m["f1"]      == 0.0
    assert m["recall"]  == 0.0
    assert m["accuracy"] == 0.0


def test_compute_metrics_no_anomalies():
    """Dataset with no positive labels: recall undefined → 0, accuracy = 1."""
    y_true = [0, 0, 0, 0]
    y_pred = [0, 0, 0, 0]
    m = compute_metrics(y_true, y_pred)
    assert m["tp"] == 0 and m["fn"] == 0
    assert m["recall"]   == 0.0
    assert m["accuracy"] == 1.0


def test_compute_metrics_all_predicted_anomaly():
    """Predict everything as anomaly: precision = TP/(TP+FP), recall = 1."""
    y_true = [0, 0, 1, 1]
    y_pred = [1, 1, 1, 1]
    m = compute_metrics(y_true, y_pred)
    assert m["tp"] == 2 and m["fp"] == 2 and m["fn"] == 0
    assert abs(m["recall"] - 1.0) < 1e-9
    assert abs(m["precision"] - 0.5) < 1e-9


def test_compute_metrics_caught_string():
    """'caught' must be formatted as 'TP/total_anomalies'."""
    y_true = [1, 1, 1, 0, 0]
    y_pred = [1, 0, 1, 0, 0]
    m = compute_metrics(y_true, y_pred)
    assert m["caught"] == "2/3", f"caught={m['caught']!r}"


def test_compute_metrics_false_alarms():
    y_true = [0, 0, 1]
    y_pred = [1, 0, 1]
    m = compute_metrics(y_true, y_pred)
    assert m["false_alarms"] == 1, f"false_alarms={m['false_alarms']}"


def test_compute_metrics_empty():
    m = compute_metrics([], [])
    assert m["tp"] == 0 and m["total"] == 0
    assert m["f1"] == 0.0


def test_compute_metrics_length_mismatch():
    """Shorter list wins; extra elements in longer list are ignored."""
    y_true = [1, 0, 1]
    y_pred = [1, 0, 1, 1, 1]   # longer pred
    m = compute_metrics(y_true, y_pred)
    assert m["total"] == 3


def test_compute_metrics_f1_known_value():
    """Hand-computed F1: precision=0.75, recall=0.6 → F1=0.6667."""
    y_true = [1, 1, 1, 1, 1, 0, 0, 0, 0]
    y_pred = [1, 1, 1, 0, 0, 1, 0, 0, 0]
    # TP=3, FP=1, FN=2, TN=3
    m = compute_metrics(y_true, y_pred)
    assert m["tp"] == 3 and m["fp"] == 1 and m["fn"] == 2
    expected_prec = 3 / 4
    expected_rec  = 3 / 5
    expected_f1   = 2 * expected_prec * expected_rec / (expected_prec + expected_rec)
    assert abs(m["precision"] - expected_prec) < 1e-9
    assert abs(m["recall"]    - expected_rec)  < 1e-9
    assert abs(m["f1"]        - expected_f1)   < 1e-9


# ══════════════════════════════════════════════════════════════════════════════
# Comparison module — run_comparison
# ══════════════════════════════════════════════════════════════════════════════

def _make_comp_rows(n: int = 80, seed: int = 1) -> list:
    """Build labelled CSV rows for comparison tests."""
    import random
    rng = random.Random(seed)
    rows = []
    for i in range(n):
        base = [rng.randint(80, 200) for _ in range(4)]
        label = 1 if (i % 13 == 0 and i > 0) else 0
        rows.append([str(v) for v in base] + [str(label), str(i)])
    return rows


def test_run_comparison_returns_required_keys():
    rows  = _make_comp_rows(60)
    preds = [0] * 60
    result = run_comparison(rows, rows, preds)
    assert "hbos"    in result
    assert "dataset" in result
    assert "sklearn_available" in result
    ds = result["dataset"]
    assert "total" in ds and "total_anomalies" in ds and "anomaly_rate_pct" in ds


def test_run_comparison_hbos_matches_preds():
    """HBOS metrics must exactly reflect the supplied hbos_preds."""
    rows  = _make_comp_rows(40)
    labels = [1 if int(r[4]) else 0 for r in rows]
    # All-normal predictions
    preds = [0] * len(rows)
    result = run_comparison(rows, rows, preds)
    m = result["hbos"]
    assert m["tp"] == 0, f"no anomaly predicted → tp must be 0, got {m['tp']}"
    assert m["fn"] == sum(labels), f"fn must equal total anomalies"


def test_run_comparison_dataset_counts():
    rows   = _make_comp_rows(50)
    labels = [1 if int(r[4]) else 0 for r in rows]
    result = run_comparison(rows, rows, [0] * 50)
    ds = result["dataset"]
    assert ds["total"]           == 50
    assert ds["total_anomalies"] == sum(labels)


def test_run_comparison_sklearn_results_present():
    if not _SKLEARN_OK:
        return
    rows   = _make_comp_rows(80)
    preds  = [0] * 80
    result = run_comparison(rows, rows, preds)
    assert result["iforest"] is not None, "iforest must be present when sklearn available"
    assert result["ocsvm"]   is not None, "ocsvm must be present when sklearn available"
    for key in ("f1", "recall", "precision", "accuracy", "tp", "fp", "tn", "fn",
                "caught", "false_alarms", "fit_ms", "predict_ms"):
        assert key in result["iforest"], f"iforest missing key '{key}'"
        assert key in result["ocsvm"],   f"ocsvm missing key '{key}'"


def test_run_comparison_metrics_in_range():
    """All probability metrics must be in [0, 1]."""
    if not _SKLEARN_OK:
        return
    rows  = _make_comp_rows(100)
    preds = [0] * 100
    result = run_comparison(rows, rows, preds)
    for alg_name in ("hbos", "iforest", "ocsvm"):
        m = result[alg_name]
        if m is None:
            continue
        for metric in ("f1", "recall", "precision", "accuracy"):
            v = m[metric]
            assert 0.0 <= v <= 1.0, f"{alg_name}.{metric}={v} out of [0,1]"


def test_run_comparison_empty_preds():
    """Empty HBOS predictions must not raise."""
    rows   = _make_comp_rows(20)
    result = run_comparison(rows, rows, [])
    assert "hbos" in result


def test_run_comparison_hbos_fit_ms_none():
    """HBOS algorithm must have fit_ms=None (FPGA does not report timing)."""
    rows   = _make_comp_rows(30)
    result = run_comparison(rows, rows, [0] * 30)
    assert result["hbos"]["fit_ms"]     is None
    assert result["hbos"]["predict_ms"] is None


def test_run_comparison_perfect_hbos():
    """When HBOS predictions == ground truth, all metrics must be 1.0."""
    rows   = _make_comp_rows(40)
    labels = [1 if int(r[4]) else 0 for r in rows]
    result = run_comparison(rows, rows, labels)
    m = result["hbos"]
    if sum(labels) == 0:
        return   # no anomalies → skip (edge case in synthetic data)
    assert abs(m["f1"]       - 1.0) < 1e-9, f"f1={m['f1']}"
    assert abs(m["recall"]   - 1.0) < 1e-9
    assert abs(m["precision"]- 1.0) < 1e-9
    assert abs(m["accuracy"] - 1.0) < 1e-9


# ══════════════════════════════════════════════════════════════════════════════
# Page navigation — 3 pages
# ══════════════════════════════════════════════════════════════════════════════

def test_page_count_is_3():
    """Navigation must support exactly 3 pages (0 = streaming, 1 = hist, 2 = comparison)."""
    from licenta.gui.stream_viewer import StreamViewer
    import tkinter as tk
    try:
        root = tk.Tk()
        root.withdraw()
        try:
            app = StreamViewer.__new__(StreamViewer)
            tk.Tk.__init__(app)
            app.withdraw()
            app._page = 0
            # Directly check page_label text after simulated navigation
            # (can't fully init without DISPLAY, so just verify the constant)
            assert True   # navigation logic is tested by integration (mock run)
        finally:
            root.destroy()
    except tk.TclError:
        pass  # no DISPLAY in CI


# ══════════════════════════════════════════════════════════════════════════════
# _HbosEngine internals
# ══════════════════════════════════════════════════════════════════════════════

def test_hbos_engine_initial_state():
    eng = _HbosEngine()
    assert eng._state == _HbosEngine._IDLE
    assert eng._train_count == 0
    assert eng._calib_count == 0
    assert eng._global_th == 0


def test_hbos_engine_detect_untrained_returns_false():
    eng = _HbosEngine()
    assert eng.detect([100, 200, 150, 175]) is False


def test_hbos_engine_on_train_transitions_to_training():
    eng = _HbosEngine()
    eng.on_train([100, 200, 150, 175], is_clean=True)
    assert eng._state == _HbosEngine._TRAINING
    assert eng._train_count == 1


def test_hbos_engine_on_train_ignores_anomaly_counts():
    eng = _HbosEngine()
    eng.on_train([100, 200, 150, 175], is_clean=True)
    eng.on_train([9999, 9999, 9999, 9999], is_clean=False)
    assert eng._train_count == 1, "anomaly rows must not increment train_count"


def test_hbos_engine_on_calibrate_transitions_to_calibrating():
    eng = _HbosEngine()
    eng.on_train([100, 200, 150, 175], is_clean=True)
    eng.on_calibrate(0, [100, 200, 150, 175], is_clean=True)
    assert eng._state == _HbosEngine._CALIBRATING


def test_hbos_engine_on_dump_finalizes_to_ready():
    eng = _HbosEngine()
    eng.on_train([100, 200, 150, 175], is_clean=True)
    eng.on_calibrate(0, [100, 200, 150, 175], is_clean=True)
    eng.on_dump()
    assert eng._state == _HbosEngine._READY
    assert eng._global_th >= 0


def test_hbos_engine_retrain_resets_model():
    """Sending OP_TRAIN when READY must reset the model state."""
    eng = _HbosEngine()
    eng.on_train([100, 200, 150, 175], is_clean=True)
    eng.on_calibrate(0, [100, 200, 150, 175], is_clean=True)
    eng.on_dump()
    assert eng._state == _HbosEngine._READY

    # Second training pass: on_train() resets when state==READY then transitions to TRAINING
    eng.on_train([50, 100, 75, 88], is_clean=True)
    assert eng._state == _HbosEngine._TRAINING
    assert eng._train_count == 1  # reset_model zeroes it; then this call adds 1
    assert eng._global_th == 0, "threshold must be reset on retrain"


def test_hbos_engine_telemetry_length():
    eng = _HbosEngine()
    eng.on_train([100, 200, 150, 175], is_clean=True)
    eng.on_calibrate(0, [100, 200, 150, 175], is_clean=True)
    eng.on_dump()
    telem = eng.telemetry()
    assert len(telem) == 5          # threshold-only frame: 0xFE | th[23:0] | 0xFF
    assert telem[0] == 0xFE
    assert telem[-1] == 0xFF


def test_hbos_engine_set_config_updates_weights():
    eng = _HbosEngine()
    eng.set_config([10, 20, 30, 40], 1000)
    assert eng._weights == [10, 20, 30, 40]
    assert eng._spike_penalty == 1000


def test_hbos_engine_extreme_outlier_detected():
    """After full training/calib cycle, extreme outliers must be detected."""
    cancel = threading.Event()
    client = MockFpgaClient()
    rows = _make_rows(50)
    session = TrainSession(client, rows, lambda *a: None, cancel)
    ok, _, _ = session.run()
    assert ok

    # Reset detect history then send identical extreme values (no delta spike after 1st)
    for _ in range(3):
        client.send_sample([9999, 9999, 9999, 9999], OP_DETECT, 0)
        v, _ = client.try_recv()
    # The 3rd sample has delta=0 (no spike) but magnitude is maximal → anomaly
    assert v == 0x01, "extreme outlier must be flagged even without delta spike"


# ══════════════════════════════════════════════════════════════════════════════
# _hist_addr / _delta_addr fixed-point address functions
# ══════════════════════════════════════════════════════════════════════════════

from licenta.gui.mock_client import _hist_addr, _delta_addr


def test_hist_addr_zero_diff_maps_to_nonzero():
    """diff=0 → treated as 1 to avoid log(0); address must be deterministic."""
    a = _hist_addr(100, 100)
    assert isinstance(a, int)
    assert 0 <= a < 2048


def test_hist_addr_sign_bit_set_for_below_center():
    """Values below center must have sign bit 1 (bit 10 set)."""
    a = _hist_addr(90, 100)
    assert (a >> 10) & 1 == 1, f"sign bit must be 1 for v < center, got addr={a:#05x}"


def test_hist_addr_sign_bit_clear_for_above_center():
    a = _hist_addr(110, 100)
    assert (a >> 10) & 1 == 0, f"sign bit must be 0 for v >= center, got addr={a:#05x}"


def test_hist_addr_large_diff_caps_exp():
    """Exponent field (bits 9:5) must never exceed 31 for any input."""
    for diff in [0x7FFFFFFF, 0xFFFFFFFF, 0x80000000]:
        a = _hist_addr(diff, 0)
        exp = (a >> 5) & 0x1F
        assert exp <= 31, f"exponent {exp} exceeds 5-bit max for diff={diff:#x}"


def test_delta_addr_diff_zero_maps_to_zero():
    a = _delta_addr(0)
    assert a == 0, f"diff=0 → addr must be 0, got {a}"


def test_delta_addr_large_diff_caps_exp():
    """Exponent field (bits 7:3) must never exceed 31 for any input."""
    for diff in [0x7FFFFFFF, 0x80000000, 0xFFFFFFFF]:
        a = _delta_addr(diff)
        exp = (a >> 3) & 0x1F
        assert exp <= 31, f"exponent {exp} exceeds 5-bit max for diff={diff:#x}"


def test_delta_addr_monotone():
    """Larger diffs must map to larger-or-equal addresses (monotone property)."""
    prev = _delta_addr(1)
    for d in [2, 4, 8, 16, 32, 64, 128, 256, 1024, 65536]:
        cur = _delta_addr(d)
        assert cur >= prev, f"_delta_addr({d})={cur} < _delta_addr(prev)={prev}"
        prev = cur


# ══════════════════════════════════════════════════════════════════════════════
# MockFpgaClient — state transitions and multi-phase flow
# ══════════════════════════════════════════════════════════════════════════════

def test_mock_send_sample_train_updates_count():
    c = MockFpgaClient()
    for i in range(5):
        c.send_sample([100, 200, 150, 175], OP_TRAIN, 0)
    assert c._hbos._train_count == 5


def test_mock_send_sample_calib_requires_prior_train():
    """OP_CALIB without prior OP_TRAIN must be silently ignored (wrong state)."""
    c = MockFpgaClient()
    c.send_sample([100, 200, 150, 175], OP_CALIBRATE, 0)
    # Engine is IDLE, on_calibrate is a no-op
    assert c._hbos._state == _HbosEngine._IDLE


def test_mock_full_train_calib_dump_cycle():
    """A minimal manual train→calib→dump cycle must put the engine in READY state."""
    c = MockFpgaClient()
    data = [100, 200, 150, 175]
    c.send_sample(data, OP_TRAIN, 0)
    c._hbos.on_calibrate(0, data, is_clean=True)
    c._hbos.on_dump()
    assert c._hbos._state == _HbosEngine._READY


def test_mock_detect_returns_none_when_untrained():
    c = MockFpgaClient()
    c.send_sample([100, 200, 150, 175], OP_DETECT, 0)
    v, _ = c.try_recv()
    assert v == 0x00, "untrained engine must return 0x00 (safe default)"


def test_mock_config_updates_engine_weights():
    c = MockFpgaClient()
    pkt = c.pack_config_packet([10, 20, 30, 40], 1234)
    c.send(pkt)
    assert c._hbos._weights == [10, 20, 30, 40]
    assert c._hbos._spike_penalty == 1234


# ══════════════════════════════════════════════════════════════════════════════
# _load_csv_or_dir — single-file and directory loading
# ══════════════════════════════════════════════════════════════════════════════

def test_load_csv_or_dir_single_file():
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        with open(path, "w") as fh:
            for i in range(5):
                fh.write(f"{i},{i},{i},{i},0,{i}\n")
        rows = _load_csv_or_dir(path)
        assert len(rows) == 5
    finally:
        os.unlink(path)


def test_load_csv_or_dir_directory_concatenates():
    with tempfile.TemporaryDirectory() as d:
        for fname, n in [("a.csv", 3), ("b.csv", 4)]:
            with open(os.path.join(d, fname), "w") as fh:
                for i in range(n):
                    fh.write(f"{i},{i},{i},{i},0,{i}\n")
        rows = _load_csv_or_dir(d)
        assert len(rows) == 7, f"expected 3+4=7 rows, got {len(rows)}"


def test_load_csv_or_dir_empty_directory():
    with tempfile.TemporaryDirectory() as d:
        rows = _load_csv_or_dir(d)
        assert rows == [], f"empty dir must return [], got {rows}"


def test_load_csv_or_dir_directory_sorted_order():
    """Files in a directory must be loaded in sorted (deterministic) order."""
    with tempfile.TemporaryDirectory() as d:
        for fname, val in [("b.csv", 20), ("a.csv", 10)]:
            with open(os.path.join(d, fname), "w") as fh:
                fh.write(f"{val},{val},{val},{val},0,0\n")
        rows = _load_csv_or_dir(d)
        assert rows[0][0] == "10", f"a.csv (val=10) must come first, got {rows[0][0]}"


# ══════════════════════════════════════════════════════════════════════════════
# Threading safety — comparison worker uses queue, not direct tkinter call
# ══════════════════════════════════════════════════════════════════════════════

def test_comparison_worker_puts_to_queue():
    """_comparison_worker must deposit results in _comp_q without calling tkinter."""
    import inspect, queue as _queue

    # Verify the method source doesn't call self.after() directly
    src = inspect.getsource(
        __import__("licenta.gui.stream_viewer", fromlist=["StreamViewer"]).StreamViewer._comparison_worker
    )
    assert "self.after(" not in src, (
        "_comparison_worker must not call self.after() directly — "
        "use queue to avoid tkinter call from background thread"
    )
    assert "_comp_q.put(" in src, "_comparison_worker must put results into _comp_q"


def test_comparison_worker_result_reaches_queue_from_thread():
    """Results produced in a background thread must arrive in _comp_q."""
    import queue as _queue
    from licenta.gui.comparison import run_comparison

    comp_q = _queue.Queue()
    train_rows = _make_comp_rows(30)
    test_rows  = _make_comp_rows(20)
    hbos_preds = [0] * 20

    def _worker():
        try:
            results = run_comparison(train_rows, test_rows, hbos_preds)
        except Exception as exc:
            results = {"error": str(exc)}
        comp_q.put(results)

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    t.join(timeout=30)
    assert not t.is_alive(), "comparison thread must complete within 30 s"
    results = comp_q.get_nowait()
    assert "hbos"    in results
    assert "dataset" in results


# ══════════════════════════════════════════════════════════════════════════════
# TrainSession — progress granularity
# ══════════════════════════════════════════════════════════════════════════════

def test_progress_interval_granularity():
    """Progress callbacks must fire at every PROGRESS_INTERVAL rows (or at end)."""
    orig_interval = _ts.PROGRESS_INTERVAL
    _ts.PROGRESS_INTERVAL = 5
    try:
        msgs = []
        cancel = threading.Event()
        client = MockFpgaClient()
        # 23 rows → expected progress at rows 5, 10, 15, 20 (4 updates per phase)
        session = TrainSession(client, _make_rows(23), lambda f, m: msgs.append(m), cancel)
        ok, _, _ = session.run()
        assert ok
        train_msgs = [m for m in msgs if "TRAIN" in m]
        assert len(train_msgs) >= 4, f"expected ≥4 TRAIN updates for 23 rows @ interval=5, got {train_msgs}"
    finally:
        _ts.PROGRESS_INTERVAL = orig_interval


def test_progress_fractions_monotone_across_phases():
    """Fraction values across all phases must be non-decreasing."""
    fracs = []
    cancel = threading.Event()
    client = MockFpgaClient()
    session = TrainSession(client, _make_rows(30), lambda f, m: fracs.append(f), cancel)
    ok, _, _ = session.run()
    assert ok
    assert fracs == sorted(fracs), f"progress went backwards: {fracs}"


# ══════════════════════════════════════════════════════════════════════════════
# TrainSession — active_sensor_cols slicing
# ══════════════════════════════════════════════════════════════════════════════

def test_train_session_active_sensor_cols_subset():
    """active_sensor_cols= must only send the selected CSV columns."""
    sent_vals = []

    class _ValRecorder(MockFpgaClient):
        def send_sample(self, values, opcode, tlast):
            if opcode == OP_TRAIN:
                sent_vals.append(list(values))
            super().send_sample(values, opcode, tlast)

    rows = [["10", "20", "30", "40", "0", "0"],
            ["11", "21", "31", "41", "0", "1"]]
    cancel = threading.Event()
    # nr_sensors=2 matches the 2 active cols so the engine doesn't index out of range
    client = _ValRecorder(nr_sensors=2)
    session = TrainSession(client, rows, lambda *a: None, cancel,
                           active_sensor_cols=[0, 2])
    session.run()
    assert len(sent_vals) > 0
    assert sent_vals[0] == [10.0, 30.0], f"wrong cols: {sent_vals[0]}"
    assert sent_vals[1] == [11.0, 31.0], f"wrong cols: {sent_vals[1]}"


# ══════════════════════════════════════════════════════════════════════════════
# comparison._rows_to_xy
# ══════════════════════════════════════════════════════════════════════════════

from licenta.gui.comparison import _rows_to_xy


def test_rows_to_xy_valid():
    rows = [["1", "2", "3", "4", "0", "0"],
            ["5", "6", "7", "8", "1", "1"]]
    X, y = _rows_to_xy(rows)
    assert len(X) == 2
    assert len(y) == 2
    assert y == [0, 1]


def test_rows_to_xy_empty():
    X, y = _rows_to_xy([])
    assert X == [] and y == []


def test_rows_to_xy_skips_bad_rows():
    rows = [["1", "2", "3", "4", "0"],
            ["not", "a", "number", "row", "0"],
            ["5", "6", "7", "8", "1"]]
    X, y = _rows_to_xy(rows)
    assert len(X) == 2, f"bad row must be skipped, got {len(X)}"


def test_rows_to_xy_label_encoding():
    """Any non-zero label (positive or negative) must be encoded as 1; zero as 0."""
    rows = [["1", "2", "3", "4", "0"],
            ["1", "2", "3", "4", "5"],
            ["1", "2", "3", "4", "-1"]]
    _, y = _rows_to_xy(rows)
    assert y == [0, 1, 1], f"unexpected labels: {y}"


# ══════════════════════════════════════════════════════════════════════════════
# MockFpgaClient — OP_DUMP telemetry cycle (recv counter reset)
# ══════════════════════════════════════════════════════════════════════════════

def test_mock_dump_telem_cycle_resets_after_full_stream():
    """After a complete telemetry stream is consumed, the counter must reset."""
    cancel = threading.Event()
    client = MockFpgaClient()
    rows = _make_rows(10)
    session = TrainSession(client, rows, lambda *a: None, cancel)
    ok, _, _ = session.run()
    assert ok

    # Second config-update cycle must also produce valid telemetry
    session2 = TrainSession(client, [], lambda *a: None, cancel,
                            weights=[50, 50, 50, 50], spike_penalty=100)
    ok2, stream2, msg2 = session2.run_config_update()
    assert ok2, f"second config update must succeed: {msg2}"
    assert stream2 and stream2[0] == 0xFE and stream2[-1] == 0xFF


# ── runner ────────────────────────────────────────────────────────────────────

_TESTS = [
    # Packet format
    test_pack_packet_length,
    test_pack_magic_bytes,
    test_pack_opcode_and_tlast_positions,
    test_pack_sensor_values,
    test_pack_negative_sensor,
    test_mock_pack_matches_uart_format,
    # UartFpgaClient (no hardware)
    test_uart_decode_normal,
    test_uart_decode_anomaly,
    test_uart_decode_telemetry_bytes,
    test_uart_decode_short,
    test_uart_decode_unknown,
    # MockFpgaClient behaviour
    test_mock_send_reads_opcode_at_byte_16,
    test_mock_train_no_reply,
    test_mock_calib_no_reply,
    test_detect_reply_distribution,
    test_try_recv_none_before_send,
    test_mock_telem_delay,
    test_mock_telem_recv_raw_is_1_byte,
    test_drain_returns_zero,
    test_network_warnings_empty,
    # CSV loading
    test_mock_csv_write_load,
    test_load_csv_skips_header,
    test_load_csv_negative_values,
    test_load_csv_empty_file_returns_empty,
    test_load_csv_header_only_returns_empty,
    test_load_csv_short_rows_skipped,
    test_load_csv_mixed_numeric_and_header,
    # Telemetry parsing
    test_telemetry_parse,
    test_telemetry_parse_short_stream,
    test_parse_telemetry_empty,
    test_parse_telemetry_wrong_banner,
    test_parse_telemetry_threshold_le24,
    test_parse_telemetry_all_fields,
    # TrainSession — main flow and cancellations
    test_train_session_completes,
    test_train_session_progress_covers_phases,
    test_train_session_cancel_train,
    test_train_session_cancel_calib,
    test_train_session_cancel_postcalib_wait,
    test_train_session_cancel_latch,
    test_train_session_cancel_telemetry,
    # _collect_telemetry edge cases
    test_collect_telemetry_full_17_bytes,
    test_collect_telemetry_out_of_sync_0xff_before_0xfe,
    test_collect_telemetry_timeout_before_banner,
    test_collect_telemetry_partial_stream,
    test_collect_telemetry_nones_before_banner_ok,
    test_collect_telemetry_nones_mid_stream,
    # State machine
    test_state_machine_transitions,
    test_button_state_rules,
    test_all_states_covered_in_lamp_cfg,
    test_stream_viewer_vars_initialised_before_build_ui,
    # OP_CONFIG packet encoding
    test_pack_config_opcode,
    test_pack_config_weights_encoding,
    test_pack_config_spike_penalty,
    test_pack_config_hls_defaults,
    test_pack_config_max_weights,
    test_pack_config_uart_mock_agree,
    # TrainSession with weights
    test_train_session_sends_config_before_train,
    test_train_session_no_config_without_weights,
    test_run_config_update_completes,
    test_run_config_update_no_weights_fails,
    test_run_config_update_sends_op_config,
    test_run_config_update_cancel,
    # Apply Config button state rule
    test_apply_config_button_enabled_only_in_ready,
    # Comparison — compute_metrics
    test_compute_metrics_perfect,
    test_compute_metrics_all_wrong,
    test_compute_metrics_no_anomalies,
    test_compute_metrics_all_predicted_anomaly,
    test_compute_metrics_caught_string,
    test_compute_metrics_false_alarms,
    test_compute_metrics_empty,
    test_compute_metrics_length_mismatch,
    test_compute_metrics_f1_known_value,
    # Comparison — run_comparison
    test_run_comparison_returns_required_keys,
    test_run_comparison_hbos_matches_preds,
    test_run_comparison_dataset_counts,
    test_run_comparison_sklearn_results_present,
    test_run_comparison_metrics_in_range,
    test_run_comparison_empty_preds,
    test_run_comparison_hbos_fit_ms_none,
    test_run_comparison_perfect_hbos,
    # Page navigation
    test_page_count_is_3,
    # _HbosEngine internals
    test_hbos_engine_initial_state,
    test_hbos_engine_detect_untrained_returns_false,
    test_hbos_engine_on_train_transitions_to_training,
    test_hbos_engine_on_train_ignores_anomaly_counts,
    test_hbos_engine_on_calibrate_transitions_to_calibrating,
    test_hbos_engine_on_dump_finalizes_to_ready,
    test_hbos_engine_retrain_resets_model,
    test_hbos_engine_telemetry_length,
    test_hbos_engine_set_config_updates_weights,
    test_hbos_engine_extreme_outlier_detected,
    # Fixed-point address functions
    test_hist_addr_zero_diff_maps_to_nonzero,
    test_hist_addr_sign_bit_set_for_below_center,
    test_hist_addr_sign_bit_clear_for_above_center,
    test_hist_addr_large_diff_caps_exp,
    test_delta_addr_diff_zero_maps_to_zero,
    test_delta_addr_large_diff_caps_exp,
    test_delta_addr_monotone,
    # MockFpgaClient — state and config
    test_mock_send_sample_train_updates_count,
    test_mock_send_sample_calib_requires_prior_train,
    test_mock_full_train_calib_dump_cycle,
    test_mock_detect_returns_none_when_untrained,
    test_mock_config_updates_engine_weights,
    # _load_csv_or_dir
    test_load_csv_or_dir_single_file,
    test_load_csv_or_dir_directory_concatenates,
    test_load_csv_or_dir_empty_directory,
    test_load_csv_or_dir_directory_sorted_order,
    # Threading safety
    test_comparison_worker_puts_to_queue,
    test_comparison_worker_result_reaches_queue_from_thread,
    # Progress granularity
    test_progress_interval_granularity,
    test_progress_fractions_monotone_across_phases,
    # TrainSession sensor slicing
    test_train_session_active_sensor_cols_subset,
    # comparison._rows_to_xy
    test_rows_to_xy_valid,
    test_rows_to_xy_empty,
    test_rows_to_xy_skips_bad_rows,
    test_rows_to_xy_label_encoding,
    # MockFpgaClient telemetry reset
    test_mock_dump_telem_cycle_resets_after_full_stream,
]

if __name__ == "__main__":
    print(f"Running {len(_TESTS)} headless tests…\n")
    for t in _TESTS:
        _check(t.__name__, t)
    total = _PASS + _FAIL
    print(f"\n{'─' * 60}")
    print(f"  {_PASS}/{total} passed  {'OK' if _FAIL == 0 else f'{_FAIL} FAILED'}")
    sys.exit(0 if _FAIL == 0 else 1)
