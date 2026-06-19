"""
MockFpgaClient — offline substitute for FpgaClient.

Runs the same fixed-point HBOS algorithm as the FPGA on the packets it
receives, so the telemetry (global_threshold, delta_th, counts) and the
detection responses are computed from the real training data rather than
being hardcoded.

Algorithm mirrors optimize_occupancy_hbos.py:
  - single-value rolling history (not 5-tap)
  - no phase gating — all clean samples update the histograms
  - calibration reuses addresses computed during training
  - global_threshold = score_bin_at_99.8th_pct << 4
"""

import random
import struct
import threading
import time

from .uart_client import OP_TRAIN, OP_CALIBRATE, OP_DETECT, OP_DUMP, OP_CONFIG, OP_RESET

NR_BINS       = 2048
NR_DELTA_BINS = 256
NR_SENSORS    = 4

# ── fixed-point helpers (hbos_top.cpp / address_engine.cpp) ──────────────────

_LOG2_LUT = [0, 22, 43, 63, 82, 101, 119, 136, 153, 170, 186, 202, 217, 232, 246, 260]

def _ilog2(x: int) -> int:
    x = int(x)
    if x <= 0:
        return 0
    msb = x.bit_length() - 1
    frac = (x >> (msb - 4)) & 0xF if msb >= 4 else (x << (4 - msb)) & 0xF
    return (msb << 8) + _LOG2_LUT[frac]

def _hist_addr(v: int, center: int) -> int:
    v, center = int(v), int(center)
    diff = abs(v - center) or 1
    sign = 0 if v >= center else 1
    msb  = diff.bit_length() - 1
    exp  = min(msb, 31)
    mant = (diff >> (msb - 5)) & 31 if msb >= 5 else (diff << (5 - msb)) & 31
    return (sign << 10) | (exp << 5) | mant

def _delta_addr(diff: int) -> int:
    diff = int(diff) or 1
    msb  = diff.bit_length() - 1
    exp  = min(msb, 31)
    mant = (diff >> (msb - 3)) & 7 if msb >= 3 else (diff << (3 - msb)) & 7
    return (exp << 3) | mant


# ── HBOS engine ───────────────────────────────────────────────────────────────

class _HbosEngine:
    """Software replica of hbos_top + address_engine, statefully fed by packets."""

    _IDLE       = 0
    _TRAINING   = 1
    _CALIBRATING= 2
    _READY      = 3

    def __init__(self, nr_sensors: int = NR_SENSORS):
        self._nr_sensors    = nr_sensors
        self._weights       = [64] * nr_sensors
        self._spike_penalty = 5632
        self._active_mask   = (1 << nr_sensors) - 1
        self._state         = self._IDLE
        self._reset_model()

    def set_config(self, weights: list, spike_penalty: int):
        # Extend or truncate weights to match nr_sensors; pad with 64 if short
        padded = list(weights) + [64] * self._nr_sensors
        self._weights       = [int(padded[i]) & 0xFF for i in range(self._nr_sensors)]
        self._spike_penalty = int(spike_penalty) & 0xFFFF

    def set_mask(self, mask: int):
        self._active_mask = int(mask) & ((1 << self._nr_sensors) - 1)

    def _reset_model(self):
        ns = self._nr_sensors
        self._center      = [0] * ns
        self._train_hist  = [[0] * NR_BINS       for _ in range(ns)]
        self._d_hist      = [[0] * NR_DELTA_BINS  for _ in range(ns)]

        # Per-sample cached addresses from the training pass (reused in calib)
        self._train_addrs:  list = []
        self._train_daddrs: list = []
        self._train_clean:  list = []

        self._score_lut   = [[0] * NR_BINS for _ in range(ns)]
        self._delta_th    = [255] * ns

        self._train_count = 0
        self._calib_count = 0
        self._calib_idx   = 0
        self._global_th   = 0

        self._train_history = None
        self._det_history   = None

    def reset(self):
        """Full flush — mirror the FPGA OP_RESET: clear model and go idle."""
        self._reset_model()
        self._state = self._IDLE

    # ── packet processing ─────────────────────────────────────────────────────

    def on_train(self, data: list, is_clean: bool):
        if self._state == self._READY:
            # New training run — reset model state but keep config
            self._reset_model()
            self._state = self._IDLE  # _reset_model doesn't clear state; do it here
        if self._state == self._IDLE:
            self._center        = list(data)
            self._train_history = list(data)
            self._state         = self._TRAINING

        ns = self._nr_sensors
        h = self._train_history
        addrs  = [_hist_addr(data[s], self._center[s]) for s in range(ns)]
        daddrs = [_delta_addr(abs(int(data[s]) - int(h[s]))) for s in range(ns)]

        self._train_addrs.append(addrs)
        self._train_daddrs.append(daddrs)
        self._train_clean.append(is_clean)

        if is_clean:
            self._train_count += 1
            for s in range(ns):
                if (self._active_mask >> s) & 1:
                    self._train_hist[s][addrs[s]] += 1
                    self._d_hist[s][daddrs[s]]    += 1

        for s in range(ns):
            self._train_history[s] = data[s]

    def _finalize_train(self):
        n = self._train_count
        log2_denom = _ilog2(n + NR_BINS)
        for s in range(self._nr_sensors):
            for b in range(NR_BINS):
                self._score_lut[s][b] = max(0, log2_denom - _ilog2(self._train_hist[s][b] + 1))
            tgt = n - (n >> 10)
            cum = 0
            for d in range(NR_DELTA_BINS):
                cum += self._d_hist[s][d]
                if cum >= tgt:
                    self._delta_th[s] = d
                    break

    def on_calibrate(self, idx: int, _data, is_clean: bool):
        """Called for each OP_CALIBRATE packet.  Uses cached training addresses."""
        if self._state == self._TRAINING:
            self._finalize_train()
            self._score_hist = [0] * 2048
            self._state      = self._CALIBRATING

        if self._state != self._CALIBRATING:
            return

        if idx >= len(self._train_addrs):
            return   # more calib packets than train packets (pump traffic) — ignore

        is_clean = self._train_clean[idx]
        score    = self._score_sample(self._train_addrs[idx], self._train_daddrs[idx])

        if is_clean:
            self._calib_count += 1
            bucket = score >> 4
            if bucket < 2048:
                self._score_hist[bucket] += 1

    def _finalize_calib(self):
        n   = self._calib_count
        tgt = n - (n >> 9)
        cum = 0
        th  = 32767
        for j in range(2048):
            cum += self._score_hist[j]
            if cum >= tgt:
                th = j << 4
                break
        self._global_th = th
        self._state     = self._READY

    def on_dump(self):
        """First OP_DUMP after calibration finalizes the threshold."""
        if self._state == self._CALIBRATING:
            self._finalize_calib()

    def detect(self, data: list) -> bool:
        if self._state != self._READY:
            return False
        ns = self._nr_sensors
        if self._det_history is None:
            self._det_history = list(data)
        addrs  = [_hist_addr(data[s], self._center[s]) for s in range(ns)]
        daddrs = [_delta_addr(abs(int(data[s]) - int(self._det_history[s]))) for s in range(ns)]
        for s in range(ns):
            self._det_history[s] = data[s]
        return self._score_sample(addrs, daddrs) >= self._global_th

    def _score_sample(self, addrs: list, daddrs: list) -> int:
        total = 0
        for s in range(self._nr_sensors):
            if not ((self._active_mask >> s) & 1):
                continue
            base = self._score_lut[s][addrs[s]]
            if daddrs[s] > self._delta_th[s]:
                base += self._spike_penalty
            total += (base * self._weights[s]) >> 8
        return total

    def telemetry(self) -> list:
        # Threshold-only readback: 0xFE | global_threshold[23:0] LE | 0xFF.
        # Mirrors the FPGA after the telemetry FSM was reduced to the threshold.
        th = self._global_th
        return [
            0xFE,
            th & 0xFF, (th >> 8) & 0xFF, (th >> 16) & 0xFF,
            0xFF,
        ]


# ── MockFpgaClient ────────────────────────────────────────────────────────────

class MockFpgaClient:
    _TELEM_DELAY = 2

    def __init__(self, nr_sensors: int = 4, anomaly_rate: float = 0.12,
                 seed: int = 42, simulated_rx_count: int = 0):
        self._lock  = threading.Lock()
        self._hbos  = _HbosEngine(nr_sensors=nr_sensors)

        self._last_opcode    = -1
        self._recv_dump_n    = 0
        self._detect_pending = False
        self._calib_idx      = 0    # packet counter for OP_CALIBRATE
        self._telem: list    = []   # built after finalize_calib

    # ── packet building ───────────────────────────────────────────────────────

    def pack_config_packet(self, weights: list, spike_penalty: int,
                           sensor_mask=None) -> bytes:
        w = [int(weights[i]) & 0xFF for i in range(4)]
        w_unsigned = w[0] | (w[1] << 8) | (w[2] << 16) | (w[3] << 24)
        s0 = struct.unpack('<i', struct.pack('<I', w_unsigned))[0]
        if sensor_mask is None:
            mask_bits = 0xF
        elif isinstance(sensor_mask, (list, tuple)):
            mask_bits = sum((1 << i) for i, v in enumerate(sensor_mask) if v) & 0xF
        else:
            mask_bits = int(sensor_mask) & 0xF
        return self.pack_packet(s0, int(spike_penalty) & 0xFFFF, mask_bits, 0, OP_CONFIG, 0)

    def pack_packet(self, s0, s1, s2, s3, opcode: int, tlast: int) -> bytes:
        return (
            struct.pack(
                "<iiiiBB",
                int(float(s0)), int(float(s1)), int(float(s2)), int(float(s3)),
                opcode, tlast,
            )
            + bytes((0xA5, 0x5A))
        )

    # ── send ──────────────────────────────────────────────────────────────────

    def send(self, payload: bytes) -> None:
        if len(payload) < 18:
            return
        opcode = payload[16]
        tlast  = payload[17]

        if opcode == OP_RESET:
            self._hbos.reset()
            self._calib_idx   = 0
            self._last_opcode = opcode
            return

        if opcode == OP_CONFIG:
            w_raw   = struct.unpack('<I', payload[0:4])[0]
            spike   = struct.unpack('<i', payload[4:8])[0] & 0xFFFF
            weights = [(w_raw >> (8 * i)) & 0xFF for i in range(4)]
            self._hbos.set_config(weights, spike)
            # All sensor slots active in mock — selection handled by _active_indices in GUI
            self._hbos.set_mask((1 << self._hbos._nr_sensors) - 1)
            self._last_opcode = opcode
            return

        if opcode == OP_TRAIN:
            data = [struct.unpack('<i', payload[4*s:4*s+4])[0] for s in range(4)]
            self._hbos.on_train(data, is_clean=(tlast == 0))
            self._last_opcode = opcode
            return

        if opcode == OP_CALIBRATE:
            data = [struct.unpack('<i', payload[4*s:4*s+4])[0] for s in range(4)]
            self._hbos.on_calibrate(self._calib_idx, data, is_clean=(tlast == 0))
            self._calib_idx += 1
            self._last_opcode = opcode
            return

        if opcode == OP_DUMP:
            if self._last_opcode not in (OP_DUMP,):
                # First OP_DUMP after calibration — finalize model
                self._hbos.on_dump()
                self._telem         = self._hbos.telemetry()
                self._recv_dump_n   = 0
                self._calib_idx     = 0
            elif self._recv_dump_n >= self._TELEM_DELAY + len(self._telem):
                # Subsequent telemetry poll cycle (e.g. after run_config_update)
                self._telem       = self._hbos.telemetry()
                self._recv_dump_n = 0
            self._last_opcode = opcode
            return

        if opcode == OP_DETECT:
            self._detect_pending = True
            self._detect_data = [struct.unpack('<i', payload[4*s:4*s+4])[0] for s in range(4)]
            self._last_opcode = opcode

    def send_reset(self) -> None:
        """Flush the mock engine — mirrors UartFpgaClient.send_reset / OP_RESET."""
        self._hbos.reset()
        self._calib_idx   = 0
        self._last_opcode = OP_RESET

    def send_sample(self, values: list, opcode: int, tlast: int) -> None:
        """Send N sensor values directly to the HBOS engine, bypassing serialisation."""
        data = [int(float(v)) for v in values]
        is_clean = (tlast == 0)
        if opcode == OP_RESET:
            self._hbos.reset()
            self._calib_idx   = 0
            self._last_opcode = opcode
        elif opcode == OP_TRAIN:
            self._hbos.on_train(data, is_clean)
            self._last_opcode = opcode
        elif opcode == OP_CALIBRATE:
            self._hbos.on_calibrate(self._calib_idx, data, is_clean)
            self._calib_idx += 1
            self._last_opcode = opcode
        elif opcode == OP_DETECT:
            self._detect_pending = True
            self._detect_data = data
            self._last_opcode = opcode

    # ── recv ──────────────────────────────────────────────────────────────────

    def recv(self, timeout: float = 1.0) -> tuple:
        time.sleep(0.004)
        if self._last_opcode == OP_DUMP:
            idx = self._recv_dump_n - self._TELEM_DELAY
            self._recv_dump_n += 1
            if idx < 0:
                return None, None
            if idx < len(self._telem):
                b = self._telem[idx]
                return b, bytes([b])
        return None, None

    # ── try_recv ──────────────────────────────────────────────────────────────

    def try_recv(self) -> tuple:
        if self._last_opcode == OP_DETECT and self._detect_pending:
            self._detect_pending = False
            data = getattr(self, '_detect_data', [0, 0, 0, 0])
            r = 0x01 if self._hbos.detect(data) else 0x00
            return r, bytes([r])
        return None, None

    # ── housekeeping ──────────────────────────────────────────────────────────

    def drain(self, timeout: float = 0.5) -> int:
        return 0

    def close(self) -> None:
        pass

    @staticmethod
    def network_warnings(iface: str = "") -> list:
        return []


# ── synthetic CSV generator (kept for tests that still use it) ────────────────

def make_mock_csv(path: str, n: int = 60, seed: int = 0) -> None:
    """Write a small synthetic CSV (header + n rows) to *path*."""
    rng = random.Random(seed)
    with open(path, "w") as fh:
        fh.write("s0,s1,s2,s3,label,seq\n")
        for i in range(n):
            base = [rng.randint(80, 200) for _ in range(4)]
            if i % 17 == 0 and i > 0:
                base = [v + rng.randint(60, 120) for v in base]
            label = 1 if (i % 17 == 0 and i > 0) else 0
            fh.write(f"{base[0]},{base[1]},{base[2]},{base[3]},{label},{i}\n")
