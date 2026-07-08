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

from .uart_client import (OP_TRAIN, OP_CALIBRATE, OP_DETECT, OP_DUMP, OP_CONFIG,
                          OP_RESET, OP_LOAD_TRAIN, OP_LOAD_TEST, DEFAULT_CALIB_SHIFT)

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
        self._calib_shift   = DEFAULT_CALIB_SHIFT   # global-threshold percentile shift
        self._active_mask   = (1 << nr_sensors) - 1
        self._state         = self._IDLE
        self._reset_model()

    def set_config(self, weights: list, spike_penalty: int,
                   calib_shift: int = DEFAULT_CALIB_SHIFT):
        # Extend or truncate weights to match nr_sensors; pad with 64 if short
        padded = list(weights) + [64] * self._nr_sensors
        self._weights       = [int(padded[i]) & 0xFF for i in range(self._nr_sensors)]
        self._spike_penalty = int(spike_penalty) & 0xFFFF
        # 0 = legacy host that sends no calib word -> keep the default 99.8th pct
        self._calib_shift   = int(calib_shift) & 0x1F or DEFAULT_CALIB_SHIFT

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
        tgt = n - (n >> self._calib_shift)
        cum = 0
        th  = 32767
        for j in range(2048):
            cum += self._score_hist[j]
            if cum >= tgt:
                th = j << 4
                break
        self._global_th = th
        self._state     = self._READY

    def refinalize(self):
        """Recompute the global threshold from the EXISTING score histogram with
        the current calib_shift. Used when config (percentile/weights) changes
        after calibration — mirrors the FPGA's OP_CONFIG -> OP_DUMP re-finalize
        without re-accumulating samples."""
        if self._state == self._READY and self._calib_count > 0:
            self._finalize_calib()

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

        # Emulated DDR2 staging: train/test regions filled by OP_LOAD_*, then
        # replayed on the OP_TRAIN/OP_CALIB/OP_DETECT triggers — mirrors dataset_dma.
        self._ddr_train: list = []
        self._ddr_test:  list = []
        self._verdict_buf = bytearray()   # 4-byte [seq:3][verdict:1] detect replies

    # ── packet building ───────────────────────────────────────────────────────

    def pack_frame(self, values: list, active_count: int, opcode: int, tlast: int,
                   seq: int = 0) -> bytes:
        vals = list(values)
        s = int(seq) & 0xFFFFFF
        out = bytes((len(vals) & 0xFF, int(active_count) & 0xFF, opcode & 0x0F, tlast & 1,
                     s & 0xFF, (s >> 8) & 0xFF, (s >> 16) & 0xFF))
        for v in vals:
            out += struct.pack("<i", int(float(v)))
        return out + bytes((0xA5, 0x5A))

    def pack_config_packet(self, weights: list, spike_penalty: int,
                           active_count: int = None, delta_stride: int = 1,
                           calib_shift: int = DEFAULT_CALIB_SHIFT) -> bytes:
        w = [int(x) & 0xFF for x in list(weights)[:16]]
        w += [0] * (16 - len(w))
        if active_count is None:
            active_count = len(list(weights))
        words = []
        for k in range(4):
            val = w[4*k] | (w[4*k+1] << 8) | (w[4*k+2] << 16) | (w[4*k+3] << 24)
            words.append(struct.unpack('<i', struct.pack('<I', val))[0])
        words.append(int(spike_penalty) & 0xFFFF)
        words.append(int(delta_stride) & 0xF)
        words.append(int(calib_shift) & 0x1F)
        return self.pack_frame(words, active_count, OP_CONFIG, 0)

    # ── send ──────────────────────────────────────────────────────────────────

    def send(self, payload: bytes) -> None:
        # New count-prefixed frame: [n_words][active_count][opcode][tlast][data..][magic]
        if len(payload) < 9:
            return
        n_words = payload[0]
        opcode  = payload[2] & 0x0F
        tlast   = payload[3] & 1
        seq     = payload[4] | (payload[5] << 8) | (payload[6] << 16)
        words = [struct.unpack('<i', payload[7 + 4*i:11 + 4*i])[0] for i in range(n_words)]

        if opcode == OP_RESET:
            self._hbos.reset()
            self._ddr_train = []
            self._ddr_test  = []
            self._verdict_buf = bytearray()
            self._calib_idx   = 0
            self._last_opcode = opcode
            return

        if opcode == OP_CONFIG:
            # words = [w0..3 packed, spike]; 16 weights, first nr_sensors are live.
            ns = self._hbos._nr_sensors
            w16 = []
            for k in range(4):
                wv = words[k] & 0xFFFFFFFF
                w16 += [(wv >> (8*b)) & 0xFF for b in range(4)]
            spike = words[4] & 0xFFFF if len(words) > 4 else 0
            calib_shift = words[6] & 0x1F if len(words) > 6 else 0
            self._hbos.set_config(w16[:ns], spike, calib_shift)
            self._hbos.set_mask((1 << ns) - 1)
            # If already calibrated, re-finalize the threshold at the new config
            # (mirrors the FPGA OP_CONFIG -> OP_DUMP re-finalize used by Apply Config).
            self._hbos.refinalize()
            self._last_opcode = opcode
            return

        # ── DDR2 staging: store one sample into the train/test region ──────────
        if opcode == OP_LOAD_TRAIN:
            self._ddr_train.append(list(words))
            self._last_opcode = opcode
            return
        if opcode == OP_LOAD_TEST:
            self._ddr_test.append(list(words))
            self._last_opcode = opcode
            return

        # ── replay triggers (data-less): rebuild/score from the DDR2 region ───
        if opcode == OP_TRAIN:
            # Rebuild histograms from the staged train region (replay, all clean).
            self._hbos.reset()
            self._calib_idx = 0
            for sample in self._ddr_train:
                self._hbos.on_train(sample, is_clean=True)
            self._last_opcode = opcode
            return

        if opcode == OP_CALIBRATE:
            # Score the staged train region to build the threshold distribution.
            self._calib_idx = 0
            for idx in range(len(self._ddr_train)):
                self._hbos.on_calibrate(idx, self._ddr_train[idx], is_clean=True)
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
            # Replay the staged test region, stamping seq = sample index, and
            # buffer the 4-byte [seq:3][verdict:1] replies for read_available().
            self._hbos._det_history = None
            buf = bytearray()
            for i, sample in enumerate(self._ddr_test):
                v = 0x01 if self._hbos.detect(sample) else 0x00
                buf += bytes((i & 0xFF, (i >> 8) & 0xFF, (i >> 16) & 0xFF, v))
            with self._lock:
                self._verdict_buf = buf
            self._last_opcode = opcode

    def send_reset(self) -> None:
        """Flush the mock engine + DDR2 regions — mirrors UartFpgaClient.send_reset."""
        self._hbos.reset()
        self._ddr_train = []
        self._ddr_test  = []
        with self._lock:
            self._verdict_buf = bytearray()
        self._calib_idx   = 0
        self._last_opcode = OP_RESET

    def read_available(self) -> bytes:
        """Return all buffered detect-verdict bytes (4 per sample) and clear them.
        Mirrors UartFpgaClient.read_available so the GUI's seq-based RX consumer
        works identically against the mock."""
        with self._lock:
            if self._verdict_buf:
                b = bytes(self._verdict_buf)
                self._verdict_buf = bytearray()
                return b
        return b""

    def send_sample(self, values: list, opcode: int, tlast: int, seq: int = 0) -> None:
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
            self._detect_seq  = seq
            self._last_opcode = opcode

    def recv_verdict(self, timeout: float = 1.0):
        """Return (seq, verdict) for a pending detect sample, else None.
        Mirrors the FPGA's 4-byte [seq:3][verdict:1] detect response."""
        if self._last_opcode == OP_DETECT and self._detect_pending:
            self._detect_pending = False
            data = getattr(self, '_detect_data', [0])
            seq  = getattr(self, '_detect_seq', 0)
            v = 0x01 if self._hbos.detect(data) else 0x00
            return seq, v
        return None

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
