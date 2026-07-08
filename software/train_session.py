"""
Full FPGA training sequence: TRAIN → CALIB → post-calib drain → config latch → telemetry.

Designed to run in a background thread.  All blocking I/O is here; the GUI
stays responsive by polling a progress queue.

Sequence
--------
1. Send all train rows as OP_TRAIN
2. Send same rows as OP_CALIBRATE
3. Wait POST_CALIB_WAIT seconds + drain stray UDP
4. Config latch: 1× OP_DUMP + DUMP_CALIB_LATCH× OP_CALIB + 2s settle
5. Telemetry poll: OP_DUMP until 0xFE…0xFF stream (17 bytes) or timeout
"""

import time
import threading
from typing import Callable

from .udp_client import (UdpFpgaClient as FpgaClient, OP_TRAIN, OP_CALIBRATE,
                         OP_DUMP, OP_CONFIG, OP_RESET, DEFAULT_CALIB_SHIFT)

TRAIN_PACE       = 0.0002   # seconds between send()s during train/calib
POST_CALIB_WAIT  = 5.0      # seconds to wait after last CALIB before latch
DUMP_CALIB_LATCH = 32       # OP_CALIB packets after the first OP_DUMP
LATCH_SETTLE     = 2.0      # seconds after latch pump before telemetry poll
TELEMETRY_POLLS  = 60       # max OP_DUMP polls for 0xFE…0xFF
POLL_GAP         = 0.08     # seconds between telemetry polls
PROGRESS_INTERVAL = 500     # rows between GUI progress updates


class TrainSession:
    """
    Run the full FPGA training/calibration/config-latch sequence.

    Parameters
    ----------
    client       FpgaClient bound to the FPGA UDP port
    rows         CSV rows [[s0, s1, s2, s3, label, ...], ...]
    progress_cb  Called with (fraction 0..1, message) from the worker thread
    cancel_event Set this to abort; checked at packet boundaries
    """

    def __init__(
        self,
        client: FpgaClient,
        rows: list,
        progress_cb: Callable,
        cancel_event: threading.Event,
        weights: "list | None" = None,
        spike_penalty: "int | None" = None,
        sensor_mask: "list | None" = None,
        active_sensor_cols: "list | None" = None,
        delta_stride: int = 1,
        calib_shift: int = DEFAULT_CALIB_SHIFT,
    ):
        self._c = client
        self._rows = rows
        self._cb = progress_cb
        self._cancel = cancel_event
        self._weights = weights
        self._spike_penalty = spike_penalty if spike_penalty is not None else 5632
        self._sensor_mask = sensor_mask        # list of bools; None = all enabled
        self._active_cols = active_sensor_cols  # CSV col index per active channel (len=N)
        self._delta_stride = delta_stride       # detect-time delta stride (1..8)
        self._calib_shift = calib_shift          # global-threshold percentile shift

    def _ac(self) -> int:
        """Active sensor count (= number of live channels)."""
        if self._active_cols:
            return len(self._active_cols)
        return len(self._weights) if self._weights else 4

    def _active_weights(self) -> list:
        """Weights repacked to channel order 0..N-1, matching the packed data."""
        w = self._weights or []
        cols = self._active_cols if self._active_cols else list(range(len(w)))
        return [int(w[c]) if c < len(w) else 0 for c in cols]

    # ── public ───────────────────────────────────────────────────────────────

    def run(self) -> tuple:
        """
        Execute the full sequence synchronously.

        Returns
        -------
        (ok: bool, telemetry: list[int] | None, message: str)
            ok=True means the FPGA is in the ready state.
        """
        n = len(self._rows)

        # ── 0a. RESET ────────────────────────────────────────────────────────
        # Flush all FPGA engine state so this run never accumulates onto the
        # previous training's histograms. Sent before CONFIG/TRAIN.
        self._cb(0.00, "Flushing FPGA state (OP_RESET)…")
        self._c.send_reset()
        time.sleep(0.05)
        self._c.drain(0.2)

        # ── 0. CONFIG (optional) ─────────────────────────────────────────────
        if self._weights is not None:
            self._cb(0.00, "Sending OP_CONFIG…")
            self._c.send(self._c.pack_config_packet(
                self._active_weights(), self._spike_penalty, self._ac(),
                delta_stride=self._delta_stride, calib_shift=self._calib_shift))
            time.sleep(0.05)

        # ── 1. TRAIN ────────────────────────────────────────────────────────
        self._cb(0.00, f"TRAIN phase — sending {n} packets…")
        if not self._send_phase(OP_TRAIN, 0.00, 0.33, "TRAIN"):
            return False, None, "Cancelled during TRAIN"
        self._c.drain(0.3)

        # ── 2. CALIB ────────────────────────────────────────────────────────
        self._cb(0.33, f"CALIB phase — sending {n} packets…")
        if not self._send_phase(OP_CALIBRATE, 0.33, 0.33, "CALIB"):
            return False, None, "Cancelled during CALIB"

        # ── 3. Post-calib drain ─────────────────────────────────────────────
        self._cb(0.66, f"Post-calib drain ({POST_CALIB_WAIT:.0f}s)…")
        if not self._wait(POST_CALIB_WAIT):
            return False, None, "Cancelled during post-calib wait"
        self._c.drain(2.0)

        # ── 4. Config latch ──────────────────────────────────────────────────
        self._cb(0.78, f"Config latch: 1× DUMP + {DUMP_CALIB_LATCH}× CALIB pump…")
        self._c.send(self._c.pack_frame([], self._ac(), OP_DUMP, 0))
        time.sleep(0.1)
        for _ in range(DUMP_CALIB_LATCH):
            if self._cancel.is_set():
                return False, None, "Cancelled during config latch"
            self._c.send(self._c.pack_frame([], self._ac(), OP_CALIBRATE, 0))
            time.sleep(0.02)
        self._cb(0.84, f"Latch settle ({LATCH_SETTLE:.0f}s)…")
        if not self._wait(LATCH_SETTLE):
            return False, None, "Cancelled during latch settle"
        self._c.drain(0.3)

        # ── 5. Telemetry poll ────────────────────────────────────────────────
        self._cb(0.90, f"Telemetry poll (up to {TELEMETRY_POLLS} OP_DUMP polls)…")
        ok, stream = self._collect_telemetry()
        if not ok:
            partial = " ".join(f"{b:02x}" for b in stream) if stream else "(empty)"
            return False, stream, f"Config latch failed — telemetry incomplete: {partial}"

        self._cb(1.00, "Ready!")
        return True, stream, "OK"

    # ── internals ────────────────────────────────────────────────────────────

    def _send_phase(
        self,
        opcode: int,
        frac_start: float,
        frac_span: float,
        label: str,
    ) -> bool:
        n    = len(self._rows)
        c    = self._c
        cols = self._active_cols or [0, 1, 2, 3]
        for i, row in enumerate(self._rows):
            if self._cancel.is_set():
                return False
            tlast = 1 if int(row[-1]) != 0 else 0
            vals  = [float(row[cols[j]]) if cols[j] < len(row) else 0.0
                     for j in range(len(cols))]
            c.send_sample(vals, opcode, tlast)
            time.sleep(TRAIN_PACE)
            if i % PROGRESS_INTERVAL == 0 and i > 0:
                frac = frac_start + frac_span * (i / n)
                self._cb(frac, f"{label} {i}/{n}")
        return True

    def _collect_telemetry(self) -> tuple:
        c = self._c
        stream: list = []
        in_telemetry = False
        for poll in range(TELEMETRY_POLLS):
            if self._cancel.is_set():
                return False, stream
            c.send(c.pack_frame([], self._ac(), OP_DUMP, 0))
            time.sleep(POLL_GAP)
            byte_val, _ = c.recv(timeout=0.35)
            if byte_val is None:
                continue
            if not in_telemetry:
                if byte_val == 0xFF:
                    continue               # normal dump_ack, telemetry not ready yet
                if byte_val == 0xFE:
                    in_telemetry = True
                    stream = [0xFE]
                continue
            stream.append(byte_val)
            if byte_val == 0xFF:
                return True, stream
        return False, stream

    def _wait(self, duration: float) -> bool:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self._cancel.is_set():
                return False
            time.sleep(0.05)
        return True

    # ── post-training config update ───────────────────────────────────────────

    def run_config_update(self) -> tuple:
        """Send OP_CONFIG then redo the config latch + telemetry poll.

        Used when the user changes sensor weights after training without
        retraining. hbos_top re-computes the threshold with the new weights
        and propagates it to detection_engine via the DUMP→CALIB pump.

        Returns (ok, telemetry_stream, message).
        """
        if self._weights is None:
            return False, None, "No weights specified"

        c = self._c

        self._cb(0.05, "Sending OP_CONFIG…")
        c.send(c.pack_config_packet(self._active_weights(), self._spike_penalty, self._ac(),
                                    delta_stride=self._delta_stride, calib_shift=self._calib_shift))
        time.sleep(0.05)

        # A single OP_DUMP re-finalizes the threshold over the EXISTING (clean)
        # score histogram with the just-applied config, and writes it to
        # detection_engine via the config stream. We deliberately do NOT pump
        # OP_CALIBRATE here: in the DDR2 design each OP_CALIB replays the whole
        # train region back into score_hist (which is never re-zeroed after the
        # first calibration), so repeated config-updates pile the distribution
        # up and collapse the threshold until OP_RESET. The percentile/weights
        # only change the cutoff, not the stored histogram, so re-finalize alone
        # is both correct and idempotent.
        self._cb(0.40, "Re-finalizing threshold (OP_DUMP)…")
        c.send(c.pack_frame([], self._ac(), OP_DUMP, 0))
        time.sleep(0.1)

        self._cb(0.70, f"Settle ({LATCH_SETTLE:.0f}s)…")
        if not self._wait(LATCH_SETTLE):
            return False, None, "Cancelled during settle"
        c.drain(0.3)

        self._cb(0.90, "Telemetry poll…")
        ok, stream = self._collect_telemetry()
        if not ok:
            partial = " ".join(f"{b:02x}" for b in stream) if stream else "(empty)"
            return False, stream, f"Config update failed — telemetry incomplete: {partial}"

        self._cb(1.00, "Config applied!")
        return True, stream, "OK"

    # ── DDR2 trigger-based training ───────────────────────────────────────────

    def run_ddr2(self) -> tuple:
        """DDR2 path: the dataset is already staged in DDR2 (the Load step did the
        OP_RESET + OP_LOAD_*). Here we just CONFIG, then fire data-less OP_TRAIN
        and OP_CALIB triggers — the engine replays the train region from DDR2 at
        engine clock — and finalize the threshold via OP_DUMP telemetry.

        Crucially we do NOT send OP_RESET here: that would wipe the staged DDR2.
        Each trigger is a single data-less frame; dataset_dma replays the whole
        region, and the in-order AXIS pipeline guarantees TRAIN completes before
        CALIB and CALIB before DUMP, so no inter-trigger host wait is needed.

        Returns (ok, telemetry_stream, message).
        """
        # ── CONFIG (weights + spike) ─────────────────────────────────────────
        if self._weights is not None:
            self._cb(0.05, "Sending OP_CONFIG…")
            self._c.send(self._c.pack_config_packet(
                self._active_weights(), self._spike_penalty, self._ac(),
                delta_stride=self._delta_stride, calib_shift=self._calib_shift))
            time.sleep(0.05)

        # ── TRAIN trigger: replay the DDR2 train region → build histograms ────
        self._cb(0.25, "TRAIN trigger — replaying train region from DDR2…")
        self._c.send(self._c.pack_frame([], self._ac(), OP_TRAIN, 0))
        time.sleep(0.05)

        # ── CALIB trigger: replay again → build the threshold distribution ────
        self._cb(0.55, "CALIB trigger — replaying train region from DDR2…")
        self._c.send(self._c.pack_frame([], self._ac(), OP_CALIBRATE, 0))
        time.sleep(0.05)
        self._c.drain(0.3)

        # ── finalize + telemetry (OP_DUMP) ───────────────────────────────────
        self._cb(0.85, f"Finalize + telemetry poll (up to {TELEMETRY_POLLS} OP_DUMP)…")
        ok, stream = self._collect_telemetry()
        if not ok:
            partial = " ".join(f"{b:02x}" for b in stream) if stream else "(empty)"
            return False, stream, f"Telemetry incomplete after DDR2 replay: {partial}"

        self._cb(1.00, "Ready!")
        return True, stream, "OK"

    # ── telemetry parsing (static helper for GUI) ────────────────────────────

    @staticmethod
    def parse_telemetry(stream: list) -> dict:
        """
        Decode the 5-byte threshold readback into a dict.

        Stream layout (one byte per OP_DUMP poll):
          [0]    0xFE  banner
          [1-3]  global_threshold  LE24
          [4]    0xFF  terminator

        (delta_th and the rx counters were removed from the FPGA telemetry —
        only the final global threshold is reported now.)
        """
        if not stream or stream[0] != 0xFE:
            return {}
        result: dict = {}
        if len(stream) >= 4:
            result["global_threshold"] = (
                stream[1] | (stream[2] << 8) | (stream[3] << 16)
            )
        return result
