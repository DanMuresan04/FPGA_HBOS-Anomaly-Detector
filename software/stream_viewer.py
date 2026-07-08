"""
Live FPGA stream viewer — main GUI entry point.

State machine
-------------
  IDLE      → user has not trained yet; Start disabled
  BUSY      → training background thread running; all buttons locked except Cancel
  READY     → FPGA config latch confirmed; Start enabled
  STREAMING → ping-pong detect loop running; Stop enabled
  ERROR     → last operation failed; lamp red

Usage
-----
  python3 -m licenta.gui.stream_viewer
"""

import csv
import gc
import os
import queue
import threading
import time
from pathlib import Path
from tkinter import messagebox, filedialog
import tkinter as tk
import tkinter.ttk as ttk

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from .udp_client import (UdpFpgaClient as FpgaClient, OP_DETECT,
                         OP_LOAD_TRAIN, OP_LOAD_TEST,
                         FPGA_IP, LoadError)
from .train_session import TrainSession
from .comparison import run_comparison, _SKLEARN_OK

try:
    import sv_ttk
    _HAS_THEME = True
except ImportError:
    _HAS_THEME = False

# ── tunables ─────────────────────────────────────────────────────────────────
VIEW_SIZE        = 120   # samples visible at once (scrollable)
STREAM_RATE_HZ   = 5
DETECT_TIMEOUT   = 1.0
PROGRESS_POLL_MS = 80
BURST_PLOT_EVERY = 50   # redraw interval in burst mode
# Inter-frame gap (s) for burst TX. No longer needed: the BD now has RX/TX AXIS
# FIFOs that absorb the board's emit stalls, and paced==burst confirmed there was
# never an overrun. Kept as a knob (0 = full speed); raise only if a future,
# slower board variant ever drops frames.
BURST_TX_GAP_S   = 0.0
PROGRESS_INTERVAL = 500  # rows between GUI progress updates (in train_session)

_DATASETS_ROOT = Path(__file__).parent.parent / "datasets"

# Dataset registry. Each entry fully specifies how to run that dataset: train
# file, test directory, detect-time delta stride, per-sensor weights, spike
# penalty, and calibration percentile. Picking a dataset in the UI applies all
# of these at once (see _apply_dataset) so there is nothing to remember per set.
#
#   water_quality  — the test file already has every 5th sample removed (those
#                    are the training samples), so a stride of 4 reaches exactly
#                    5 original samples back = the scale the histograms were
#                    trained on. Using stride 5 here would overshoot (~6.25 back)
#                    and collapse recall.
#   room_occupancy — contiguous stream, immediate-previous delta (stride 1).
DATASETS = {
    "water_quality": {
        "train":   str(_DATASETS_ROOT / "training" / "water_quality" / "water_quality_train.csv"),
        "test":    str(_DATASETS_ROOT / "test" / "water_quality"),
        "stride":  4,
        "weights": [50, 93, 58, 55],
        "spike":   5632,
        "calib":   "99.8",
    },
    "room_occupancy": {
        "train":   str(_DATASETS_ROOT / "training" / "room_occupancy" / "datatraining_stripped.csv"),
        "test":    str(_DATASETS_ROOT / "test" / "room_occupancy"),
        "stride":  1,
        "weights": [14, 44, 159, 37],
        "spike":   5632,
        "calib":   "99.8",
    },
}
DEFAULT_DATASET = "water_quality"

# Backward-compatible module constants, derived from the default dataset.
DEFAULT_TRAIN_CSV = DATASETS[DEFAULT_DATASET]["train"]
DEFAULT_TEST_DIR  = DATASETS[DEFAULT_DATASET]["test"]
DEFAULT_WEIGHTS   = DATASETS[DEFAULT_DATASET]["weights"]
DEFAULT_SPIKE     = DATASETS[DEFAULT_DATASET]["spike"]

# Global-threshold percentile knob (OP_CONFIG data[6]). The calibration keeps a
# fraction (1 - 2**-shift) of clean samples below threshold, i.e. that percentile.
# Higher shift -> stricter threshold -> higher precision / lower recall.
# Listed low->high so the spinbox arrows go up = stricter, down = more recall.
_CALIB_LEVELS = [
    (4,  "93.8"), (5,  "96.9"), (6,  "98.4"), (7,  "99.2"),
    (8,  "99.6"), (9,  "99.8"), (10, "99.9"), (11, "99.95"),
]
_CALIB_PCT_TO_SHIFT = {pct: shift for shift, pct in _CALIB_LEVELS}
_CALIB_PCT_VALUES   = [pct for _, pct in _CALIB_LEVELS]
DEFAULT_CALIB_PCT   = "99.8"   # shift 9 — the original hardcoded behavior

NR_SENSORS_MAX = 4    # toggle strip capacity — matches HLS NR_SENSORS (4-channel TDM build)
NR_SLOTS       = 4    # algorithm slots (current HLS packet limit)

SENSOR_LABELS = [f"s{i}" for i in range(NR_SENSORS_MAX)]
SENSOR_COLORS = [
    "#4fc3f7", "#80deea", "#a5d6a7", "#f48fb1",
    "#ffcc80", "#ef9a9a", "#80cbc4", "#b0bec5",
    "#f48fb1", "#c5e1a5", "#9fa8da", "#ffe082",
    "#b39ddb", "#90caf9", "#bcaaa4", "#ffab91",
]

ANOM_COLOR    = "#ff5252"

# ── palette (sv-ttk dark: TFrame bg is ~#1c1c1c) ─────────────────────────────
_BG       = "#1c1c1c"
_BG2      = "#2d2d30"
_BG3      = "#252526"
_FG       = "#d4d4d4"
_DIM      = "#6e6e6e"
_GREEN    = "#4ec94e"
_ORANGE   = "#d4a017"
_RED      = "#f44747"

# ── states ────────────────────────────────────────────────────────────────────
IDLE      = "idle"
BUSY      = "busy"
LOADED    = "loaded"
READY     = "ready"
STREAMING = "streaming"
ERROR     = "error"

_LAMP_CFG = {
    IDLE:      ("#2e2e3e", "#4a4a6a", _DIM,    "NOT READY"),
    BUSY:      ("#4a3700", "#c48a00", _ORANGE, "WORKING"),
    LOADED:    ("#0a3a4a", "#0a8ab0", "#4fc3f7", "DATA LOADED"),
    READY:     ("#1a4a1a", "#3aaa3a", _GREEN,  "READY"),
    STREAMING: ("#002244", "#0078cc", "#4fc3f7", "STREAMING"),
    ERROR:     ("#4a1a1a", "#cc3333", _RED,    "ERROR"),
}


# ── helpers ───────────────────────────────────────────────────────────────────

def _load_csv(path: str) -> list:
    rows = []
    with open(path, newline="") as fh:
        reader = csv.reader(fh)
        for r in reader:
            if len(r) >= 2:
                try:
                    float(r[0])   # skip header rows silently
                    rows.append(r)
                except ValueError:
                    pass
    return rows


def _load_csv_or_dir(path: str) -> list:
    """Load rows from a single CSV file or concatenate all *.csv files in a directory."""
    p = Path(path)
    if p.is_dir():
        rows = []
        for csv_file in sorted(p.glob("*.csv")):
            rows.extend(_load_csv(str(csv_file)))
        return rows
    return _load_csv(path)


# ── LED lamp ──────────────────────────────────────────────────────────────────

class _Lamp(tk.Frame):
    """Hardware-style LED indicator with a glowing oval."""

    _SIZE = 14

    def __init__(self, parent: tk.Widget, bg: str):
        super().__init__(parent, bg=bg)
        s = self._SIZE
        self._bg = bg
        self._cv = tk.Canvas(self, width=s + 4, height=s + 4,
                             bg=bg, bd=0, highlightthickness=0)
        self._cv.pack(side=tk.LEFT, padx=(0, 7))
        # outer glow ring + inner dot
        self._ring = self._cv.create_oval(0, 0, s + 3, s + 3,
                                          fill="#2e2e3e", outline="", width=0)
        self._dot  = self._cv.create_oval(3, 3, s, s,
                                          fill="#2e2e3e", outline="", width=0)
        self._lbl = tk.Label(self, text="NOT READY", bg=bg, fg=_DIM,
                             font=("Helvetica", 10, "bold"))
        self._lbl.pack(side=tk.LEFT)
        self.set_state(IDLE)

    def set_state(self, state: str) -> None:
        ring_c, dot_c, text_c, text = _LAMP_CFG.get(state, _LAMP_CFG[IDLE])
        self._cv.itemconfig(self._ring, fill=ring_c)
        self._cv.itemconfig(self._dot,  fill=dot_c)
        self._lbl.config(text=text, fg=text_c)

    def set_custom(self, text: str, fg: str) -> None:
        self._lbl.config(text=text, fg=fg)


# ── stat pill ─────────────────────────────────────────────────────────────────

class _Stat(tk.Frame):
    """A dim label + bright value stacked vertically — for the status bar."""

    def __init__(self, parent: tk.Widget, label: str, bg: str):
        super().__init__(parent, bg=bg)
        tk.Label(self, text=label.upper(), bg=bg, fg=_DIM,
                 font=("Helvetica", 7)).pack(anchor=tk.W)
        self._var = tk.StringVar(value="—")
        tk.Label(self, textvariable=self._var, bg=bg, fg=_FG,
                 font=("Courier", 10, "bold")).pack(anchor=tk.W)

    def set(self, v: str) -> None:
        self._var.set(v)


# ── main window ───────────────────────────────────────────────────────────────

class StreamViewer(tk.Tk):
    def __init__(self, mock: bool = False):
        super().__init__()
        self._mock = mock
        self.title("FPGA Live Stream Viewer")
        self.geometry("1080x720")
        self.minsize(800, 560)

        if _HAS_THEME:
            sv_ttk.set_theme("dark")

        # After sv_ttk applies, query the actual frame bg so custom tk widgets match
        self._bg = ttk.Style().lookup("TFrame", "background") or _BG

        self._state  = IDLE
        self._data_loaded = False   # True once a dataset is staged in DDR2
        self._client: FpgaClient | None = None
        self._cancel = threading.Event()
        # Live background threads, tracked so close/rerun can wait them out
        # instead of tearing down Tk/serial underneath them (which triggers
        # "Tcl_AsyncDelete: async handler deleted by the wrong thread").
        self._worker_thread: "threading.Thread | None" = None
        self._tx_thread:     "threading.Thread | None" = None
        self._comp_thread:   "threading.Thread | None" = None
        self._comp_gen = 0   # bumped each run; stale comparison results are ignored
        self._pq: queue.Queue = queue.Queue()
        self._comp_q: queue.Queue = queue.Queue()

        # Full history (unlimited) for scrollable replay
        self._all_xs:       list = []
        self._all_sensor:   list = []   # sized to len(_active_indices) at _on_start
        self._all_anom:     list = []   # True = FPGA detected
        self._all_csv_anom: list = []   # True = CSV label says anomaly

        self._view_start   = 0
        self._view_follows = True   # auto-scroll to follow new data

        self._sample_idx      = 0
        self._anom_count      = 0
        self._stream_rows: list = []
        self._stream_pos      = 0
        self._last_rtt        = 0

        # Comparison page state
        self._hbos_predictions: list  = []
        self._test_labels:      list  = []
        self._train_rows_cache: list  = []
        self._comp_results:     "dict | None" = None

        # Must be initialised before _build_ui() which binds these to Entry widgets.
        self._ip_var     = tk.StringVar(value=FPGA_IP)
        self._weight_vars = [tk.IntVar(value=w) for w in DEFAULT_WEIGHTS]
        self._spike_var   = tk.IntVar(value=DEFAULT_SPIKE)
        self._stride_var  = tk.IntVar(value=1)   # detect-time delta stride (1..8)
        self._calib_var   = tk.StringVar(value=DEFAULT_CALIB_PCT)  # threshold percentile
        self._dataset_var = tk.StringVar(value=DEFAULT_DATASET)    # dataset picker
        self._burst_var   = tk.BooleanVar(value=False)
        self._follow_var  = tk.BooleanVar(value=True)
        self._sensor_mask_vars = [tk.BooleanVar(value=False) for _ in range(NR_SENSORS_MAX)]
        self._csv_nr_sensors: int = 0          # detected from CSV on load
        self._active_indices: list = []        # CSV col per algorithm slot

        self._vline_artists: list = []

        self._page             = 0
        self._train_hist_data: "list | None" = None

        self._build_ui()

        # Fill train/test paths, stride, weights, spike and calib from the
        # default dataset (must run after _build_ui creates the bound widgets).
        self._apply_dataset(DEFAULT_DATASET)

        self._set_state(IDLE)
        self.configure(bg=self._bg)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Tkinter objects may only be finalized on the main (mainloop) thread.
        # Python's cyclic GC can otherwise run on a background worker (e.g. the
        # sklearn comparison thread allocating heavily) and finalize leftover
        # Tk/matplotlib objects there, calling Tcl from the wrong thread ->
        # "Tcl_AsyncDelete: async handler deleted by the wrong thread" / SIGABRT.
        # Disable automatic GC (ref-counted frees still happen immediately) and
        # run cyclic collection only here, on the main thread, via a timer.
        gc.disable()
        self.after(2000, self._gc_tick)
        # Auto-detect sensors from the default CSV if present
        _default = self._train_var.get()
        if _default and os.path.exists(_default):
            self._detect_csv_sensors(_default)

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        bg = self._bg

        # ── top panel: lamp + buttons ─────────────────────────────────────────
        top = ttk.Frame(self, padding=(12, 8))
        top.pack(fill=tk.X, side=tk.TOP)

        self._lamp = _Lamp(top, bg)
        self._lamp.pack(side=tk.LEFT, padx=(0, 16))

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        self._btn_load   = ttk.Button(top, text="Load Data", command=self._on_load, width=10)
        self._btn_train  = ttk.Button(top, text="Train",  command=self._on_train,  width=9)
        self._btn_reset  = ttk.Button(top, text="Reset",  command=self._on_reset,  width=9)
        self._btn_cancel = ttk.Button(top, text="Cancel", command=self._on_cancel, width=9)
        self._btn_start  = ttk.Button(top, text="▶  Start", command=self._on_start, width=11)
        self._btn_stop   = ttk.Button(top, text="■  Stop",  command=self._on_stop,  width=11)

        for b in (self._btn_load, self._btn_train, self._btn_reset, self._btn_cancel):
            b.pack(side=tk.LEFT, padx=3)
        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        for b in (self._btn_start, self._btn_stop):
            b.pack(side=tk.LEFT, padx=3)
        ttk.Checkbutton(top, text="Burst", variable=self._burst_var).pack(
            side=tk.LEFT, padx=(10, 0)
        )
        ttk.Checkbutton(top, text="Follow", variable=self._follow_var,
                        command=self._on_follow_toggle).pack(side=tk.LEFT, padx=(6, 0))

        # ── dataset picker ────────────────────────────────────────────────────
        dsrow = ttk.Frame(self, padding=(12, 4, 12, 0))
        dsrow.pack(fill=tk.X, side=tk.TOP)
        ttk.Label(dsrow, text="Dataset", foreground=_DIM,
                  font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 4))
        self._dataset_combo = ttk.Combobox(
            dsrow, textvariable=self._dataset_var, values=list(DATASETS.keys()),
            state="readonly", width=18
        )
        self._dataset_combo.pack(side=tk.LEFT)
        self._dataset_combo.bind(
            "<<ComboboxSelected>>", lambda _e: self._on_dataset_change()
        )
        ttk.Label(dsrow, text="(sets train/test paths, stride, weights, spike, pctile)",
                  foreground=_DIM, font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(8, 0))

        # ── file row ──────────────────────────────────────────────────────────
        files = ttk.Frame(self, padding=(12, 4, 12, 8))
        files.pack(fill=tk.X, side=tk.TOP)

        ttk.Label(files, text="Train CSV / Dir", foreground=_DIM,
                  font=("Helvetica", 8)).grid(row=0, column=0, sticky=tk.W, padx=(0, 4))
        self._train_var = tk.StringVar(
            value=DEFAULT_TRAIN_CSV if os.path.exists(DEFAULT_TRAIN_CSV) else ""
        )
        ttk.Entry(files, textvariable=self._train_var, width=46).grid(row=1, column=0, padx=(0, 2))
        ttk.Button(files, text="…", width=3,
                   command=self._pick_train).grid(row=1, column=1, padx=(0, 20))

        ttk.Label(files, text="Test CSV / Dir", foreground=_DIM,
                  font=("Helvetica", 8)).grid(row=0, column=2, sticky=tk.W, padx=(0, 4))
        self._test_var = tk.StringVar(
            value=DEFAULT_TEST_DIR if os.path.isdir(DEFAULT_TEST_DIR) else ""
        )
        ttk.Entry(files, textvariable=self._test_var, width=46).grid(row=1, column=2, padx=(0, 2))
        ttk.Button(files, text="…", width=3,
                   command=self._pick_test).grid(row=1, column=3)

        # ── serial port settings ──────────────────────────────────────────────
        conn = ttk.Frame(self, padding=(12, 0, 12, 6))
        conn.pack(fill=tk.X, side=tk.TOP)
        self._conn_frame = conn

        ttk.Label(conn, text="FPGA IP", foreground=_DIM,
                  font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Entry(conn, textvariable=self._ip_var, width=16).pack(side=tk.LEFT)

        # ── sensor weight config ───────────────────────────────────────────────
        wcfg = ttk.Frame(self, padding=(12, 0, 12, 6))
        wcfg.pack(fill=tk.X, side=tk.TOP)
        self._wcfg_frame = wcfg

        ttk.Label(wcfg, text="Sensors", foreground=_DIM,
                  font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 4))
        _sg = ttk.Frame(wcfg)
        _sg.pack(side=tk.LEFT, padx=(0, 2))
        self._sensor_checkboxes = []
        for i, var in enumerate(self._sensor_mask_vars):
            row_i, col_i = divmod(i, 8)
            cb = ttk.Checkbutton(_sg, text=str(i), variable=var, width=2,
                                 state=tk.DISABLED)
            cb.grid(row=row_i, column=col_i, padx=1, pady=0)
            self._sensor_checkboxes.append(cb)

        ttk.Separator(wcfg, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Label(wcfg, text="Weights", foreground=_DIM,
                  font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 8))
        for i, var in enumerate(self._weight_vars):
            ttk.Label(wcfg, text=f"w{i}", foreground=_DIM,
                      font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 2))
            ttk.Spinbox(wcfg, textvariable=var, from_=0, to=255,
                        width=4).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Separator(wcfg, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Label(wcfg, text="Spike", foreground=_DIM,
                  font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Spinbox(wcfg, textvariable=self._spike_var, from_=0, to=65535,
                    width=6).pack(side=tk.LEFT, padx=(0, 14))

        ttk.Label(wcfg, text="Δstride", foreground=_DIM,
                  font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Spinbox(wcfg, textvariable=self._stride_var, from_=1, to=8,
                    width=3).pack(side=tk.LEFT, padx=(0, 14))

        ttk.Label(wcfg, text="Thr pctile", foreground=_DIM,
                  font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Spinbox(wcfg, textvariable=self._calib_var, values=_CALIB_PCT_VALUES,
                    state="readonly", width=5).pack(side=tk.LEFT, padx=(0, 1))
        ttk.Label(wcfg, text="%", foreground=_DIM,
                  font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(0, 14))

        self._btn_apply = ttk.Button(wcfg, text="Apply Config",
                                     command=self._on_apply_config, width=12)
        self._btn_apply.pack(side=tk.LEFT)

        # ── error banner (hidden until an error occurs) ───────────────────────
        self._err_frame = tk.Frame(self, bg="#3d1414", padx=10, pady=6)
        # not packed yet — _show_error() inserts it after _conn_frame
        self._err_visible = False
        self._err_var = tk.StringVar(value="")
        tk.Label(self._err_frame, text="⚠", bg="#3d1414", fg=_RED,
                 font=("Helvetica", 12, "bold")).pack(side=tk.LEFT, padx=(0, 8))
        tk.Label(self._err_frame, textvariable=self._err_var,
                 bg="#3d1414", fg="#ff8a80",
                 font=("Helvetica", 9), anchor=tk.W, justify=tk.LEFT,
                 wraplength=820).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(self._err_frame, text="Dismiss",
                   command=self._dismiss_error).pack(side=tk.RIGHT)

        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=12)

        # ── status strip — packed BEFORE the canvas so expand=True leaves room ─
        sbar = tk.Frame(self, bg=_BG3, pady=6)
        sbar.pack(fill=tk.X, side=tk.BOTTOM)

        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=12, side=tk.BOTTOM)

        left  = tk.Frame(sbar, bg=_BG3)
        left.pack(side=tk.LEFT,  padx=14)
        right = tk.Frame(sbar, bg=_BG3)
        right.pack(side=tk.RIGHT, padx=14)

        self._st_sample = _Stat(left, "sample", _BG3)
        self._st_reply  = _Stat(left, "reply",  _BG3)
        self._st_rtt    = _Stat(left, "RTT",    _BG3)
        self._st_anom   = _Stat(left, "anomalies", _BG3)
        for w in (self._st_sample, self._st_reply, self._st_rtt, self._st_anom):
            w.pack(side=tk.LEFT, padx=12)

        # message + progress bar on the right
        msg_col = tk.Frame(right, bg=_BG3)
        msg_col.pack(side=tk.TOP, anchor=tk.E)
        self._status_var = tk.StringVar(value="Idle — select data and press Train")
        tk.Label(msg_col, textvariable=self._status_var, bg=_BG3, fg=_DIM,
                 font=("Helvetica", 8), anchor=tk.E).pack(anchor=tk.E)
        self._progress = ttk.Progressbar(right, length=220, maximum=1000, mode="determinate")
        self._progress.pack(anchor=tk.E, pady=(2, 0))

        # ── page navigation strip ─────────────────────────────────────────────
        nav = tk.Frame(self, bg=_BG3, pady=4)
        nav.pack(fill=tk.X, side=tk.TOP)

        self._btn_page_prev = ttk.Button(nav, text="◀", width=3,
                                          command=lambda: self._switch_page(-1))
        self._btn_page_prev.pack(side=tk.LEFT, padx=(12, 4))
        self._btn_page_prev.config(state=tk.DISABLED)

        self._page_label = tk.Label(nav, text="Streaming  1 / 3",
                                     bg=_BG3, fg=_DIM,
                                     font=("Helvetica", 8, "bold"))
        self._page_label.pack(side=tk.LEFT, padx=6)

        self._btn_page_next = ttk.Button(nav, text="▶", width=3,
                                          command=lambda: self._switch_page(1))
        self._btn_page_next.pack(side=tk.LEFT, padx=(4, 0))

        # ── page frames (stacked; only one visible at a time) ─────────────────
        self._stream_frame = tk.Frame(self, bg=_BG)
        self._hist_frame   = tk.Frame(self, bg=_BG)
        self._comp_frame   = tk.Frame(self, bg=_BG)

        # ── streaming page ────────────────────────────────────────────────────
        plt.style.use("dark_background")

        # Horizontal time-axis scrollbar (packed first so expand=True leaves room)
        self._scroll_var = tk.DoubleVar(value=1.0)
        self._scrollbar  = ttk.Scale(
            self._stream_frame,
            orient=tk.HORIZONTAL,
            variable=self._scroll_var,
            from_=0.0, to=1.0,
            command=self._on_scroll,
        )
        self._scrollbar.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=(0, 4))

        # Vertical scrollable container for the matplotlib figure
        _vscroll = ttk.Scrollbar(self._stream_frame, orient=tk.VERTICAL)
        _vscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self._plot_scroll_cv = tk.Canvas(
            self._stream_frame, bg=self._bg, highlightthickness=0,
            yscrollcommand=_vscroll.set,
        )
        self._plot_scroll_cv.pack(fill=tk.BOTH, expand=True)
        _vscroll.config(command=self._plot_scroll_cv.yview)

        def _vwheel(event):
            if event.num == 4:
                self._plot_scroll_cv.yview_scroll(-1, "units")
            elif event.num == 5:
                self._plot_scroll_cv.yview_scroll(1, "units")
            else:
                self._plot_scroll_cv.yview_scroll(-1 * (event.delta // 120), "units")
        self._plot_scroll_cv.bind("<MouseWheel>", _vwheel)
        self._plot_scroll_cv.bind("<Button-4>",   _vwheel)
        self._plot_scroll_cv.bind("<Button-5>",   _vwheel)

        self._plot_inner = tk.Frame(self._plot_scroll_cv, bg=self._bg)
        self._plot_cv_id = self._plot_scroll_cv.create_window(
            (0, 0), window=self._plot_inner, anchor="nw"
        )
        self._plot_inner.bind(
            "<Configure>",
            lambda e: (
                self._plot_scroll_cv.config(
                    scrollregion=self._plot_scroll_cv.bbox("all")
                ),
                self._plot_scroll_cv.itemconfig(
                    self._plot_cv_id,
                    width=self._plot_scroll_cv.winfo_width(),
                ),
            ),
        )
        self._plot_scroll_cv.bind(
            "<Configure>",
            lambda e: self._plot_scroll_cv.itemconfig(self._plot_cv_id, width=e.width),
        )

        # Build the initial figure (rebuilt on each Train with the right N)
        self._canvas: "FigureCanvasTkAgg | None" = None
        self._rebuild_stream_figure(NR_SLOTS)

        # ── histogram page ────────────────────────────────────────────────────
        self._build_hist_page()

        # ── comparison page ───────────────────────────────────────────────────
        self._build_comparison_page()

        # Page 0 (streaming) visible by default
        self._stream_frame.pack(fill=tk.BOTH, expand=True)

    # ── file pickers ─────────────────────────────────────────────────────────

    def _on_dataset_change(self):
        self._apply_dataset(self._dataset_var.get())

    def _apply_dataset(self, name: str) -> None:
        """Apply a registry dataset: fill train/test paths and every knob it
        pins (stride, weights, spike, calib percentile), then re-detect the
        sensor count from the new train CSV. Missing paths are left blank so the
        user can browse rather than the field silently pointing at nothing."""
        cfg = DATASETS.get(name)
        if not cfg:
            return
        self._train_var.set(cfg["train"] if os.path.exists(cfg["train"]) else "")
        self._test_var.set(cfg["test"] if os.path.isdir(cfg["test"]) else "")
        self._stride_var.set(cfg["stride"])
        for var, w in zip(self._weight_vars, cfg["weights"]):
            var.set(w)
        self._spike_var.set(cfg["spike"])
        self._calib_var.set(cfg["calib"])
        tp = self._train_var.get()
        if tp and os.path.exists(tp):
            self._detect_csv_sensors(tp)

    def _pick_train(self):
        p = filedialog.askdirectory(title="Select train CSV directory")
        if p:
            self._train_var.set(p)
            self._detect_csv_sensors(p)

    def _pick_test(self):
        p = filedialog.askdirectory(title="Select test CSV directory")
        if p:
            self._test_var.set(p)

    # ── sensor auto-detect ────────────────────────────────────────────────────

    def _detect_csv_sensors(self, path: str) -> None:
        """Read the first data row of the CSV, count sensor columns (all but last),
        enable those toggle slots and grey out the rest."""
        p = Path(path)
        if p.is_dir():
            csvs = sorted(p.glob("*.csv"))
            if not csvs:
                return
            path = str(csvs[0])
        try:
            with open(path, newline="") as fh:
                reader = csv.reader(fh)
                for row in reader:
                    if len(row) >= 2:
                        try:
                            float(row[0])
                            self._csv_nr_sensors = max(1, min(len(row) - 1, NR_SENSORS_MAX))
                            break
                        except ValueError:
                            continue
        except Exception:
            return
        self._refresh_sensor_toggles()

    def _refresh_sensor_toggles(self) -> None:
        """Sync toggle enabled/disabled state with CSV column count and app state."""
        editable = self._state in (IDLE, ERROR)
        for i, (cb, var) in enumerate(zip(self._sensor_checkboxes, self._sensor_mask_vars)):
            if i >= self._csv_nr_sensors:
                var.set(False)
                cb.config(state=tk.DISABLED)
            elif editable:
                cb.config(state=tk.NORMAL)
            else:
                cb.config(state=tk.DISABLED)

    def _rebuild_stream_figure(self, n: int) -> None:
        """Destroy the old matplotlib figure and create a new one with n subplots.
        Called at training time so the graph always matches the active sensor count."""
        if self._canvas is not None:
            self._canvas.get_tk_widget().destroy()
            plt.close(self._fig)
            self._canvas = None
            self._fig = None
            gc.collect()  # collect PhotoImage/Variable objects here, not in background thread

        height = max(3.5, n * 1.5)
        self._fig, axes_raw = plt.subplots(n, 1, figsize=(10, height), sharex=True)
        self._axes = [axes_raw] if n == 1 else list(axes_raw)
        self._fig.patch.set_facecolor(self._bg)
        self._fig.subplots_adjust(left=0.06, right=0.98, top=0.97, bottom=0.06, hspace=0.08)

        active = self._active_indices if self._active_indices else list(range(n))
        self._lines, self._scatters, self._fn_scatters = [], [], []
        for slot, ax in enumerate(self._axes):
            csv_col = active[slot] if slot < len(active) else slot
            col   = SENSOR_COLORS[csv_col % len(SENSOR_COLORS)]
            label = SENSOR_LABELS[csv_col] if csv_col < len(SENSOR_LABELS) else f"s{csv_col}"
            ax.set_facecolor(_BG2)
            ax.set_ylabel(label, color=_DIM, fontsize=8, labelpad=4)
            ax.tick_params(colors=_DIM, labelsize=7, length=2)
            ax.grid(True, color="#555560", linewidth=0.4, linestyle="--", alpha=0.6)
            for side, sp in ax.spines.items():
                sp.set_visible(side in ("left", "bottom"))
                sp.set_edgecolor("#3a3a40")
                sp.set_linewidth(0.7)
            ln, = ax.plot([], [], color=col, linewidth=0.9, alpha=0.9)
            sc  = ax.scatter([], [], color=ANOM_COLOR, s=22, zorder=5,
                             edgecolors="#ff8a80", linewidths=0.6)
            fn  = ax.scatter([], [], color="#4fc3f7", s=40, zorder=6,
                             marker="x", linewidths=1.4)
            self._lines.append(ln)
            self._scatters.append(sc)
            self._fn_scatters.append(fn)

        stream_cv = FigureCanvasTkAgg(self._fig, master=self._plot_inner)
        stream_cv.get_tk_widget().pack(fill=tk.X, expand=False)
        stream_cv.get_tk_widget().configure(bg=self._bg, highlightthickness=0)
        self._canvas = stream_cv
        self._plot_scroll_cv.yview_moveto(0.0)

    # ── state machine ─────────────────────────────────────────────────────────

    def _set_state(self, s: str):
        self._state = s
        self._lamp.set_state(s)
        idle_like = s in (IDLE, LOADED, READY, ERROR)
        # Train is a DDR2 replay trigger: only meaningful once data is staged.
        can_train = idle_like and self._data_loaded
        self._btn_load.config(  state=tk.NORMAL if idle_like             else tk.DISABLED)
        self._btn_train.config( state=tk.NORMAL if can_train             else tk.DISABLED)
        self._btn_reset.config( state=tk.NORMAL if idle_like             else tk.DISABLED)
        self._btn_start.config( state=tk.NORMAL if s == READY            else tk.DISABLED)
        self._btn_stop.config(  state=tk.NORMAL if s == STREAMING        else tk.DISABLED)
        self._btn_cancel.config(state=tk.NORMAL if s == BUSY             else tk.DISABLED)
        self._btn_apply.config( state=tk.NORMAL if s == READY            else tk.DISABLED)
        self._refresh_sensor_toggles()
        if s != ERROR:
            self._dismiss_error()

    # ── error banner ──────────────────────────────────────────────────────────

    def _show_error(self, msg: str) -> None:
        self._err_var.set(msg)
        if not self._err_visible:
            self._err_frame.pack(fill=tk.X, padx=12, pady=2,
                                 after=self._conn_frame)
            self._err_visible = True

    def _dismiss_error(self) -> None:
        if self._err_visible:
            self._err_frame.pack_forget()
            self._err_visible = False

    # ── Train ─────────────────────────────────────────────────────────────────

    # ── Load Data (stage both datasets into DDR2 once) ─────────────────────────

    def _on_load(self):
        """Stream the train + test CSVs into the FPGA's DDR2 exactly once
        (OP_RESET then OP_LOAD_TRAIN / OP_LOAD_TEST). Train and Start afterwards
        replay from DDR2 at engine clock instead of re-streaming over UART."""
        if self._state not in (IDLE, LOADED, READY, ERROR):
            return

        train_path = self._train_var.get().strip()
        test_path  = self._test_var.get().strip()
        if not train_path or not os.path.exists(train_path):
            messagebox.showerror("Train data missing", "Select a valid train CSV or directory first.")
            return
        if not test_path or not os.path.exists(test_path):
            messagebox.showerror("Test data missing", "Select a valid test CSV or directory first.")
            return

        try:
            train_rows = _load_csv_or_dir(train_path)
            test_rows  = _load_csv_or_dir(test_path)
        except Exception as exc:
            messagebox.showerror("CSV error", str(exc))
            return
        if not train_rows:
            messagebox.showerror("Empty CSV", "No data rows found in train CSV.")
            return
        if not test_rows:
            messagebox.showerror("Empty dataset", "No data rows found in test CSV(s).")
            return

        # Re-detect sensors (handles manually typed path) and resolve active slots.
        self._detect_csv_sensors(train_path)
        active = [i for i in range(self._csv_nr_sensors)
                  if self._sensor_mask_vars[i].get()]
        if not active:
            messagebox.showerror("No sensors selected", "Enable at least one sensor.")
            return
        # Hardware UART is capped at 4 sensor slots; mock supports any count
        if not self._mock and len(active) > NR_SLOTS:
            active = active[:NR_SLOTS]
        self._active_indices = active

        # Rebuild figures + cache train/test rows on the host (used for plotting,
        # histograms, the sklearn comparison, and CSV-label ground truth).
        self._rebuild_stream_figure(len(active))
        self._rebuild_hist_figure(len(active))
        self._train_rows_cache = train_rows
        self._stream_rows      = test_rows
        self._comp_results     = None
        self._train_hist_data = [
            np.array([float(row[col]) for row in train_rows], dtype=np.float64)
            for col in active
        ]
        if self._page == 1:
            self._render_histograms()

        if self._mock:
            from .mock_client import MockFpgaClient
            self._client = MockFpgaClient(nr_sensors=len(active),
                                          simulated_rx_count=len(train_rows))
        elif self._client is None:
            try:
                self._client = FpgaClient(fpga_ip=self._ip_var.get().strip())
            except Exception as exc:
                messagebox.showerror("UDP socket error", str(exc))
                return

        self._data_loaded = False
        self._cancel.clear()
        self._set_state(BUSY)
        self._lamp.set_custom("LOADING", "#4fc3f7")
        self._progress["value"] = 0
        self._progress["mode"]  = "determinate"
        total = len(train_rows) + len(test_rows)
        self._status_var.set(f"Loading {total:,} samples to DDR2…")
        threading.Thread(target=self._load_worker,
                         args=(train_rows, test_rows, list(active)), daemon=True).start()
        self.after(PROGRESS_POLL_MS, self._poll_train_queue)

    def _load_worker(self, train_rows: list, test_rows: list, active: list):
        try:
            c = self._client
            # OP_RESET wipes DDR2 + engine so a reload never appends to stale data.
            c.send_reset()
            time.sleep(0.05)
            c.drain(0.2)

            # Bulk UDP load: 16-byte rows blasted to the data port, receipt
            # bitmap verified via LOAD_STATUS, holes resent (udp_client). The
            # row label is pre-embedded at bit 127 so the FPGA trains on clean
            # samples only (mock keeps the legacy per-frame path).
            def _prep(rows):
                vals, labels = [], []
                for row in rows:
                    vals.append([int(float(row[col])) if col < len(row) else 0
                                 for col in active])
                    labels.append(1 if (len(row) >= 1 and int(float(row[-1])) != 0) else 0)
                return vals, labels

            if self._mock:
                total = len(train_rows) + len(test_rows)
                done = 0
                for opcode, rows in ((OP_LOAD_TRAIN, train_rows), (OP_LOAD_TEST, test_rows)):
                    vals, labels = _prep(rows)
                    for i, (v, l) in enumerate(zip(vals, labels)):
                        if self._cancel.is_set():
                            self._pq.put(("done", False, None, "Cancelled during load", "load"))
                            return
                        c.send(c.pack_frame(v, len(v), opcode, l, i))
                        done += 1
                        if done % PROGRESS_INTERVAL == 0:
                            self._pq.put(("progress", done / total,
                                          f"Loading to DDR2 {done:,}/{total:,}"))
            else:
                t0 = time.monotonic()
                for region, rows, name in ((0, train_rows, "train"), (1, test_rows, "test")):
                    if self._cancel.is_set():
                        self._pq.put(("done", False, None, "Cancelled during load", "load"))
                        return
                    vals, labels = _prep(rows)
                    base = 0.0 if region == 0 else 0.5
                    self._pq.put(("progress", base,
                                  f"Blasting {name} region ({len(rows):,} rows)…"))
                    info = c.load_region(vals, region=region, labels=labels,
                                         active_count=len(active))
                    self._pq.put(("progress", base + 0.5,
                                  f"{name}: {info['rows']:,} rows in {info['blocks']:,} "
                                  f"blocks, {info['rounds']} round(s)"))
                dt = time.monotonic() - t0
                print(f"[load] {len(train_rows)+len(test_rows):,} rows in {dt:.2f}s "
                      f"({16*(len(train_rows)+len(test_rows))/dt/1e6:.2f} MB/s)")

            c.drain(0.2)
            self._pq.put(("loaded",
                          f"Staged {len(train_rows):,} train + {len(test_rows):,} test "
                          f"samples in DDR2 — press Train."))
        except LoadError as exc:
            self._pq.put(("done", False, None, f"Load failed: {exc}", "load"))
        except Exception as exc:
            import traceback; traceback.print_exc()
            self._pq.put(("done", False, None, f"Load failed: {exc}", "load"))

    # ── Train (replay train region from DDR2: TRAIN + CALIB triggers) ──────────

    def _on_train(self):
        if not self._data_loaded:
            messagebox.showerror("No data loaded", "Press Load Data first.")
            return
        if self._state not in (LOADED, READY, ERROR):
            return
        if self._client is None:
            messagebox.showerror("Not connected", "Load data first.")
            return

        active = self._active_indices
        self._comp_results = None
        self._cancel.clear()
        self._set_state(BUSY)
        self._lamp.set_custom("TRAINING", _ORANGE)
        self._progress["value"] = 0
        self._progress["mode"]  = "determinate"
        self._status_var.set("Training from DDR2 (replay)…")

        weights     = [v.get() for v in self._weight_vars]
        spike       = self._spike_var.get()
        sensor_mask = [j < len(active) for j in range(NR_SLOTS)]
        active_cols = list(active)
        session = TrainSession(self._client, [], self._train_progress_cb, self._cancel,
                               weights=weights, spike_penalty=spike, sensor_mask=sensor_mask,
                               active_sensor_cols=active_cols,
                               delta_stride=self._stride_var.get(),
                               calib_shift=_CALIB_PCT_TO_SHIFT.get(self._calib_var.get(), 9))
        threading.Thread(target=self._train_worker_ddr2, args=(session,), daemon=True).start()
        self.after(PROGRESS_POLL_MS, self._poll_train_queue)

    def _train_worker_ddr2(self, session: TrainSession):
        try:
            ok, stream, msg = session.run_ddr2()
        except Exception as exc:
            import traceback
            traceback.print_exc()
            ok, stream, msg = False, None, f"Worker error: {exc}"
        self._pq.put(("done", ok, stream, msg))

    # ── Reset / flush ──────────────────────────────────────────────────────────

    def _on_reset(self):
        """Send a standalone OP_RESET to flush all FPGA engine state. Leaves the
        device untrained (state → IDLE), so a Train is required afterwards."""
        if self._state not in (IDLE, READY, ERROR):
            return
        if self._mock:
            from .mock_client import MockFpgaClient
            if self._client is None:
                self._client = MockFpgaClient(nr_sensors=max(1, len(self._active_indices)))
        elif self._client is None:
            try:
                self._client = FpgaClient(fpga_ip=self._ip_var.get().strip())
            except Exception as exc:
                messagebox.showerror("UDP socket error", str(exc))
                return

        self._cancel.clear()
        self._set_state(BUSY)
        self._lamp.set_custom("FLUSHING", _ORANGE)
        self._progress["mode"]  = "determinate"
        self._progress["value"] = 0
        self._status_var.set("Flushing FPGA state (OP_RESET)…")
        threading.Thread(target=self._reset_worker, daemon=True).start()
        self.after(PROGRESS_POLL_MS, self._poll_train_queue)

    def _reset_worker(self):
        try:
            self._client.send_reset()
            time.sleep(0.05)
            self._client.drain(0.2)
            self._pq.put(("reset_done", "FPGA state flushed — train to use."))
        except Exception as exc:
            self._pq.put(("done", False, None, f"Reset failed: {exc}", "reset"))

    def _train_progress_cb(self, frac: float, msg: str):
        self._pq.put(("progress", frac, msg))

    def _train_worker(self, session: TrainSession):
        try:
            ok, stream, msg = session.run()
        except Exception as exc:
            import traceback
            traceback.print_exc()
            ok, stream, msg = False, None, f"Worker error: {exc}"
        self._pq.put(("done", ok, stream, msg))

    def _poll_train_queue(self):
        try:
            while True:
                item = self._pq.get_nowait()
                if item[0] == "progress":
                    _, frac, msg = item
                    self._progress["value"] = int(frac * 1000)
                    self._status_var.set(msg)
                elif item[0] == "reset_done":
                    self._progress["value"] = 0
                    self._data_loaded = False
                    self._status_var.set(item[1])
                    self._set_state(IDLE)
                    return
                elif item[0] == "loaded":
                    self._data_loaded = True
                    self._progress["value"] = 1000
                    self._status_var.set(item[1])
                    self._set_state(LOADED)
                    return
                elif item[0] == "done":
                    ok, stream, msg = item[1], item[2], item[3]
                    tag = item[4] if len(item) > 4 else "train"
                    if ok:
                        parsed = TrainSession.parse_telemetry(stream)
                        detail = ""
                        if parsed:
                            detail = f"  global threshold = {parsed.get('global_threshold','?')}"
                        self._progress["value"] = 1000
                        prefix = "FPGA ready." if tag == "train" else "Config applied."
                        self._status_var.set(f"{prefix}{detail}")
                        self._set_state(READY)
                    else:
                        self._progress["value"] = 0
                        self._status_var.set(f"Failed: {msg}")
                        self._set_state(ERROR)
                        label = "Training" if tag == "train" else "Config update"
                        self._show_error(f"{label} failed: {msg}")
                    return
        except queue.Empty:
            pass

        if self._state == BUSY:
            self.after(PROGRESS_POLL_MS, self._poll_train_queue)

    # ── Apply Config ──────────────────────────────────────────────────────────

    def _on_apply_config(self):
        if self._state != READY:
            return

        weights     = [v.get() for v in self._weight_vars]
        spike       = self._spike_var.get()
        sensor_mask = [v.get() for v in self._sensor_mask_vars]

        self._cancel.clear()
        self._set_state(BUSY)
        self._lamp.set_custom("CONFIGURING", _ORANGE)
        self._progress["value"] = 0
        self._progress["mode"]  = "determinate"
        self._status_var.set("Updating sensor config…")

        session = TrainSession(self._client, [], self._train_progress_cb, self._cancel,
                               weights=weights, spike_penalty=spike, sensor_mask=sensor_mask,
                               delta_stride=self._stride_var.get(),
                               calib_shift=_CALIB_PCT_TO_SHIFT.get(self._calib_var.get(), 9))
        threading.Thread(target=self._apply_config_worker, args=(session,),
                         daemon=True).start()
        self.after(PROGRESS_POLL_MS, self._poll_train_queue)

    def _apply_config_worker(self, session: TrainSession):
        try:
            ok, stream, msg = session.run_config_update()
        except Exception as exc:
            import traceback
            traceback.print_exc()
            ok, stream, msg = False, None, f"Worker error: {exc}"
        self._pq.put(("done", ok, stream, msg, "config"))

    # ── Cancel ────────────────────────────────────────────────────────────────

    def _on_cancel(self):
        self._cancel.set()
        self._status_var.set("Cancelling…")

    # ── Start ─────────────────────────────────────────────────────────────────

    def _on_start(self):
        if self._state != READY:
            return

        # Test data was streamed into DDR2 by Load Data; reuse those exact rows.
        # The FPGA replays them from DDR2, so re-reading the CSV here could desync
        # the host's plot/labels from what the hardware actually scores.
        if not self._stream_rows:
            messagebox.showerror("No data loaded", "Press Load Data first.")
            return

        self._stream_pos    = 0
        self._sample_idx    = 0
        self._anom_count    = 0
        self._all_xs        = []
        self._all_sensor    = [[] for _ in range(len(self._active_indices))]
        self._all_anom      = []
        self._all_csv_anom  = []
        self._view_start    = 0
        self._view_follows  = True
        self._follow_var.set(True)
        self._vline_artists = []
        self._scroll_var.set(1.0)

        # Reset per-run comparison state
        self._hbos_predictions = []
        self._test_labels = [
            1 if int(float(r[-1])) != 0 else 0
            for r in self._stream_rows
            if len(r) >= 2
        ]
        self._comp_results = None
        self._show_comparison_pending()

        for stat in (self._st_sample, self._st_reply, self._st_rtt, self._st_anom):
            stat.set("—")

        # A previous fast run's worker/tx may still be draining; cancel and let
        # them exit before reusing the serial port to avoid interleaved traffic.
        self._cancel.set()
        self._join_stream_threads(timeout=1.0)

        self._comp_gen += 1   # invalidate any in-flight comparison from a prior run
        self._cancel.clear()
        self._set_state(STREAMING)
        self._progress["mode"]  = "determinate"
        self._progress["value"] = 0
        self._status_var.set("Streaming…")
        self._stream_burst = self._burst_var.get()
        self._last_rtt     = 0
        self._stream_q     = queue.Queue()
        self._worker_thread = threading.Thread(target=self._stream_worker, daemon=True)
        self._worker_thread.start()
        self.after(PROGRESS_POLL_MS, self._poll_stream_queue)

    def _join_stream_threads(self, timeout: float = 1.0):
        """Wait briefly for the stream worker + tx threads to exit. Used before
        reusing the serial port (rerun) or tearing down Tk (close)."""
        deadline = time.monotonic() + timeout
        for t in (self._tx_thread, self._worker_thread):
            if t is not None and t.is_alive():
                t.join(max(0.0, deadline - time.monotonic()))

    # ── streaming loop ────────────────────────────────────────────────────────

    # The detect stream runs on a background worker thread: it sends each
    # OP_DETECT packet and waits (off the Tk loop) for the 1-byte verdict,
    # pushing results to _stream_q.  The main thread drains the queue on a timer
    # and redraws the plot once per poll, so throughput is bound by the UART
    # round-trip rather than the Tk after() granularity.  Non-burst mode still
    # paces the worker to STREAM_RATE_HZ for a watchable live view.

    def _stream_worker(self):
        # DDR2 detect: a single data-less OP_DETECT trigger makes the FPGA replay
        # the staged test region, emitting a 4-byte [seq:3][verdict:1] reply per
        # sample. Each verdict is placed by its own seq (stamped by the FPGA ==
        # load order == row index), so a dropped/reordered reply only affects that
        # one sample. Replies still arrive over UART, so the plot animates live.
        rows   = self._stream_rows
        active = self._active_indices
        q      = self._stream_q
        n      = len(rows)

        # Pre-build per-row payloads + CSV labels; the RX consumer indexes these
        # by the seq the FPGA stamps.
        vals_list, csv_list = [], []
        for pos in range(n):
            row = rows[pos]
            if len(row) < 2:
                q.put(("error", f"row {pos} has {len(row)} columns, need ≥2"))
                return
            csv_list.append(int(float(row[-1])) != 0)
            vals_list.append([float(row[active[j]]) if active[j] < len(row) else 0.0
                              for j in range(len(active))])

        try:
            # Flush stale telemetry/ack bytes, then fire the one detect trigger.
            self._client.drain(0.1)
            self._client.send(self._client.pack_frame([], len(active), OP_DETECT, 0))

            received      = 0
            buf           = bytearray()
            prev_t        = time.monotonic()
            idle_deadline = None
            while received < n:
                if self._cancel.is_set():
                    q.put(("done", "Stopped."))
                    return
                chunk = self._client.read_available()
                if not chunk:
                    # No bytes yet: allow for the replay + UART round-trip, then
                    # treat extended silence as missing replies.
                    if idle_deadline is None:
                        idle_deadline = time.monotonic() + DETECT_TIMEOUT
                    elif time.monotonic() > idle_deadline:
                        q.put(("timeout", received))
                        q.put(("done", None))
                        return
                    time.sleep(0.0005)
                    continue

                idle_deadline = None
                now    = time.monotonic()
                rtt_ms = int((now - prev_t) * 1000)   # chunk inter-arrival proxy
                prev_t = now
                buf.extend(chunk)
                while len(buf) >= 4 and received < n:
                    seq = buf[0] | (buf[1] << 8) | (buf[2] << 16)
                    verdict = buf[3]
                    del buf[:4]
                    if seq >= n:
                        continue   # stray/desync guard
                    q.put(("sample", seq + 1, verdict == 0x01, csv_list[seq],
                           vals_list[seq], rtt_ms))
                    received += 1
            q.put(("done", None))
        except Exception as exc:
            import traceback; traceback.print_exc()
            q.put(("error", f"Stream error: {exc}"))

    def _poll_stream_queue(self):
        # Refresh the burst flag on the main thread; the worker reads the bool.
        self._stream_burst = self._burst_var.get()

        got_sample = False
        done = None
        try:
            while True:
                item = self._stream_q.get_nowait()
                kind = item[0]
                if kind == "sample":
                    _, pos, is_anom, csv_anom, vals, rtt_ms = item
                    self._stream_pos = pos
                    self._all_xs.append(self._sample_idx)
                    for slot in range(len(self._active_indices)):
                        self._all_sensor[slot].append(
                            vals[slot] if slot < len(vals) else 0.0)
                    self._all_anom.append(is_anom)
                    self._all_csv_anom.append(csv_anom)
                    self._hbos_predictions.append(1 if is_anom else 0)
                    if is_anom:
                        self._anom_count += 1
                    self._sample_idx += 1
                    self._last_rtt = rtt_ms
                    got_sample = True
                elif kind == "timeout":
                    self._stream_pos = item[1]
                    self._status_var.set(
                        f"timeout at sample {self._sample_idx} "
                        f"({self._stream_pos}/{len(self._stream_rows)})"
                    )
                elif kind == "done":
                    done = (False, item[1]); break
                elif kind == "error":
                    done = (True, item[1]); break
        except queue.Empty:
            pass

        if got_sample:
            if self._view_follows:
                self._view_start = max(0, len(self._all_xs) - VIEW_SIZE)
                self._update_scrollbar()
            try:
                self._update_plot()
            except Exception:
                import traceback; traceback.print_exc()
            pct = self._stream_pos / max(len(self._stream_rows), 1)
            self._progress["value"] = int(pct * 1000)
            self._st_sample.set(str(self._sample_idx - 1))
            self._st_reply.set("anomaly" if self._all_anom[-1] else "normal")
            self._st_rtt.set(f"{self._last_rtt} ms")
            self._st_anom.set(f"{self._anom_count} / {self._sample_idx}")
            self._status_var.set(
                f"{self._stream_pos:,} / {len(self._stream_rows):,} packets sent"
            )

        if done is not None:
            is_error, dmsg = done
            if is_error:
                self._finish_stream(dmsg, error=True)
            else:
                self._finish_stream(
                    dmsg or f"Done — {self._sample_idx:,} samples, "
                            f"{self._anom_count} anomalies."
                )
            return

        if self._state == STREAMING:
            self.after(PROGRESS_POLL_MS, self._poll_stream_queue)

    def _finish_stream(self, msg: str, error: bool = False) -> None:
        self._set_state(ERROR if error else READY)
        self._status_var.set(msg)
        try:
            self._update_plot()
        except Exception:
            pass
        if error:
            self._show_error(msg)

        # Launch comparison in a background thread; poll results via main-thread queue.
        # With the pipelined path the stream finishes near-instantly, so the
        # (slower) sklearn comparison is usually still running when the user
        # flips to tab 3 — show an explicit progress message, not the idle
        # placeholder, so it doesn't look like nothing happened.
        if self._hbos_predictions and self._test_labels:
            self._show_comparison_computing()
            gen = self._comp_gen
            self._comp_thread = threading.Thread(
                target=lambda: self._comparison_worker(gen),
                daemon=True,
            )
            self._comp_thread.start()
            self.after(200, self._poll_comp_queue)

    # ── Stop ─────────────────────────────────────────────────────────────────

    def _on_stop(self):
        self._cancel.set()

    # ── plot update ───────────────────────────────────────────────────────────

    def _update_plot(self):
        n  = len(self._all_xs)
        vs = self._view_start
        ve = min(vs + VIEW_SIZE, n)

        xs_view   = self._all_xs[vs:ve]
        anom_view = self._all_anom[vs:ve]
        csv_view  = self._all_csv_anom[vs:ve]

        anom_xs = [x for x, a in zip(xs_view, anom_view) if a]

        # True positives: HBOS flagged AND CSV label agrees → get vertical lines
        # False positives: HBOS flagged but CSV says normal → dot only, no line
        if csv_view:
            tp_xs = [x for x, a, c in zip(xs_view, anom_view, csv_view) if a and c]
        else:
            tp_xs = anom_xs

        # Remove previous vertical lines
        for art in self._vline_artists:
            try:
                art.remove()
            except Exception:
                pass
        self._vline_artists = []

        active = self._active_indices
        for slot, (ax, line, sc, fn_sc) in enumerate(
                zip(self._axes, self._lines, self._scatters, self._fn_scatters)):

            if slot >= len(active):
                line.set_data([], [])
                sc.set_offsets(np.empty((0, 2)))
                fn_sc.set_offsets(np.empty((0, 2)))
                ax.set_ylabel("—", color="#444450", fontsize=8, labelpad=4)
                continue

            csv_col = active[slot]
            label   = SENSOR_LABELS[csv_col]
            col     = SENSOR_COLORS[csv_col]
            vals    = self._all_sensor[slot][vs:ve]
            line.set_data(xs_view, vals)
            line.set_color(col)
            line.set_alpha(0.85)
            ax.set_ylabel(label, color=_DIM, fontsize=8, labelpad=4)

            # FPGA-detected anomalies → red dots
            ay = [v for v, a in zip(vals, anom_view) if a]
            sc.set_offsets(list(zip(anom_xs, ay)) if anom_xs else np.empty((0, 2)))

            # Missed anomalies (false negatives) → blue X
            fn_x = [x for x, a, c in zip(xs_view, anom_view, csv_view) if c and not a]
            fn_y = [v for v, a, c in zip(vals,    anom_view, csv_view) if c and not a]
            fn_sc.set_offsets(list(zip(fn_x, fn_y)) if fn_x else np.empty((0, 2)))

            ax.relim()
            ax.autoscale_view()

            # Vertical red lines only at true positives (not false positives)
            for x in tp_xs:
                vl = ax.axvline(x, color="#ff5252", linewidth=0.8, alpha=0.35, zorder=2)
                self._vline_artists.append(vl)

        self._canvas.draw_idle()

    # ── scrollbar / follow ────────────────────────────────────────────────────

    def _update_scrollbar(self):
        n         = len(self._all_xs)
        max_start = max(0, n - VIEW_SIZE)
        self._scroll_var.set(self._view_start / max_start if max_start else 1.0)

    def _on_scroll(self, val):
        n         = len(self._all_xs)
        max_start = max(0, n - VIEW_SIZE)
        new_start = int(float(val) * max_start)
        if new_start == self._view_start:
            return
        self._view_start   = new_start
        self._view_follows = (new_start >= max_start)
        self._follow_var.set(self._view_follows)
        try:
            self._update_plot()
        except Exception:
            pass

    def _on_follow_toggle(self):
        self._view_follows = self._follow_var.get()
        if self._view_follows:
            n = len(self._all_xs)
            self._view_start = max(0, n - VIEW_SIZE)
            self._update_scrollbar()
            try:
                self._update_plot()
            except Exception:
                pass

    # ── page navigation ───────────────────────────────────────────────────────

    def _switch_page(self, delta: int):
        n = 3
        self._page = max(0, min(n - 1, self._page + delta))
        for frame in (self._stream_frame, self._hist_frame, self._comp_frame):
            frame.pack_forget()
        if self._page == 0:
            self._stream_frame.pack(fill=tk.BOTH, expand=True)
            self._page_label.config(text="Streaming  1 / 3")
        elif self._page == 1:
            self._hist_frame.pack(fill=tk.BOTH, expand=True)
            self._page_label.config(text="Histograms  2 / 3")
            self._render_histograms()
        else:
            self._comp_frame.pack(fill=tk.BOTH, expand=True)
            self._page_label.config(text="Comparison  3 / 3")
            if self._comp_results:
                self._render_comparison(self._comp_results)
        self._btn_page_prev.config(state=tk.NORMAL if self._page > 0 else tk.DISABLED)
        self._btn_page_next.config(state=tk.NORMAL if self._page < n - 1 else tk.DISABLED)

    # ── comparison page ───────────────────────────────────────────────────────

    def _build_comparison_page(self):
        bg = _BG
        outer = self._comp_frame

        # Scrollable canvas so the page works at any window size
        vscroll = ttk.Scrollbar(outer, orient=tk.VERTICAL)
        vscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self._comp_canvas = tk.Canvas(outer, bg=bg, highlightthickness=0,
                                      yscrollcommand=vscroll.set)
        self._comp_canvas.pack(fill=tk.BOTH, expand=True)
        vscroll.config(command=self._comp_canvas.yview)

        def _wheel(event):
            if event.num == 4:
                self._comp_canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                self._comp_canvas.yview_scroll(1, "units")
            else:
                self._comp_canvas.yview_scroll(-1 * (event.delta // 120), "units")

        self._comp_canvas.bind("<MouseWheel>", _wheel)
        self._comp_canvas.bind("<Button-4>",   _wheel)
        self._comp_canvas.bind("<Button-5>",   _wheel)

        self._comp_inner = tk.Frame(self._comp_canvas, bg=bg)
        self._comp_win_id = self._comp_canvas.create_window(
            (0, 0), window=self._comp_inner, anchor="nw"
        )
        self._comp_inner.bind(
            "<Configure>",
            lambda e: (
                self._comp_canvas.config(
                    scrollregion=self._comp_canvas.bbox("all")
                ),
                self._comp_canvas.itemconfig(
                    self._comp_win_id,
                    width=self._comp_canvas.winfo_width()
                ),
            )
        )
        self._comp_canvas.bind(
            "<Configure>",
            lambda e: self._comp_canvas.itemconfig(
                self._comp_win_id, width=e.width
            )
        )

        # Initial placeholder
        self._show_comparison_pending()

    def _show_comparison_pending(self):
        """Clear the comparison panel and show a waiting message."""
        for w in self._comp_inner.winfo_children():
            w.destroy()
        msg = ("Run ▶ Start to compare FPGA HBOS against\n"
               "CPU HBOS, Isolation Forest, and One-Class SVM.")
        if not _SKLEARN_OK:
            msg = "scikit-learn not installed.\npip install scikit-learn to enable comparison."
        tk.Label(
            self._comp_inner, text=msg,
            bg=_BG, fg=_DIM, font=("Helvetica", 11),
            justify=tk.CENTER,
        ).pack(expand=True, pady=60)

    def _show_comparison_computing(self):
        """Clear the comparison panel and show a 'computing' message while the
        sklearn comparison runs in the background."""
        for w in self._comp_inner.winfo_children():
            w.destroy()
        tk.Label(
            self._comp_inner,
            text="Computing comparison…\n(training CPU HBOS, Isolation Forest, One-Class SVM)",
            bg=_BG, fg=_DIM, font=("Helvetica", 11), justify=tk.CENTER,
        ).pack(expand=True, pady=60)

    def _comparison_worker(self, gen: int):
        import sys
        t0 = time.monotonic()
        print(f"[comp] start gen={gen} preds={len(self._hbos_predictions)} "
              f"train={len(self._train_rows_cache)} test={len(self._stream_rows)}",
              file=sys.stderr, flush=True)
        try:
            # Pass the live calib percentile so the CPU-HBOS replica thresholds
            # at the SAME percentile as the FPGA. This re-runs on every detect,
            # so changing the threshold (Apply Config → re-detect) re-calibrates
            # CPU HBOS to track the live detection.
            try:
                calib_pct = float(self._calib_var.get())
            except (TypeError, ValueError):
                calib_pct = None
            results = run_comparison(
                self._train_rows_cache,
                self._stream_rows,
                self._hbos_predictions,
                calib_percentile=calib_pct,
            )
        except Exception as exc:
            import traceback
            traceback.print_exc()
            results = {"error": str(exc)}
        print(f"[comp] done gen={gen} in {time.monotonic()-t0:.2f}s "
              f"keys={list(results)}", file=sys.stderr, flush=True)
        # Tag with the run generation so a stale result from a superseded run
        # (rapid rerun) is discarded instead of overwriting the current panel.
        self._comp_q.put((gen, results))  # thread-safe: no tkinter call here

    def _poll_comp_queue(self):
        try:
            gen, results = self._comp_q.get_nowait()
            import sys
            print(f"[comp] deliver gen={gen} cur_gen={self._comp_gen} page={self._page}",
                  file=sys.stderr, flush=True)
            if gen != self._comp_gen:
                return  # stale result from a superseded run; drop it
            self._comp_results = results
            if self._page == 2:
                self._render_comparison(results)
            return  # done, stop polling
        except queue.Empty:
            pass
        self.after(200, self._poll_comp_queue)

    def _on_comparison_done(self, results: dict):
        if self._page == 2:
            self._render_comparison(results)

    def _render_comparison(self, results: dict):
        bg = _BG
        for w in self._comp_inner.winfo_children():
            w.destroy()

        if "error" in results:
            tk.Label(self._comp_inner, text=f"Comparison error:\n{results['error']}",
                     bg=bg, fg=_RED, font=("Helvetica", 10),
                     justify=tk.CENTER).pack(pady=40)
            return

        ds = results.get("dataset", {})

        # ── header strip ─────────────────────────────────────────────────────
        hdr = tk.Frame(self._comp_inner, bg=_BG3, padx=16, pady=10)
        hdr.pack(fill=tk.X, padx=0, pady=0)

        tk.Label(hdr, text="Algorithm Comparison", bg=_BG3, fg=_FG,
                 font=("Helvetica", 14, "bold")).pack(side=tk.LEFT, padx=(0, 24))

        for label, value in [
            ("Dataset",   f"{ds.get('total', 0):,} samples"),
            ("Anomalies", f"{ds.get('total_anomalies', 0):,} "
                          f"({ds.get('anomaly_rate_pct', 0):.1f}%)"),
        ]:
            pill = tk.Frame(hdr, bg="#2a2a30", padx=10, pady=4)
            pill.pack(side=tk.LEFT, padx=6)
            tk.Label(pill, text=label.upper(), bg="#2a2a30", fg=_DIM,
                     font=("Helvetica", 7)).pack(anchor=tk.W)
            tk.Label(pill, text=value, bg="#2a2a30", fg=_FG,
                     font=("Courier", 10, "bold")).pack(anchor=tk.W)

        if not results.get("sklearn_available", True):
            tk.Label(hdr, text="⚠  scikit-learn not installed — CPU results unavailable",
                     bg=_BG3, fg=_ORANGE, font=("Helvetica", 9)).pack(
                         side=tk.RIGHT, padx=12)

        tk.Label(self._comp_inner,
                 text="CPU·TUNED baselines are shown at their best-case (label-optimal) "
                      "threshold — an upper bound; FPGA HBOS+ is the actual deployed result.",
                 bg=bg, fg=_DIM, font=("Helvetica", 8), anchor=tk.W,
                 justify=tk.LEFT).pack(fill=tk.X, padx=16, pady=(6, 0))

        ttk.Separator(self._comp_inner, orient=tk.HORIZONTAL).pack(fill=tk.X)

        # ── algorithm columns ─────────────────────────────────────────────────
        cards_frame = tk.Frame(self._comp_inner, bg=bg)
        cards_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=14)
        algo_specs = [
            ("FPGA  HBOS+",       results.get("hbos"),     "#4fc3f7", "HARDWARE"),
            ("CPU  HBOS (best F1)",             results.get("hbos_cpu"), "#a5d6a7", "CPU·TUNED"),
            ("Isolation Forest (best F1)",      results.get("iforest"), "#80deea", "CPU·TUNED"),
            ("One-Class SVM (tuned)",           results.get("ocsvm"),   "#ce93d8", "CPU·TUNED"),
            ("k-NN (supervised)", results.get("knn"),      "#ffb74d", "SUPERVISED"),
        ]
        for col in range(len(algo_specs)):
            cards_frame.columnconfigure(col, weight=1, uniform="col")

        for col, (title, metrics, accent, badge) in enumerate(algo_specs):
            self._build_algo_card(cards_frame, col, title, metrics, accent, badge)

    # ── metric bar card ───────────────────────────────────────────────────────

    def _build_algo_card(self, parent, col: int, title: str,
                         metrics: "dict | None", accent: str, badge: str):
        bg   = _BG2
        card = tk.Frame(parent, bg=bg, padx=0, pady=0,
                        highlightbackground="#3a3a48",
                        highlightthickness=1)
        card.grid(row=0, column=col, sticky="nsew", padx=8, pady=4)

        # ── card header ───────────────────────────────────────────────────────
        chdr = tk.Frame(card, bg=accent, padx=12, pady=8)
        chdr.pack(fill=tk.X)

        tk.Label(chdr, text=title, bg=accent, fg="#0d0d0d",
                 font=("Helvetica", 11, "bold")).pack(side=tk.LEFT)
        tk.Label(chdr, text=badge, bg=accent, fg="#0d0d0d",
                 font=("Helvetica", 7, "bold")).pack(side=tk.RIGHT, pady=2)

        body = tk.Frame(card, bg=bg, padx=14, pady=10)
        body.pack(fill=tk.BOTH, expand=True)

        if metrics is None:
            tk.Label(body, text="Not available\n(scikit-learn missing)",
                     bg=bg, fg=_DIM, font=("Helvetica", 9),
                     justify=tk.CENTER).pack(pady=20)
            return

        # Degenerate result (e.g. supervised model with no anomalies to learn):
        # a dict carrying only a "note" and no metric fields.
        if "f1" not in metrics:
            tk.Label(body, text="Not applicable", bg=bg, fg=_ORANGE,
                     font=("Helvetica", 10, "bold"), justify=tk.CENTER).pack(pady=(20, 6))
            tk.Label(body, text=metrics.get("note", ""), bg=bg, fg=_DIM,
                     font=("Helvetica", 8), wraplength=150,
                     justify=tk.CENTER).pack(pady=(0, 20))
            return

        # ── main metric bars ──────────────────────────────────────────────────
        bar_metrics = [
            ("F1 Score",   metrics["f1"],        "#4ec94e"),
            ("Recall",     metrics["recall"],     "#4fc3f7"),
            ("Precision",  metrics["precision"],  "#80deea"),
            ("Accuracy",   metrics["accuracy"],   "#ce93d8"),
        ]
        for label, value, bar_color in bar_metrics:
            self._metric_bar_row(body, bg, label, value, bar_color)

        ttk.Separator(body, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)

        # ── count pills ───────────────────────────────────────────────────────
        counts = tk.Frame(body, bg=bg)
        counts.pack(fill=tk.X)
        counts.columnconfigure(0, weight=1)
        counts.columnconfigure(1, weight=1)

        self._count_pill(counts, 0, 0, "Caught",
                         metrics["caught"], _GREEN)
        self._count_pill(counts, 0, 1, "False Alarms",
                         str(metrics["false_alarms"]), _RED)
        self._count_pill(counts, 1, 0, "True Pos",
                         str(metrics["tp"]), _GREEN)
        self._count_pill(counts, 1, 1, "False Neg",
                         str(metrics["fn"]), _ORANGE)
        self._count_pill(counts, 2, 0, "True Neg",
                         str(metrics["tn"]), _DIM)
        self._count_pill(counts, 2, 1, "False Pos",
                         str(metrics["fp"]), _RED)

        # ── timing (CPU only) ─────────────────────────────────────────────────
        fit_ms     = metrics.get("fit_ms")
        predict_ms = metrics.get("predict_ms")
        if fit_ms is not None:
            ttk.Separator(body, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(8, 4))
            timing = tk.Frame(body, bg=bg)
            timing.pack(fill=tk.X)
            # predict_ms is None for the single-pass fit_predict path (iForest):
            # show one combined row instead of a bogus "None ms".
            if predict_ms is None:
                rows = [("Fit+detect", f"{fit_ms} ms")]
            else:
                rows = [("Fit", f"{fit_ms} ms"), ("Predict", f"{predict_ms} ms")]
            for label, val in rows:
                row = tk.Frame(timing, bg=bg)
                row.pack(fill=tk.X, pady=1)
                tk.Label(row, text=label, bg=bg, fg=_DIM,
                         font=("Helvetica", 8), width=9, anchor=tk.W).pack(side=tk.LEFT)
                tk.Label(row, text=val, bg=bg, fg=_FG,
                         font=("Courier", 9, "bold"), anchor=tk.E).pack(
                             side=tk.RIGHT)
        else:
            ttk.Separator(body, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=(8, 4))
            timing = tk.Frame(body, bg=bg)
            timing.pack(fill=tk.X)
            row = tk.Frame(timing, bg=bg)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text="Platform", bg=bg, fg=_DIM,
                     font=("Helvetica", 8), width=9, anchor=tk.W).pack(side=tk.LEFT)
            tk.Label(row, text="FPGA @ real-time", bg=bg, fg=_FG,
                     font=("Courier", 9, "bold"), anchor=tk.E).pack(side=tk.RIGHT)

    @staticmethod
    def _metric_bar_row(parent, bg: str, label: str, value: float, bar_color: str):
        """Horizontal metric row: label  ██████░░░░ 0.87"""
        row = tk.Frame(parent, bg=bg)
        row.pack(fill=tk.X, pady=3)

        tk.Label(row, text=label, bg=bg, fg=_DIM,
                 font=("Helvetica", 8), width=10, anchor=tk.W).pack(side=tk.LEFT)

        # Bar canvas
        BAR_W, BAR_H = 120, 10
        cv = tk.Canvas(row, width=BAR_W, height=BAR_H,
                       bg="#1a1a22", bd=0, highlightthickness=0)
        cv.pack(side=tk.LEFT, padx=(4, 6))
        fill_w = max(2, int(BAR_W * value))
        cv.create_rectangle(0, 0, fill_w, BAR_H, fill=bar_color, outline="")

        pct = f"{value:.1%}"
        tk.Label(row, text=pct, bg=bg, fg=_FG,
                 font=("Courier", 9, "bold"), width=6, anchor=tk.E).pack(side=tk.LEFT)

    @staticmethod
    def _count_pill(parent, grid_row: int, grid_col: int,
                    label: str, value: str, fg: str):
        f = tk.Frame(parent, bg=_BG3, padx=8, pady=5)
        f.grid(row=grid_row, column=grid_col, padx=3, pady=3, sticky="ew")
        tk.Label(f, text=label.upper(), bg=_BG3, fg=_DIM,
                 font=("Helvetica", 6, "bold")).pack(anchor=tk.W)
        tk.Label(f, text=value, bg=_BG3, fg=fg,
                 font=("Courier", 12, "bold")).pack(anchor=tk.W)

    # ── histogram page ────────────────────────────────────────────────────────

    def _build_hist_page(self):
        outer = self._hist_frame

        hscroll = ttk.Scrollbar(outer, orient=tk.HORIZONTAL)
        hscroll.pack(side=tk.BOTTOM, fill=tk.X)

        scroll_cv = tk.Canvas(outer, bg=_BG, highlightthickness=0,
                               xscrollcommand=hscroll.set)
        scroll_cv.pack(fill=tk.BOTH, expand=True)
        hscroll.config(command=scroll_cv.xview)
        self._hist_scroll_cv = scroll_cv

        self._hist_inner = tk.Frame(scroll_cv, bg=_BG)
        scroll_cv.create_window((0, 0), window=self._hist_inner, anchor="nw")
        self._hist_inner.bind("<Configure>",
                              lambda e: scroll_cv.config(scrollregion=scroll_cv.bbox("all")))

        def _on_wheel(event):
            if event.num == 4:
                scroll_cv.xview_scroll(-1, "units")
            elif event.num == 5:
                scroll_cv.xview_scroll(1, "units")
            else:
                scroll_cv.xview_scroll(-1 * (event.delta // 120), "units")
        scroll_cv.bind("<MouseWheel>", _on_wheel)
        scroll_cv.bind("<Button-4>", _on_wheel)
        scroll_cv.bind("<Button-5>", _on_wheel)

        self._hist_canvas = None
        self._rebuild_hist_figure(NR_SLOTS)

    def _rebuild_hist_figure(self, n: int) -> None:
        """Destroy the old histogram figure and create one with n columns."""
        if self._hist_canvas is not None:
            self._hist_canvas.get_tk_widget().destroy()
            plt.close(self._hist_fig)
            self._hist_canvas = None
            self._hist_fig = None
            gc.collect()  # collect PhotoImage/Variable objects here, not in background thread

        self._hist_fig, axes_raw = plt.subplots(1, n, figsize=(4.8 * n, 4.6))
        self._hist_axes = [axes_raw] if n == 1 else list(axes_raw)
        self._hist_fig.patch.set_facecolor(_BG)
        self._hist_fig.subplots_adjust(
            left=0.06, right=0.97, top=0.88, bottom=0.13, wspace=0.38
        )
        for ax in self._hist_axes:
            ax.set_facecolor(_BG2)
            ax.tick_params(colors=_DIM, labelsize=7, length=2)
            for sp in ax.spines.values():
                sp.set_edgecolor("#3a3a40")
                sp.set_linewidth(0.7)

        hist_canvas = FigureCanvasTkAgg(self._hist_fig, master=self._hist_inner)
        hist_canvas.get_tk_widget().pack(side=tk.LEFT)
        hist_canvas.get_tk_widget().configure(bg=_BG, highlightthickness=0)
        self._hist_canvas = hist_canvas

    def _render_histograms(self):
        active = self._active_indices
        for slot, ax in enumerate(self._hist_axes):
            ax.clear()
            ax.set_facecolor(_BG2)
            for sp in ax.spines.values():
                sp.set_edgecolor("#3a3a40")
                sp.set_linewidth(0.7)

            if slot >= len(active):
                ax.set_title("—", color="#444450", fontsize=9, pad=6)
                continue

            csv_col = active[slot]
            label   = SENSOR_LABELS[csv_col]
            col     = SENSOR_COLORS[csv_col]

            if self._train_hist_data is None or slot >= len(self._train_hist_data):
                ax.text(0.5, 0.5, "No training data\npress Train first",
                        transform=ax.transAxes, ha="center", va="center",
                        color=_DIM, fontsize=8, linespacing=1.8)
                ax.set_title(label, color=_FG, fontsize=9, pad=6, fontweight="bold")
                continue

            values = self._train_hist_data[slot]

            n_bins = min(40, max(10, len(values) // 10))
            counts, edges = np.histogram(values, bins=n_bins)
            w = edges[1] - edges[0]

            # Kaggle-style bars
            ax.bar(edges[:-1], counts, width=w * 0.92, align="edge",
                   color=col, alpha=0.68, edgecolor=_BG, linewidth=0.5)

            # KDE overlay — numpy-only Gaussian KDE (Scott's rule)
            try:
                n_v, std_v = len(values), values.std()
                if n_v >= 2 and std_v > 0:
                    h = 1.06 * std_v * n_v ** (-0.2)
                    v_samp = (values if n_v <= 4000
                              else values[np.random.choice(n_v, 4000, replace=False)])
                    xs = np.linspace(values.min(), values.max(), 400)
                    diff = (xs[:, None] - v_samp[None, :]) / h
                    density = (np.exp(-0.5 * diff ** 2).sum(axis=1)
                               / (len(v_samp) * h * np.sqrt(2 * np.pi)))
                    ax.plot(xs, density * n_v * w,
                            color=col, linewidth=1.8, alpha=0.95)
            except Exception:
                pass

            # Mean reference line
            mu = float(values.mean())
            ax.axvline(mu, color="#ff5252", linewidth=1.1,
                       linestyle="--", alpha=0.85)

            # Stats box — top-right, monospaced, like Kaggle
            n_val = len(values)
            sigma = float(values.std())
            ax.text(
                0.97, 0.97,
                f"n     {n_val}\n"
                f"mean  {mu:.2f}\n"
                f"std   {sigma:.2f}\n"
                f"min   {float(values.min()):.2f}\n"
                f"max   {float(values.max()):.2f}",
                transform=ax.transAxes, fontsize=6.5, color=_DIM,
                va="top", ha="right", fontfamily="monospace", linespacing=1.7,
                bbox=dict(boxstyle="round,pad=0.3", facecolor=_BG3,
                          edgecolor="#3a3a40", alpha=0.85),
            )

            ax.set_title(label, color=_FG, fontsize=9,
                         pad=6, fontweight="bold")
            ax.set_xlabel("value", color=_DIM, fontsize=7)
            ax.set_ylabel("count", color=_DIM, fontsize=7)
            ax.tick_params(colors=_DIM, labelsize=7, length=2)
            ax.grid(True, color="#333338", linewidth=0.35,
                    linestyle="--", alpha=0.5)

        self._hist_fig.canvas.draw_idle()
        self._hist_scroll_cv.update_idletasks()
        self._hist_scroll_cv.config(scrollregion=self._hist_scroll_cv.bbox("all"))

    # ── cleanup ───────────────────────────────────────────────────────────────

    def _gc_tick(self):
        # Main-thread cyclic GC: reclaims Tk/matplotlib reference cycles safely
        # (off-thread finalization is what aborts the process). Reschedules
        # itself; cheap because little cyclic garbage accumulates between ticks.
        gc.collect()
        self.after(2000, self._gc_tick)

    def _on_close(self):
        # HARD-EXIT, and do NOT call self.destroy()/plt.close first. Tk teardown
        # while a daemon thread (the sklearn comparison, or stream tx/rx) is
        # still alive panics with "Tcl_AsyncDelete: async handler deleted by the
        # wrong thread" and SIGABRTs before we could exit. os._exit() is a direct
        # _exit(2) syscall: it terminates immediately with no Python finalizers
        # and no Tcl teardown, so the OS just reaps the process and closes the
        # window — clean regardless of what the background threads are doing.
        self._cancel.set()
        try:
            if self._client:
                self._client.close()
        except Exception:
            pass
        os._exit(0)


def main():
    import argparse
    ap = argparse.ArgumentParser(description="FPGA Live Stream Viewer")
    ap.add_argument(
        "--mock", action="store_true",
        help="Run with a mock FPGA (no hardware required; speeds up timing for quick test)"
    )
    args = ap.parse_args()

    if args.mock:
        from . import train_session as _ts
        _ts.POST_CALIB_WAIT  = 0.3
        _ts.LATCH_SETTLE     = 0.2
        _ts.DUMP_CALIB_LATCH = 8
        _ts.POLL_GAP         = 0.01

    app = StreamViewer(mock=args.mock)
    app.mainloop()


if __name__ == "__main__":
    main()
