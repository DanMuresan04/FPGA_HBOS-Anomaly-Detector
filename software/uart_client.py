"""
UART transport layer for the FPGA HBOS anomaly-detection pipeline.

Drop-in replacement for gui/fpga_client.FpgaClient — same method signatures,
UART serial instead of UDP.

TX packet (20 bytes):
  [0-15]  4 x int32 little-endian sensor values
  [16]    opcode  (0=train 1=calibrate 2=detect 3=dump)
  [17]    tlast   (1 = last sample in sequence)
  [18]    0xA5    frame magic lo
  [19]    0x5A    frame magic hi

RX reply (1 byte):
  [0]     result  -> 0x00 normal, 0x01 anomaly, 0xFE banner, 0xFF terminator
"""

import glob
import os
import platform
import serial
import sys
import struct
import threading
import time

DEFAULT_BAUD = 1_000_000


def find_nexys_uart_port() -> "str | None":
    """Auto-detect the Nexys A7 UART port (Linux only)."""
    by_id = sorted(glob.glob("/dev/serial/by-id/usb-Digilent_*-if01-port0"))
    if by_id:
        return os.path.realpath(by_id[0])
    for path in sorted(glob.glob("/dev/ttyUSB*")):
        dev = os.path.basename(path)
        try:
            with open(f"/sys/class/tty/{dev}/device/interface") as f:
                if f.read().strip() == "1":
                    return path
        except OSError:
            pass
    return None


if platform.system() == "Windows":
    DEFAULT_PORT = "COM5"
else:
    DEFAULT_PORT = find_nexys_uart_port() or "/dev/ttyUSB1"
REPLY_BYTES  = 1

OP_TRAIN     = 0
OP_CALIBRATE = 1
OP_DETECT    = 2
OP_DUMP      = 3
OP_CONFIG    = 4
OP_RESET     = 5

RESULT_NAMES = {
    0x00: "normal",
    0x01: "anomaly",
    0xFE: "telemetry_banner",
    0xFF: "dump_ack",
}


class UartFpgaClient:
    """Thread-safe UART client for the FPGA pipeline."""

    # UART is an in-order, lossless byte stream, so the host can blast detect
    # packets and collect verdicts by position without per-sample lock-step.
    # The stream worker checks this flag to enable the pipelined fast path.
    PIPELINED = True

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
        self._lock = threading.Lock()
        self._ser = serial.Serial(port, baud, timeout=0)
        self._set_low_latency(port)

    @staticmethod
    def _set_low_latency(port: str) -> None:
        """Best-effort: drop the FTDI latency timer 16ms->1ms so the 1-byte
        verdict replies aren't held by the chip's buffer-flush timer. Needs
        write access to the sysfs node (udev rule or root); skipped otherwise.
        """
        if platform.system() != "Linux":
            return
        dev  = os.path.basename(os.path.realpath(port))
        path = f"/sys/bus/usb-serial/devices/{dev}/latency_timer"
        try:
            with open(path, "w") as f:
                f.write("1")
        except OSError:
            pass
        try:
            with open(path) as f:
                val = f.read().strip()
            if val != "1":
                print(f"[uart] latency_timer={val}ms (couldn't lower; run once: "
                      f"echo 1 | sudo tee {path})", file=sys.stderr)
        except OSError:
            pass

    # -- packet building ------------------------------------------------------

    def pack_frame(self, values: list, active_count: int, opcode: int, tlast: int) -> bytes:
        """Count-prefixed variable frame (matches packet_assembler.cpp):
          [n_words][active_count][opcode][tlast][ n_words LE int32 ][0xA5][0x5A]
        n_words = len(values); the assembler waits for exactly that many ints."""
        vals = list(values)
        n = len(vals)
        out = bytes((n & 0xFF, int(active_count) & 0xFF, opcode & 0x07, tlast & 1))
        for v in vals:
            out += struct.pack("<i", int(float(v)))
        return out + bytes((0xA5, 0x5A))

    def pack_config_packet(self, weights: list, spike_penalty: int,
                           active_count: int = None) -> bytes:
        """OP_CONFIG frame: 16 weights packed LE into 4 int32 words + spike as a
        5th word.  `weights` = active-channel weights in order (channel 0..N-1)."""
        w = [int(x) & 0xFF for x in list(weights)[:16]]
        w += [0] * (16 - len(w))
        if active_count is None:
            active_count = len(list(weights))
        words = []
        for k in range(4):
            val = w[4*k] | (w[4*k+1] << 8) | (w[4*k+2] << 16) | (w[4*k+3] << 24)
            words.append(struct.unpack('<i', struct.pack('<I', val))[0])
        words.append(int(spike_penalty) & 0xFFFF)   # data[4] = spike_penalty
        return self.pack_frame(words, active_count, OP_CONFIG, 0)

    # -- send / recv ----------------------------------------------------------

    def send(self, payload: bytes) -> None:
        with self._lock:
            self._ser.write(payload)

    def send_sample(self, values: list, opcode: int, tlast: int) -> None:
        """Send one frame carrying exactly the active sensor values; the engine
        reads active_count = len(values) and gates the calc on the first N units."""
        vals = list(values)
        self.send(self.pack_frame(vals, len(vals), opcode, tlast))

    def send_control(self, opcode: int, active_count: int = 0) -> None:
        """Send a data-less control frame (RESET / DUMP / CALIB pump)."""
        self.send(self.pack_frame([], active_count, opcode, 0))

    def send_reset(self) -> None:
        """OP_RESET: flush all engine state so the next training run starts clean."""
        self.send_control(OP_RESET)

    def recv(self, timeout: float = 1.0) -> tuple:
        """Block until a 2-byte reply arrives or timeout expires.
        Returns (result_byte, raw_bytes) or (None, None).
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._ser.in_waiting >= REPLY_BYTES:
                    data = self._ser.read(REPLY_BYTES)
                    return self.decode(data)["result"], data
            time.sleep(0.001)
        return None, None

    def try_recv(self) -> tuple:
        """Non-blocking recv. Returns (result_byte, raw_bytes) or (None, None)."""
        with self._lock:
            if self._ser.in_waiting < REPLY_BYTES:
                return None, None
            data = self._ser.read(REPLY_BYTES)
        return self.decode(data)["result"], data

    def read_available(self) -> bytes:
        """Non-blocking bulk read: return all currently buffered RX bytes (one
        verdict byte per detect). Used by the pipelined stream path so the host
        drains replies in chunks instead of one round-trip per sample.
        """
        with self._lock:
            n = self._ser.in_waiting
            if n:
                return self._ser.read(n)
        return b""

    def drain(self, timeout: float = 0.5) -> int:
        """Discard buffered RX bytes. Returns number of bytes discarded."""
        count = 0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                n = self._ser.in_waiting
                if n == 0:
                    break
                self._ser.read(n)
            count += n
        return count

    # -- decode ---------------------------------------------------------------

    @staticmethod
    def decode(data: bytes) -> dict:
        """Parse FPGA UART reply. Single result byte."""
        if len(data) < 1:
            return {"result": None, "name": "short"}
        result = data[0]
        return {"result": result, "name": RESULT_NAMES.get(result, f"0x{result:02X}")}

    # -- cleanup --------------------------------------------------------------

    def close(self) -> None:
        # Hold the lock so we don't close the port out from under a concurrent
        # send/read on the TX or RX thread.
        with self._lock:
            try:
                self._ser.close()
            except Exception:
                pass


# -- smoke test ---------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="UART FPGA client smoke test")
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help="serial port (e.g. COM5 or /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--op", type=int, default=OP_DETECT,
                    help="opcode: 0=train 1=calibrate 2=detect 3=dump")
    args = ap.parse_args()

    port = args.port
    if not os.path.exists(port):
        detected = find_nexys_uart_port()
        if detected:
            print(f"Port {port!r} not found; using auto-detected {detected}")
            port = detected
        else:
            print(f"ERROR: port {port!r} not found and no Nexys UART detected.", file=sys.stderr)
            print("  Check: ls /dev/serial/by-id/usb-Digilent_*", file=sys.stderr)
            sys.exit(1)

    client = UartFpgaClient(port=port, baud=args.baud)
    print(f"Opened {port} @ {args.baud} baud")

    pkt = client.pack_frame([100, 200, 300, 400], 4, args.op, 1)
    print(f"TX {len(pkt)} bytes: {pkt.hex()}")
    client.send(pkt)

    result, raw = client.recv(timeout=2.0)
    if raw is None:
        print("RX timeout -- no reply from FPGA")
    else:
        info = client.decode(raw)
        print(f"RX {len(raw)} bytes: {raw.hex()} -> {info['name']} (0x{info['result']:02X})")

    client.close()
