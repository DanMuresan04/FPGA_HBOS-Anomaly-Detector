"""
UDP transport layer for the FPGA HBOS anomaly-detection pipeline.

Packet format (22 bytes total):
  [0-1]   big-endian sequence counter  (stripped by UDP RX FSM before HLS sees it)
  [2-17]  4 × int32 LE sensor values
  [18]    opcode (1 byte)
  [19]    tlast  (1 byte)
  [20]    0xA5   frame magic lo
  [21]    0x5A   frame magic hi

FPGA reply (18-byte UDP payload from udp_tx_fsm):
  [0]     0x00  (seq high byte, always 0)
  [1]     result byte  → 0x00 normal, 0x01 anomaly, 0xFE banner, 0xFF terminator
  [2-17]  0x00  padding
"""

import select
import socket
import struct
import subprocess
import threading

FPGA_IP    = "10.0.0.2"
FPGA_PORT  = 12345
PC_IP      = "10.0.0.1"
PC_PORT    = 12345
FPGA_MAC   = "00:18:3e:04:f7:0a"
DEFAULT_IFACE = "enp56s0"

OP_TRAIN     = 0
OP_CALIBRATE = 1
OP_DETECT    = 2
OP_DUMP      = 3

RESULT_NAMES = {
    0x00: "normal",
    0x01: "anomaly",
    0xFE: "telemetry_banner",
    0xFF: "dump_ack",
}


class FpgaClient:
    """Thread-safe UDP client for the FPGA pipeline.

    One instance is created per application session and reused across
    training and streaming phases (the socket stays bound throughout).
    """

    def __init__(self):
        self._seq = 0
        self._lock = threading.Lock()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._sock.bind((PC_IP, PC_PORT))
        except OSError as exc:
            if exc.errno == 98:
                raise RuntimeError(
                    f"UDP {PC_IP}:{PC_PORT} already in use — "
                    f"stop the other listener first (lsof -i UDP:{PC_PORT})"
                ) from exc
            raise

    # ── packet building ──────────────────────────────────────────────────────

    def pack_packet(
        self,
        s0: int,
        s1: int,
        s2: int,
        s3: int,
        opcode: int,
        tlast: int,
    ) -> bytes:
        """Build a 22-byte UDP payload with rolling sequence prefix."""
        with self._lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
        return (
            struct.pack(">H", seq)
            + struct.pack("<iiiiBB", int(s0), int(s1), int(s2), int(s3), opcode, tlast)
            + bytes((0xA5, 0x5A))
        )

    # ── send / recv ──────────────────────────────────────────────────────────

    def send(self, payload: bytes) -> None:
        self._sock.sendto(payload, (FPGA_IP, FPGA_PORT))

    def recv(self, timeout: float = 1.0) -> tuple:
        """Blocking recv with timeout.  Returns (byte_val, raw_data) or (None, None)."""
        self._sock.settimeout(timeout)
        try:
            data, _ = self._sock.recvfrom(65535)
            return self.decode(data)["result"], data
        except socket.timeout:
            return None, None

    def try_recv(self) -> tuple:
        """Fully non-blocking recv via select(0).  Returns (byte_val, raw) or (None, None)."""
        r, _, _ = select.select([self._sock], [], [], 0)
        if not r:
            return None, None
        try:
            data, _ = self._sock.recvfrom(65535)
            return self.decode(data)["result"], data
        except OSError:
            return None, None

    def drain(self, timeout: float = 0.5) -> int:
        """Discard stale datagrams; return count drained."""
        import time
        self._sock.settimeout(timeout)
        count = 0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                self._sock.recvfrom(65535)
                count += 1
            except socket.timeout:
                break
        return count

    # ── decode ───────────────────────────────────────────────────────────────

    @staticmethod
    def decode(data: bytes) -> dict:
        """Parse FPGA UDP reply.  Result byte is at offset 1."""
        if len(data) < 2:
            return {"result": None, "name": "short"}
        result = data[1]
        return {"result": result, "name": RESULT_NAMES.get(result, f"0x{result:02X}")}

    # ── cleanup ──────────────────────────────────────────────────────────────

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    # ── network preflight ─────────────────────────────────────────────────────

    @staticmethod
    def network_warnings(iface: str = DEFAULT_IFACE) -> list:
        """Return human-readable warning strings for network misconfigurations."""
        warnings = []
        try:
            out = subprocess.check_output(
                ["ip", "-br", "link", "show", iface],
                text=True,
                stderr=subprocess.DEVNULL,
            )
            if "NO-CARRIER" in out or "state DOWN" in out:
                warnings.append(
                    f"{iface}: no link — cable disconnected or FPGA unpowered?"
                )
        except subprocess.CalledProcessError:
            warnings.append(f"Interface '{iface}' not found")

        try:
            addrs = subprocess.check_output(
                ["ip", "-4", "-o", "addr", "show"],
                text=True,
                stderr=subprocess.DEVNULL,
            )
            if f"inet {PC_IP}/" not in addrs:
                warnings.append(
                    f"{PC_IP} not assigned to any interface. Fix:\n"
                    f"  sudo ip addr add {PC_IP}/24 dev {iface}"
                )
        except subprocess.CalledProcessError:
            pass

        try:
            neigh = subprocess.check_output(
                ["ip", "neigh", "show", FPGA_IP],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            if not neigh or "FAILED" in neigh or "INCOMPLETE" in neigh:
                warnings.append(
                    f"No static ARP for {FPGA_IP}. Fix:\n"
                    f"  sudo ip neigh replace {FPGA_IP} lladdr {FPGA_MAC}"
                    f" dev {iface} nud permanent"
                )
        except subprocess.CalledProcessError:
            warnings.append(f"Could not query ARP table for {FPGA_IP}")

        return warnings
