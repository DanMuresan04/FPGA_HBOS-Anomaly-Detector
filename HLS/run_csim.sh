#!/usr/bin/env bash
# Run HLS C-simulation for one kernel (fast, no Vivado bitstream).
# Usage: ./run_csim.sh packet_assembler|address_engine|hbos_top|detection_engine|full
#
# Requires vitis-run on PATH, or set VITIS_RUN, or source:
#   source /home/dan/Vivado/2025.2/Vitis/settings64.sh
set -euo pipefail
cd "$(dirname "$0")"

find_vitis_run() {
  if [[ -n "${VITIS_RUN:-}" && -x "${VITIS_RUN}" ]]; then
    echo "${VITIS_RUN}"
    return 0
  fi
  if command -v vitis-run >/dev/null 2>&1; then
    command -v vitis-run
    return 0
  fi
  local candidates=(
    "${XILINX_VITIS:-}/bin/vitis-run"
    "${HOME}/Vivado/2025.2/Vitis/bin/vitis-run"
    "/tools/Xilinx/Vitis/2025.2/bin/vitis-run"
    "/opt/Xilinx/Vitis/2025.2/bin/vitis-run"
  )
  local p
  for p in "${candidates[@]}"; do
  if [[ -n "$p" && -x "$p" ]]; then
      echo "$p"
      return 0
    fi
  done
  return 1
}

VITIS_RUN_BIN="$(find_vitis_run)" || {
  echo "ERROR: vitis-run not found." >&2
  echo "  source /home/dan/Vivado/2025.2/Vitis/settings64.sh" >&2
  echo "  or: export VITIS_RUN=/home/dan/Vivado/2025.2/Vitis/bin/vitis-run" >&2
  exit 127
}

TOP="${1:?usage: $0 packet_assembler|address_engine|hbos_top|detection_engine|full}"

COMMON=(
  syn.file=hbos_types.h
  syn.file=hbos_math.h
  syn.file=packet_assembler.cpp
  syn.file=address_engine.cpp
  syn.file=hbos_top.cpp
  syn.file=hbos_top.h
  syn.file=detection_engine.cpp
  syn.file=hls_test_stream.csv
)

case "$TOP" in
  packet_assembler)
    TB=tb_packet_assembler.cpp
    DISPLAY=packet_assembler
    ;;
  address_engine)
    TB=tb_address_engine.cpp
    DISPLAY=address_engine
    ;;
  hbos_top)
    TB=hls_tb.cpp
    DISPLAY=hbos_top
    ;;
  detection_engine)
    TB=tb_detection_engine.cpp
    DISPLAY=detection_engine
    ;;
  full)
    TB=hls_tb.cpp
    DISPLAY=training_engine
    TOP=hbos_top
    ;;
  *)
    echo "unknown top: $TOP" >&2
    exit 1
    ;;
esac

CFG="$(mktemp /tmp/hls_config_XXXXXX.cfg)"
trap 'rm -f "$CFG"' EXIT

{
  echo "part=xc7a100tcsg324-1"
  echo ""
  echo "[hls]"
  echo "flow_target=vivado"
  echo "package.output.format=ip_catalog"
  echo "package.output.syn=false"
  echo "clock=100Mhz"
  echo "tb.file=$TB"
  echo "syn.top=$TOP"
  echo "package.ip.display_name=$DISPLAY"
  for f in "${COMMON[@]}"; do echo "$f"; done
} >"$CFG"

WORK_DIR="${CSIM_WORK_DIR:-$(pwd)/csim_${TOP}}"
mkdir -p "$WORK_DIR"
echo "=== CSIM top=$TOP tb=$TB ==="
echo "    vitis-run=$VITIS_RUN_BIN"
echo "    work_dir=$WORK_DIR"
"$VITIS_RUN_BIN" --mode hls --csim --config "$CFG" --work_dir "$WORK_DIR"
