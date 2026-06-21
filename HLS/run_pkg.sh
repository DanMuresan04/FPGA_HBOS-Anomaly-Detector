#!/usr/bin/env bash
# Synthesize + package ONE HLS kernel as an IP, using a per-top config generated
# on the fly. This exists because the single shared hls_config.cfg has one
# hand-edited syn.top: leaving it stale made "package hbos_top" actually build
# address_engine while labelling the IP hbos_top. Here syn.top and
# display_name are always derived from the argument, so they can never drift.
#
# Usage: ./run_pkg.sh <all|packet_assembler|address_engine|detection_engine|hbos_top> [K] [--deploy]
#   all      builds the three K-affected IPs in one go:
#            address_engine, detection_engine, hbos_top (all at the same K).
#   K (optional) overrides K_PARALLEL at build time, e.g.:
#       ./run_pkg.sh hbos_top            # uses K_PARALLEL default from hbos_types.h
#       ./run_pkg.sh hbos_top 4          # force fully-parallel
#       ./run_pkg.sh hbos_top 1          # force TDM/shared
#   --deploy (optional) after packaging, clears $IP_REPO/<top>/ and unzips the
#   fresh IP into it (replaces the by-hand copy+unzip). IP_REPO defaults to
#   /home/dan/HLS/IP_REPOSITORY.
#       ./run_pkg.sh all 4 --deploy      # rebuild + deploy all 3 at K=4 (board-consistent)
#       ./run_pkg.sh hbos_top 1 --deploy
#
# Requires vitis-run/v++ on PATH, or set VITIS_BIN, or source:
#   source /home/dan/Vivado/2025.2/Vitis/settings64.sh
set -euo pipefail
cd "$(dirname "$0")"
SRC="$(pwd)"

# This script runs under bash (shebang) even when launched from fish, so it can
# source the bash Vitis settings itself — the caller doesn't need to (and from
# fish, can't) source settings64.sh. Skip if v++ is already on PATH.
if ! command -v v++ >/dev/null 2>&1; then
  VITIS_SETTINGS="${VITIS_SETTINGS:-/home/dan/Vivado/2025.2/Vitis/settings64.sh}"
  # settings64.sh references unset vars (e.g. PYTHONPATH); relax nounset around it.
  if [[ -f "$VITIS_SETTINGS" ]]; then
    set +u; source "$VITIS_SETTINGS"; set -u
  fi
fi

find_vitis_bin() {
  if [[ -n "${VITIS_BIN:-}" && -x "${VITIS_BIN}/vitis-run" ]]; then
    echo "${VITIS_BIN}"; return 0
  fi
  if command -v vitis-run >/dev/null 2>&1; then
    dirname "$(command -v vitis-run)"; return 0
  fi
  local candidates=(
    "${XILINX_VITIS:-}/bin"
    "${HOME}/Vivado/2025.2/Vitis/bin"
    "/tools/Xilinx/Vitis/2025.2/bin"
    "/opt/Xilinx/Vitis/2025.2/bin"
  )
  local p
  for p in "${candidates[@]}"; do
    if [[ -n "$p" && -x "$p/vitis-run" ]]; then echo "$p"; return 0; fi
  done
  return 1
}

VBIN="$(find_vitis_bin)" || {
  echo "ERROR: vitis-run/v++ not found." >&2
  echo "  source /home/dan/Vivado/2025.2/Vitis/settings64.sh" >&2
  echo "  or: export VITIS_BIN=/home/dan/Vivado/2025.2/Vitis/bin" >&2
  exit 127
}

TOP="${1:?usage: $0 <all|packet_assembler|address_engine|detection_engine|hbos_top> [K] [--deploy]}"
shift
K=""
DEPLOY=0
for arg in "$@"; do
  case "$arg" in
    --deploy) DEPLOY=1 ;;
    ''|*[!0-9]*) echo "unknown argument: $arg" >&2; exit 1 ;;
    *) K="$arg" ;;
  esac
done
IP_REPO="${IP_REPO:-/home/dan/HLS/IP_REPOSITORY}"

# The three kernels whose RTL is parameterised by K_PARALLEL. packet_assembler
# is intentionally NOT here: it has no per-sensor K loop, so K never changes it.
# hbos_top + detection_engine MUST share the same K (they share the BD's
# histogram BRAMs); address_engine's interface is K-invariant but its internals
# follow K, so it's rebuilt too for consistency.
K_AFFECTED=(address_engine detection_engine hbos_top)

# Synthesize + package (and optionally deploy) one top with a freshly generated
# per-top config. Returns non-zero on failure so the 'all' loop can stop.
build_one() {
  local TOP="$1"
  local FILES
  case "$TOP" in
    packet_assembler) FILES=(hbos_types.h packet_assembler.cpp) ;;
    address_engine)   FILES=(hbos_types.h hbos_math.h address_engine.cpp) ;;
    detection_engine) FILES=(hbos_types.h hbos_math.h hbos_top.h detection_engine.cpp) ;;
    hbos_top)         FILES=(hbos_types.h hbos_math.h hbos_top.h hbos_top.cpp) ;;
    hbos_engine)      FILES=(hbos_types.h hbos_math.h hbos_engine.h hbos_engine.cpp) ;;
    *) echo "unknown top: $TOP" >&2; return 1 ;;
  esac

  local WORK="$SRC/${TOP}_ip"
  local CFG; CFG="$(mktemp /tmp/pkg_${TOP}_XXXXXX.cfg)"
  {
    echo "part=xc7a100tcsg324-1"
    echo ""
    echo "[hls]"
    echo "flow_target=vivado"
    echo "package.output.format=ip_catalog"
    echo "package.output.syn=false"
    echo "clock=100Mhz"
    echo "syn.top=$TOP"
    echo "package.ip.library=hls"
    echo "package.ip.display_name=$TOP"
    [[ -n "$K" ]] && echo "syn.cflags=-DK_PARALLEL=$K"
    # Absolute paths so the build is robust regardless of where v++ cds to.
    local f; for f in "${FILES[@]}"; do echo "syn.file=$SRC/$f"; done
  } >"$CFG"

  rm -rf "$WORK"
  echo "=== SYNTH + PACKAGE  top=$TOP  K=${K:-<default>}  work=$WORK ==="
  "$VBIN/v++" -c --mode hls --config "$CFG" --work_dir "$WORK"
  "$VBIN/vitis-run" --mode hls --package --config "$CFG" --work_dir "$WORK"
  rm -f "$CFG"
  echo "=== DONE: IP packaged for top=$TOP -> $WORK ==="

  if [[ "$DEPLOY" == "1" ]]; then
    local ZIP="$WORK/$TOP.zip"
    [[ -f "$ZIP" ]] || { echo "ERROR: expected IP archive not found: $ZIP" >&2; return 1; }
    local DEST="$IP_REPO/$TOP"
    echo "=== DEPLOY: $ZIP -> $DEST (clear + unzip) ==="
    rm -rf "$DEST"; mkdir -p "$DEST"
    unzip -o -q "$ZIP" -d "$DEST"
    [[ -f "$DEST/component.xml" ]] \
      && echo "=== DEPLOYED: $DEST (component.xml present) ===" \
      || { echo "ERROR: component.xml missing after unzip in $DEST" >&2; return 1; }
  fi
}

if [[ "$TOP" == "all" ]]; then
  echo "### Building all K-affected IPs: ${K_AFFECTED[*]}  (K=${K:-<default>}, deploy=$DEPLOY) ###"
  for t in "${K_AFFECTED[@]}"; do
    build_one "$t"
  done
  [[ "$DEPLOY" == "1" ]] && verb="built and deployed" || verb="built"
  echo "### ALL DONE: ${K_AFFECTED[*]} $verb at K=${K:-<default>} ###"
else
  build_one "$TOP"
fi
