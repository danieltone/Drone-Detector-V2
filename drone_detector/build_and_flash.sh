#!/usr/bin/env bash
# ==========================================================================
# build_and_flash.sh  —  Build, flash, and monitor the drone detector
#
# Usage:
#   source ~/esp/esp-idf/export.sh   # activate IDF environment first
#   chmod +x build_and_flash.sh
#   ./build_and_flash.sh
#
# Or with a custom port:
#   PORT=/dev/ttyUSB0 ./build_and_flash.sh
# ==========================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PORT="${PORT:-/dev/ttyACM0}"
BAUD="${BAUD:-115200}"

echo "======================================================"
echo " ESP32-C5 Drone Detector — Build & Flash"
echo " Port : $PORT"
echo " Baud : $BAUD"
echo "======================================================"

# ── Sanity checks ─────────────────────────────────────────────────────────
if ! command -v idf.py &>/dev/null; then
    echo ""
    echo "[ERROR] idf.py not found in PATH."
    echo "  Run:  source ~/esp/esp-idf/export.sh"
    echo "  Then re-run this script."
    exit 1
fi

if [ ! -c "$PORT" ]; then
    echo ""
    echo "[ERROR] Serial port $PORT not found."
    echo "  Available ports:"
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  (none)"
    echo "  Set:  PORT=/dev/ttyXXX ./build_and_flash.sh"
    exit 1
fi

# Add user to 'dialout' and 'plugdev' groups if needed
if ! groups | grep -qE "dialout|plugdev"; then
    echo "[WARN] You may need port permissions:  sudo usermod -aG plugdev,dialout $USER"
fi

cd "$SCRIPT_DIR"

# ── Set target (idempotent) ───────────────────────────────────────────────
echo ""
echo "[*] Setting target to esp32c5..."
idf.py set-target esp32c5

# ── Build ─────────────────────────────────────────────────────────────────
echo ""
echo "[*] Building firmware..."
idf.py build

# ── Flash ─────────────────────────────────────────────────────────────────
echo ""
echo "[*] Flashing to $PORT..."
idf.py -p "$PORT" flash

# ── Monitor ───────────────────────────────────────────────────────────────
echo ""
echo "[*] Opening serial monitor (Ctrl+] to exit)..."
idf.py -p "$PORT" -b "$BAUD" monitor
