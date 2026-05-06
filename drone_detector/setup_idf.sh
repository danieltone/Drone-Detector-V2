#!/usr/bin/env bash
# ==========================================================================
# setup_idf.sh  —  Install ESP-IDF v6.1 with ESP32-C5 support
# Run this ONCE before building the drone detector firmware.
#
# What it does:
#   1. Clones ESP-IDF v6.1 into ~/esp/esp-idf
#   2. Installs tools for the esp32c5 target
#   3. Prints the "source" command you need to activate the environment
#
# Usage:
#   chmod +x setup_idf.sh
#   ./setup_idf.sh
# ==========================================================================
set -e

IDF_DIR="$HOME/esp/esp-idf"
IDF_TAG="v6.0.1"

echo "======================================================"
echo " ESP-IDF $IDF_TAG Setup for ESP32-C5 Drone Detector"
echo "======================================================"

# ── 1. Clone ESP-IDF ──────────────────────────────────────────────────────
if [ -d "$IDF_DIR" ]; then
    echo "[OK] ESP-IDF already cloned at $IDF_DIR"
    pushd "$IDF_DIR" >/dev/null
    CURRENT_TAG=$(git describe --tags 2>/dev/null || git rev-parse --short HEAD 2>/dev/null || echo "unknown")
    echo "[INFO] Current version: $CURRENT_TAG"
    popd >/dev/null
else
    echo "[*] Cloning ESP-IDF $IDF_TAG (shallow clone, ~500 MB)..."
    mkdir -p "$HOME/esp"
    git clone --depth=1 --branch "$IDF_TAG" --recurse-submodules \
        --shallow-submodules \
        https://github.com/espressif/esp-idf.git "$IDF_DIR"
    echo "[OK] ESP-IDF cloned."
fi

# ── 2. Install tools for esp32c5 ─────────────────────────────────────────
echo ""
echo "[*] Installing tools for esp32c5 target..."
cd "$IDF_DIR"
./install.sh esp32c5

# ── 3. Done ───────────────────────────────────────────────────────────────
echo ""
echo "======================================================"
echo " Setup complete!"
echo ""
echo " To activate the ESP-IDF environment, run:"
echo "   source ~/esp/esp-idf/export.sh"
echo ""
echo " Then build the drone detector:"
echo "   cd $(dirname "$(realpath "$0")")"
echo "   idf.py set-target esp32c5"
echo "   idf.py build"
echo "======================================================"
