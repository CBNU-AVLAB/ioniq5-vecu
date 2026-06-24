#!/usr/bin/env bash
# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      setdown_vcan.sh
# @brief     Bring down and remove the virtual CAN (vcan0) on the host
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#
# Run on the host. Bring down and remove the virtual CAN (vcan0).
set -euo pipefail

DEV="${1:-vcan0}"
SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"

if ip link show "$DEV" &>/dev/null; then
    $SUDO ip link set down "$DEV"
    $SUDO ip link del dev "$DEV"
    echo "[ioniq5_vcan] '$DEV' removed"
else
    echo "[ioniq5_vcan] '$DEV' not found (already removed)"
fi
