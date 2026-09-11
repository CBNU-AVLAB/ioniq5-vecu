#!/usr/bin/env bash
# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      setdown_vcan.sh
# @brief     Bring down and remove the virtual CAN (vcan0) on the host
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

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
