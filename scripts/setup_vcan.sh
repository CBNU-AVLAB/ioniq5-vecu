#!/usr/bin/env bash
# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      setup_vcan.sh
# @brief     Create and bring up the virtual CAN (vcan0) on the host
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

set -euo pipefail

DEV="${1:-vcan0}"
SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"

$SUDO modprobe vcan

if ! ip link show "$DEV" &>/dev/null; then
    $SUDO ip link add dev "$DEV" type vcan
fi
$SUDO ip link set up "$DEV"

echo "[ioniq5_vcan] '$DEV' up:"
ip -brief link show "$DEV"
