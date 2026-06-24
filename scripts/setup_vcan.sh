#!/usr/bin/env bash
# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      setup_vcan.sh
# @brief     Create and bring up the virtual CAN (vcan0) on the host
#
# @date      2026-06-24 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#
# Run once on the host. Create the virtual CAN (vcan0) and bring it up.
#
# modprobe vcan loads into the host kernel (the container shares the kernel); the vcan0
# interface lives in the network namespace, so the container must share it via
# network_mode: host to see it. If the host creates/brings it up beforehand, the
# container needs no NET_ADMIN (it just opens a socket on the already-up interface).
#
# Idempotent: safe to run multiple times.
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
