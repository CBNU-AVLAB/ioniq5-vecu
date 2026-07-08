#!/usr/bin/env bash
# @copyright Chungbuk National University, Autonomous Vehicle Laboratory, 2026. All rights reserved.
#            Subject to limited distribution and restricted disclosure only.
#
# @file      run_docker.sh
# @brief     Build + run the vECU container, and remove it on exit
#
# @date      2026-07-08 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)
#
# Bring the vECU container up in the foreground. On Ctrl+C (or any exit) the
# container and its compose network are torn down (`docker compose down`), so no
# stopped `Exited` container is left behind.
#
# Prerequisite: bring up the virtual CAN on the host first.
#     ./scripts/setup_vcan.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE="$SCRIPT_DIR/../docker/docker-compose.yml"

# Always tear the container down when this script exits (Ctrl+C included).
cleanup() {
    echo "[ioniq5_vecu] removing container ..."
    docker compose -f "$COMPOSE" down
}
trap cleanup EXIT

docker compose -f "$COMPOSE" up --build "$@"
