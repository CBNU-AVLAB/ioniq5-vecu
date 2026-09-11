#!/usr/bin/env bash
# @copyright (c) 2026 Autonomous Vehicle Laboratory, Chungbuk National University
#            SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#
# @file      run_docker.sh
# @brief     Build + run the vECU container, and remove it on exit
#
# @date      2026-07-08 created by Junhyeok Seo (jun2342@chungbuk.ac.kr)

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
