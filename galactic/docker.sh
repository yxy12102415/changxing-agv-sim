#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

MODE="${1:-shell}"
if [ "$#" -gt 0 ]; then
  shift
fi

if ! docker info >/dev/null 2>&1 && getent group docker | awk -F: -v user="${USER}" '
  $1 == "docker" {
    split($4, members, ",")
    for (i in members) {
      if (members[i] == user) {
        found = 1
      }
    }
  }
  END { exit found ? 0 : 1 }
'; then
  exec sg docker -c "$(printf "%q " "$0" "$MODE" "$@")"
fi

exec ./galactic/tools/galactic_docker.sh "$MODE" "$@"
