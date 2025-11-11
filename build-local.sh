#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ensure_docker() {
  if docker info >/dev/null 2>&1; then
    return
  fi

  if ! command -v colima >/dev/null 2>&1; then
    echo "Docker daemon is unavailable and Colima is not installed." >&2
    exit 1
  fi

  echo "Docker daemon unavailable, attempting to start Colima..."
  colima start
}

main() {
  ensure_docker
  (cd "$REPO_ROOT/local-build" && docker-compose run --rm builder)
}

main "$@"
