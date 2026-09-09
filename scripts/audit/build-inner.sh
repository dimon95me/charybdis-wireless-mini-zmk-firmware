#!/usr/bin/env bash
set -euo pipefail

BUNDLE="$1"
MODE="${2:-right}"
OUTPUT="$3"
IMAGE="zmkfirmware/zmk-build-arm:3.5-branch"
CACHE="/opt/zmk-builder/workspace/config-repo"
ROOT="$(mktemp -d /tmp/zmk-audit.XXXXXX)"
OUT="$ROOT/out"
mkdir -p "$ROOT/source" "$OUT"
tar -xzf "$BUNDLE" -C "$ROOT/source"
mkdir -p "$OUTPUT"
rm -f "$OUTPUT.tar.gz"

docker pull "$IMAGE" >/dev/null
IMAGE_DIGEST="$(docker image inspect --format '{{index .RepoDigests 0}}' "$IMAGE" 2>/dev/null || true)"
set +e
docker run --rm \
    -e AUDIT_MODE="$MODE" \
    -e AUDIT_OUTPUT=/out \
    -e AUDIT_IMAGE="$IMAGE" \
    -e AUDIT_IMAGE_DIGEST="$IMAGE_DIGEST" \
    -v "$ROOT/source:/config:ro" \
    -v "$CACHE:/dependency-cache:ro" \
    -v "$OUT:/out" \
    "$IMAGE" bash /config/scripts/audit/build-inner-docker.sh 2>&1 | tee "$OUTPUT.log"
DOCKER_STATUS="${PIPESTATUS[0]}"
set -e

cp "$OUTPUT.log" "$OUT/build.log"
if [ "$DOCKER_STATUS" -ne 0 ] || [ ! -f "$OUT/BUILD_OK" ]; then touch "$OUT/BUILD_FAILED"; fi
tar -czf "$OUTPUT.tar.gz" -C "$OUT" .
[ "$DOCKER_STATUS" -eq 0 ] && [ -f "$OUT/BUILD_OK" ]
