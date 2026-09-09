#!/usr/bin/env bash
set -uo pipefail

MODE="$AUDIT_MODE"
STAGE="$(mktemp -d /tmp/zmk-audit-stage.XXXXXX)"
CONFIG="$STAGE/config"
mkdir -p "$CONFIG" "$STAGE/zmk" "$AUDIT_OUTPUT"
cp -a /config/. "$CONFIG/"
if [ -d /dependency-cache/zmk ]; then
    cp -a /dependency-cache/zmk/. "$STAGE/zmk/"
else
    echo "missing dependency cache project: /dependency-cache/zmk" >&2
    exit 2
fi
if [ "$MODE" = left ]; then
    TARGETS="left"
elif [ "$MODE" = right ]; then
    TARGETS="right"
else
    TARGETS="left right"
fi
export ZMK_EXTRA_MODULES=/config/vendor/pmw3610-driver
cd "$STAGE"
if [ -d /dependency-cache/.west ]; then
    cp -a /dependency-cache/.west "$STAGE/.west"
else
    echo "missing west metadata: /dependency-cache/.west" >&2
    exit 2
fi
ln -s /dependency-cache/zephyr "$STAGE/zephyr"
if [ -d /dependency-cache/modules ]; then ln -s /dependency-cache/modules "$STAGE/modules"; fi
export ZEPHYR_BASE="$STAGE/zephyr"
west list > "$AUDIT_OUTPUT/west-list.txt" 2>&1 || true
status=0
for target in $TARGETS; do
    mkdir -p "$STAGE/zmk/app/overlays"
    cp "$CONFIG/config/keymap/qwerty.keymap" "$STAGE/zmk/app/overlays/charybdis.keymap"
    cp "$CONFIG/config/"*.dtsi "$STAGE/zmk/app/overlays/" 2>/dev/null || true
    cp "$CONFIG/config/keymap/"*.dtsi "$STAGE/zmk/app/overlays/" 2>/dev/null || true
    rm -rf "$STAGE/zmk/app/boards/shields/charybdis_bt"
    mkdir -p "$STAGE/zmk/app/boards/shields/charybdis_bt"
    cp -a "$CONFIG/boards/shields/charybdis_bt/." "$STAGE/zmk/app/boards/shields/charybdis_bt/"
    if [ "$target" = left ]; then
        rm -f "$STAGE/zmk/app/boards/shields/charybdis_bt/charybdis_right.conf" "$STAGE/zmk/app/boards/shields/charybdis_bt/charybdis_right.overlay"
    else
        rm -f "$STAGE/zmk/app/boards/shields/charybdis_bt/charybdis_left.conf" "$STAGE/zmk/app/boards/shields/charybdis_bt/charybdis_left.overlay"
    fi
    cp "$CONFIG/config/"*.dtsi "$STAGE/zmk/app/boards/shields/charybdis_bt/" 2>/dev/null || true
    BUILD="$STAGE/build-$target"
    cmake_args=(-DZEPHYR_BASE="$STAGE/zephyr" -DZephyr_DIR="$STAGE/zephyr/share/zephyr-package/cmake" -DSHIELD=charybdis_"$target" -DKEYMAP_FILE=overlays/charybdis.keymap -DZMK_CONFIG=/config/config -DZMK_EXTRA_MODULES=/config/vendor/pmw3610-driver)
    if [ "$target" = right ]; then cmake_args+=( -DSNIPPET=studio-rpc-usb-uart ); fi
    if ! west build -p always -d "$BUILD" -s "$STAGE/zmk/app" -b nice_nano_v2 -- "${cmake_args[@]}" > "$AUDIT_OUTPUT/$target-build.log" 2>&1; then status=1; fi
    mkdir -p "$AUDIT_OUTPUT/$target"
    for file in "$BUILD/zephyr/.config" "$BUILD/zephyr/zephyr.dts" "$BUILD/zephyr/zephyr.elf" "$BUILD/zephyr/zmk.uf2"; do
        [ -f "$file" ] && cp "$file" "$AUDIT_OUTPUT/$target/"
    done
done
snippet="not-used"
[ "$MODE" = right ] || [ "$MODE" = both ] && snippet="studio-rpc-usb-uart"
python3 /config/scripts/audit/provenance.py --source /config --output "$AUDIT_OUTPUT/provenance.json" --text "$AUDIT_OUTPUT/provenance.txt" --west "$AUDIT_OUTPUT/west-list.txt" --artifacts "$AUDIT_OUTPUT" --image "$AUDIT_IMAGE" --image-digest "$AUDIT_IMAGE_DIGEST" --mode "$MODE" --board nice_nano_v2 --keymap qwerty --snippet "$snippet"
if [ "$status" -eq 0 ]; then touch "$AUDIT_OUTPUT/BUILD_OK"; else touch "$AUDIT_OUTPUT/BUILD_FAILED"; fi
exit "$status"
