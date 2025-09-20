#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(dirname "$(realpath "$0")")"
NUTTX_PATH=$SCRIPT_PATH/..
APP_PATH=$SCRIPT_PATH/../../apps

# === CONFIGURATION ===
# Allow overriding from environment for CI
EXPORT_CONFIG=${EXPORT_CONFIG:-"stm32f4discovery:cxx-oot-build"}
OOT_SRC=${OOT_SRC:-"$APP_PATH/testing/oot-build"}
BUILD_DIR=${BUILD_DIR:-"$OOT_SRC/build"}

# Place ourselves in the nuttx dir
cd $NUTTX_PATH

echo "=== [1/5] Configuring NuttX Export ($EXPORT_CONFIG) ==="
./tools/configure.sh -E -l "$EXPORT_CONFIG"

echo "=== [2/5] Building Export Tarball ==="
make -j"$(nproc)" export

EXPORT_TARBALL=$(find . -maxdepth 1 -type f -name "nuttx-export-*.tar.gz" | sort | tail -n 1)
if [[ -z "$EXPORT_TARBALL" ]]; then
    echo "❌ ERROR: No export tarball found"
    exit 1
fi

echo "=== [3/5] Preparing Out of Tree (OOT) Project ==="
rm -rf "$OOT_SRC"/nuttx-export-*
tar -xzf "$EXPORT_TARBALL" -C "$OOT_SRC"

TOOLCHAIN_FILE=$(find "$OOT_SRC" -type f -path "*/scripts/toolchain.cmake" | head -n 1)
if [[ ! -f "$TOOLCHAIN_FILE" ]]; then
    echo "❌ ERROR: toolchain.cmake not found after export"
    exit 1
fi

echo "=== [4/5] Building OOT ==="
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"
make -j"$(nproc)"

echo "=== [5/5] Verifying Output ==="
if [[ ! -f "$BUILD_DIR/oot" || ! -f "$BUILD_DIR/oot.bin" ]]; then
    echo "❌ ERROR: oot or oot.bin not found in $BUILD_DIR"
    exit 1
fi

echo "✅ SUCCESS: OOT build completed. Output:"
ls -lh "$BUILD_DIR"/oot*
