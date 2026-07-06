#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_fetch_toolchain.sh
#
# Idempotently download + extract the asdk (arm-none-eabi) toolchain that the
# fetched SDK pins, WITHOUT modifying the SDK.  The version, archive name and
# download URLs are all read from the SDK's own toolchain cmake (single source
# of truth), then fetched with wget + tar exactly the way the SDK's
# cmake/toolchain/ameba-toolchain-check.cmake does:
#
#   download  $TOOLCHAINURL/$NAME      ->  $TOOLCHAIN_DIR
#   tar -jxf  $NAME                    ->  $TOOLCHAIN_DIR/asdk-<ver>
#   rename    asdk-<ver>               ->  asdk-<ver>-<build>
#
# Usage: ameba_fetch_toolchain.sh <sdk_dir> [toolchain_dir]
#   <sdk_dir>        ameba-rtos checkout (has cmake/global_define.cmake)
#   [toolchain_dir]  install root (default: $HOME/rtk-toolchain)
#
# No-op when the matching toolchain is already present, so it is safe to call
# at the start of every build.
############################################################################

set -e

SDK="$1"
TOOLCHAIN_DIR="${2:-$HOME/rtk-toolchain}"
SOC="$3"

if [ -z "$SDK" ] || [ ! -f "$SDK/cmake/global_define.cmake" ]; then
  echo "ameba_fetch_toolchain.sh: bad SDK dir '$SDK'" >&2
  exit 1
fi

# --- Read the pinned version from the SDK (the single source of truth) ------
# The version is declared per SoC-project (component/soc/<soc>/project/
# CMakeLists.txt) which OVERRIDES the top-level global_define.cmake default
# (e.g. RTL8720F pins 12.3.1 vs the amebadplus 10.3.1 default).  Prefer the
# per-SoC file when a SoC name is given, so a bare machine fetches the exact
# toolchain the NuttX image2 link will use.

VER_SRC="$SDK/cmake/global_define.cmake"
if [ -n "$SOC" ] && [ -f "$SDK/component/soc/$SOC/project/CMakeLists.txt" ]; then
  VER_SRC="$SDK/component/soc/$SOC/project/CMakeLists.txt"
fi

VER=$(sed -n 's/.*v_ASDK_VER[ \t][ \t]*\([0-9.][0-9.]*\).*/\1/p' "$VER_SRC")
TC_CMAKE="$SDK/cmake/toolchain/ameba-toolchain-asdk-$VER.cmake"
if [ -z "$VER" ] || [ ! -f "$TC_CMAKE" ]; then
  echo "ameba_fetch_toolchain.sh: cannot resolve ASDK version from SDK" >&2
  exit 1
fi
BUILD=$(sed -n 's/.*ToolChainVerMinor[ \t][ \t]*\([0-9][0-9]*\).*/\1/p' "$TC_CMAKE")

MAJOR="asdk-$VER"
DEST="$TOOLCHAIN_DIR/$MAJOR-$BUILD"

# Already installed?  (matches what toolchain.mk expects on PATH)
if [ -x "$DEST/linux/newlib/bin/arm-none-eabi-gcc" ]; then
  exit 0
fi

# Archive name + candidate URLs, read from the SDK toolchain cmake.  The cmake
# offers an Aliyun (default) and a GitHub (USE_SECOND_SOURCE) mirror; try both.
#
# The archive name differs per toolchain version (e.g. asdk-10.3.1 has a
# "_with_small_reent" suffix, asdk-12.3.1 does not), so read the Linux
# TOOLCHAINNAME template from the cmake and expand its ${ToolChainVer*} vars
# instead of hardcoding the suffix -- hardcoding it 404s on 12.3.1.
NAME=$(grep -E 'set\(TOOLCHAINNAME .*linux-newlib.*\.tar\.bz2' "$TC_CMAKE" \
       | head -1 | sed -E 's/.*set\(TOOLCHAINNAME[ \t]+(.+)\).*/\1/')
NAME=$(printf '%s' "$NAME" \
       | sed -e "s/\${ToolChainVerMajor}/$MAJOR/g" -e "s/\${ToolChainVerMinor}/$BUILD/g")
if [ -z "$NAME" ]; then
  echo "ameba_fetch_toolchain.sh: cannot resolve toolchain archive name from $TC_CMAKE" >&2
  exit 1
fi
URLS=$(sed -n 's/.*set(TOOLCHAINURL[ \t][ \t]*\(http[^ )]*\).*/\1/p' "$TC_CMAKE")
if [ -z "$URLS" ]; then
  echo "ameba_fetch_toolchain.sh: no TOOLCHAINURL found in $TC_CMAKE" >&2
  exit 1
fi

echo "Fetching asdk toolchain $MAJOR-$BUILD (one-time) into $TOOLCHAIN_DIR ..."
mkdir -p "$TOOLCHAIN_DIR"

# Download (skip if the archive is already there), trying each mirror.
if [ ! -f "$TOOLCHAIN_DIR/$NAME" ]; then
  ok=
  for u in $URLS; do
    echo "  trying $u/$NAME"
    if wget --progress=bar:force -O "$TOOLCHAIN_DIR/$NAME" "$u/$NAME"; then
      ok=1; break
    fi
    rm -f "$TOOLCHAIN_DIR/$NAME"
  done
  if [ -z "$ok" ]; then
    echo "ameba_fetch_toolchain.sh: download failed from all mirrors" >&2
    exit 1
  fi
fi

# Extract (-> asdk-<ver>) then rename to asdk-<ver>-<build>, as the SDK does.
echo "  extracting $NAME ..."
rm -rf "$TOOLCHAIN_DIR/$MAJOR"
tar -jxf "$TOOLCHAIN_DIR/$NAME" -C "$TOOLCHAIN_DIR"
rm -rf "$DEST"
mv "$TOOLCHAIN_DIR/$MAJOR" "$DEST"

if [ ! -x "$DEST/linux/newlib/bin/arm-none-eabi-gcc" ]; then
  echo "ameba_fetch_toolchain.sh: extracted tree missing arm-none-eabi-gcc at $DEST" >&2
  exit 1
fi
echo "  installed $MAJOR-$BUILD"
