#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_asdk_version.sh
#
# Resolve the asdk (arm-none-eabi) toolchain VERSION the SDK requires for a
# given SoC, so the NuttX objects are built with the SAME toolchain the SDK
# archives they link against were built with.  The SDK is the single source
# of truth and resolves the version per-SoC, in this order:
#
#   1. component/soc/<soc>/project/CMakeLists.txt  ->  set(v_ASDK_VER X)
#        Per-IC override; e.g. RTL8720F / amebagreen2 pin 12.3.1.
#   2. cmake/global_define.cmake  ->  ameba_set_if_unset(v_ASDK_VER X)
#        The default for SoCs without an override (e.g. RTL8721Dx -> 10.3.1).
#
# Usage: ameba_asdk_version.sh <sdk_dir> [soc_name]
#   Prints the resolved version (e.g. "12.3.1") to stdout, nothing on failure.
#   With no <soc_name> (or an unknown one) it returns the global default, which
#   keeps the make/cmake callers backward compatible.
############################################################################

SDK="$1"
SOC="$2"

[ -n "$SDK" ] || exit 0

ver=

# 1. Per-SoC override, if this SoC pins its own version.
if [ -n "$SOC" ] && [ -f "$SDK/component/soc/$SOC/project/CMakeLists.txt" ]; then
  ver=$(sed -n 's/.*v_ASDK_VER[ \t][ \t]*\([0-9.][0-9.]*\).*/\1/p' \
        "$SDK/component/soc/$SOC/project/CMakeLists.txt" | head -1)
fi

# 2. Fall back to the global default.
if [ -z "$ver" ]; then
  ver=$(sed -n 's/.*v_ASDK_VER[ \t][ \t]*\([0-9.][0-9.]*\).*/\1/p' \
        "$SDK/cmake/global_define.cmake" 2>/dev/null | head -1)
fi

[ -n "$ver" ] && echo "$ver"
