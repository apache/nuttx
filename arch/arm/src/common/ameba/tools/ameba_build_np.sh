#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_build_np.sh
#
# Build the NP (network/device core) image2 AND the bootloader FROM THE PINNED
# SDK SOURCE, and drop <NP_TARGET>_image2_all.bin + boot.bin into <outdir>.
# The NP core's SDK build target differs per IC (passed as <np_target>):
# km0 on RTL8721Dx (amebadplus), km4ns on RTL8720F, ...
#
# Why build (not ship release bins): the SDK shares many .c files and configs
# between the AP (NuttX) and NP/boot, so they must be built from the SAME pinned
# SDK as the AP image.  The SDK build is incremental, so re-running this every
# NuttX build is cheap when nothing changed.
#
# Uses the SDK's native single-core / boot build entry points:
#   ameba.py build <SOC> --core <NP_TARGET>   # -> project_<NP_TARGET>/image/<NP_TARGET>_image2_all.bin
#   ameba.py build <SOC> -g boot              # -> .../image/boot.bin
# The upstream SDK supports these natively (no SDK patching): `-k/--core`
# builds the given core (its target already depends on the image2 postbuild),
# `-g boot` builds the bootloader, and wifi_feature_disable honours the
# AMEBA_AP_ASM env var to generate the "noused" stubs from an external AP.
#
# The NP WiFi driver strips ("noused") any host (AP) WiFi API the AP image does
# not reference, replacing it with a stub that, if ever called, sets call_noused
# and the NP api task DEADLOCKS ("Compile NP after AP!").  The SDK normally feeds
# gen_noused_c.py the AP image's disassembly (built first).  Here the AP is NuttX
# (built outside the SDK), so its disassembly is passed in as <ap_asm> and handed
# to the SDK via the AMEBA_AP_ASM env var (see wifi_feature_disable/CMakeLists.txt).
# Without it the NP stubs APIs NuttX uses (e.g. wifi_get_scan_records) and hangs.
#
# Usage: ameba_build_np.sh <sdk_dir> <soc> <outdir> [<ap_asm>]
############################################################################

set -e

SDK="$1"
SOC="$2"
OUT="$3"
AP_ASM="$4"
# NP/device-core build target: "km0" on amebadplus, "km4ns" on RTL8720F, etc.
# Its image2 is project_<NP_TARGET>/image/<NP_TARGET>_image2_all.bin.
NP_TARGET="${5:-km0}"
# AP-core project name: "km4" on amebadplus, "km4tz" on RTL8720F.  Selects which
# per-core SDK kconfig (build_<SOC>/menuconfig/.config_<AP_CORE>) to stage as
# config_km4 for the AP image2 packaging step (see below).
AP_CORE="${6:-km4}"

if [ -z "$SDK" ] || [ -z "$SOC" ] || [ -z "$OUT" ]; then
  echo "usage: ameba_build_np.sh <sdk_dir> <soc> <outdir> [<ap_asm>] [<np_target>]" >&2
  exit 1
fi

# Hand the AP (NuttX KM4) disassembly to the SDK's WiFi noused generator so the
# NP keeps real implementations for the WiFi APIs NuttX actually calls.
if [ -n "$AP_ASM" ]; then
  [ -f "$AP_ASM" ] || { echo "ameba_build_np.sh: AP asm '$AP_ASM' not found" >&2; exit 1; }
  AMEBA_AP_ASM="$AP_ASM"
  export AMEBA_AP_ASM
fi

# Use the python dir ameba_setup_env.sh resolved (a real .venv when one
# exists, otherwise a system-python3 shim); fall back to system python3.
PYDIR=$(cat "$SDK/.amebapy/bindir" 2>/dev/null)
VENV_PY="$PYDIR/python"
[ -x "$VENV_PY" ] || VENV_PY=python3

mkdir -p "$OUT"

# Put that bin dir FIRST on PATH so the SDK build's bare `python` calls
# (setconfig.py / axf2bin) and cmake's FindPython3 resolve to the same
# interpreter (which has json5/pycryptodome).  Without this the SDK configure
# fails with "Miss module: json5".  prebuilts (ninja/cmake) + asdk are already
# on PATH from toolchain.mk in the NuttX build context.
PATH="${PYDIR:+$PYDIR:}$PATH"
export PATH

# Isolate the SDK build from NuttX's Kconfig environment: NuttX exports
# BINDIR/APPSDIR/APPSBINDIR/EXTERNALDIR (tools/Unix.mk KCONFIG_ENV) plus
# srctree/KCONFIG_CONFIG, which leak into the SDK's own kconfiglib (ameba.py
# menuconfig/build) and break its Kconfig path resolution.  Clear them so the
# SDK uses its own.
unset srctree BINDIR APPSDIR APPSBINDIR EXTERNALDIR EXTERNDIR KCONFIG_CONFIG \
      ARCHDIR DRIVERDIR BOARDDIR 2>/dev/null || true
# Also drop the make jobserver so the SDK's nested ninja doesn't choke on
# NuttX's jobserver fds ("Could not initialize jobserver: Invalid fds").
unset MAKEFLAGS MAKELEVEL MFLAGS MAKEOVERRIDES 2>/dev/null || true

# ameba.py resolves build_<SOC>/ relative to the CWD, so it must run from the
# SDK root (not the NuttX tree).
cd "$SDK" || { echo "ameba_build_np.sh: cannot cd to $SDK" >&2; exit 1; }

WORKDIR="$SDK/build_$SOC/build"

# collect <name> <relpath-under-WORKDIR>: copy immediately after its build, so
# a later -g build (which cleans the other core's image dir) cannot wipe it.
collect() {
  name="$1"; rel="$2"
  src="$WORKDIR/$rel"
  [ -f "$src" ] || src=$(find "$SDK/build_$SOC" -name "$name" 2>/dev/null | head -1)
  [ -f "$src" ] || { echo "ameba_build_np.sh: $name not produced" >&2; exit 1; }
  cp "$src" "$OUT/$name"
}

"$VENV_PY" "$SDK/ameba.py" soc "$SOC"
# --set needs an existing .config; -r generates the default one first (so this
# works on a clean tree).  Result is deterministic: default config + SHELL=n.
"$VENV_PY" "$SDK/ameba.py" menuconfig "$SOC" -r
"$VENV_PY" "$SDK/ameba.py" menuconfig "$SOC" --set SHELL=n

# Build the NP/device image2 (with AMEBA_AP_ASM-driven noused, so it stays
# aligned with exactly the host WiFi APIs NuttX references).
"$VENV_PY" "$SDK/ameba.py" build "$SOC" --core "$NP_TARGET"

# Collect the NP image2 NOW, before the boot build: an isolated `-g boot` (which
# succeeds on amebadplus) reconfigures and cleans the OTHER core's image dir,
# wiping the just-built NP image -- so it must be saved immediately.
# project_km0 on amebadplus, project_km4ns on RTL8720F -- collect() falls back
# to a tree-wide find, so the exact relpath is only a hint.
collect "${NP_TARGET}_image2_all.bin" "project_${NP_TARGET}/image/${NP_TARGET}_image2_all.bin"

# Build the boot (image1) loader.  Prefer the isolated `-g boot` target; some
# SoCs (RTL8720F) cannot wire its loader-postbuild dependency in isolation, so
# fall back to a full SoC build, which builds boot correctly -- and, because
# AMEBA_AP_ASM is still exported, also rebuilds the NP image with the right
# noused set (the SDK's own AP image is built too and simply discarded).  Both
# the NP and boot are therefore freshly rebuilt every NuttX build; nothing is
# reused stale across the two cores.
if ! "$VENV_PY" "$SDK/ameba.py" build "$SOC" -g boot; then
  echo "ameba_build_np.sh: '-g boot' not buildable in isolation on $SOC;" \
       "doing a full SoC build to produce boot (AMEBA_AP_ASM kept)" >&2
  "$VENV_PY" "$SDK/ameba.py" build "$SOC"
fi

collect boot.bin project_km4/image/boot.bin

# Stage the SDK kconfig snapshots the NuttX POSTBUILD packaging step consumes:
#   config_km4 - the AP-core (<AP_CORE>) kconfig, used to wrap NuttX's img2 into
#                <ap>_image2_all.bin (axf2bin make/image2/postbuild.cmake).
#   config_fw  - the full-firmware kconfig, used to combine boot + NP + AP into
#                app.bin (project/postbuild.cmake).
# These are taken from the SDK BUILD output (build_<SOC>/build/...), which is the
# exact set the SDK's own postbuild reads -- see cmake/common.cmake: the SoC
# combine uses ${BINARY_DIR}/.config (build/.config) and each image2 uses
# ${BINARY_DIR}/.config_<core> (build/project_<core>/.config_<core>).  These are
# written by the configure step of the builds run above (the AP core is
# configured by the `-g boot` / full SoC build), with SHELL=n, from the PINNED
# SDK -- never committed -- so a clean checkout reproduces them and they can
# never drift from the images they were built against.  Fall back to the
# menuconfig snapshot (byte-identical in practice) if the build dir lacks one.
BUILDCFG="$SDK/build_$SOC/build"
MENUCFG="$SDK/build_$SOC/menuconfig"
stage_cfg() {
  dst="$1"; build_rel="$2"; menu_rel="$3"
  if [ -f "$BUILDCFG/$build_rel" ]; then
    cp "$BUILDCFG/$build_rel" "$OUT/$dst"
  elif [ -f "$MENUCFG/$menu_rel" ]; then
    cp "$MENUCFG/$menu_rel" "$OUT/$dst"
  else
    echo "ameba_build_np.sh: kconfig for $dst not produced" \
         "($build_rel / $menu_rel)" >&2
    exit 1
  fi
}
stage_cfg config_km4 "project_$AP_CORE/.config_$AP_CORE" ".config_$AP_CORE"
stage_cfg config_fw  ".config"                           ".config"

echo "NP/boot: built ${NP_TARGET}_image2_all.bin + staged kconfig from SDK ($SOC)"
