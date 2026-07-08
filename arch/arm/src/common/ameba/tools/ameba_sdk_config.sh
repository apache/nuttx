#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_sdk_config.sh
#
# Materialize the SDK menuconfig .config = SDK default + the board's optional
# prj.conf-style overlay fragment, but ONLY when the inputs actually change.
#
# Why the guard: the SDK's `menuconfig -r` / `--apply-file` rewrites .config
# every time it runs, which makes the SDK build reconfigure and rebuild most of
# the NP/boot image (~hundreds of objects, ~10s+) on EVERY NuttX build -- even
# when nothing changed.  The resulting config is deterministic (SDK default +
# a fixed overlay), so it only needs regenerating when the SDK revision or the
# overlay fragment changes.  We stamp those inputs; a steady-state build finds
# the stamp unchanged and skips regeneration entirely, so only the AP-driven
# "noused" file is rebuilt.  This is also what makes `make ameba_menuconfig`
# edits take effect: editing the fragment changes the stamp -> next build
# regenerates the SDK config and rebuilds.
#
# usage: ameba_sdk_config.sh <sdk_dir> <soc> [<overlay_frag>]
#   sdk_dir       - ameba-rtos checkout
#   soc           - ameba.py SoC id (RTL8721Dx / RTL8720F / ...)
#   overlay_frag  - optional prj.conf-style fragment (CONFIG_*=... lines,
#                   diff vs SDK default) applied on top of the default config.
#                   If absent, the plain SDK default is used.
#
# Idempotent and safe to call from both the PREBUILD (autoconf generation) and
# the POSTBUILD (NP build): the first call materializes the config + writes the
# stamp, the second sees the stamp match and returns immediately.
############################################################################

set -e

SDK="$1"
SOC="$2"
FRAG="$3"

if [ -z "$SDK" ] || [ -z "$SOC" ]; then
  echo "usage: ameba_sdk_config.sh <sdk_dir> <soc> [<overlay_frag>]" >&2
  exit 1
fi

# Resolve the SDK python (real .venv if present, else system python3), matching
# the other ameba tool scripts.
PYDIR=$(cat "$SDK/.amebapy/bindir" 2>/dev/null)
VENV_PY="$PYDIR/python"
[ -x "$VENV_PY" ] || VENV_PY=python3
PATH="${PYDIR:+$PYDIR:}$PATH"
export PATH

# Isolate from NuttX's Kconfig environment (which otherwise leaks into the SDK's
# own kconfiglib and breaks its Kconfig path resolution), and drop the make
# jobserver fds.  Same prologue as ameba_build_np.sh / ameba_gen_autoconf.sh.
unset srctree BINDIR APPSDIR APPSBINDIR EXTERNALDIR EXTERNDIR KCONFIG_CONFIG \
      ARCHDIR DRIVERDIR BOARDDIR 2>/dev/null || true
unset MAKEFLAGS MAKELEVEL MFLAGS MAKEOVERRIDES 2>/dev/null || true

MC="$SDK/build_$SOC/menuconfig/.config"
STAMP="$SDK/build_$SOC/.nuttx_sdkcfg_stamp"

# Two overlays are layered on the SDK default, in order (later wins):
#   1. user overlay  = $FRAG (ameba_sdk.conf)         -- editable via TUI
#   2. forced overlay = <same dir>/ameba_sdk_force.conf -- always wins, not
#      user-editable (e.g. CONFIG_SHELL=n).  Applied LAST so it overrides both
#      the SDK default and the user overlay.
FORCE=""
if [ -n "$FRAG" ]; then
  FORCE="$(dirname "$FRAG")/ameba_sdk_force.conf"
fi

# Collect the overlays that actually exist, in apply order (user then forced).
OVERLAYS=""
[ -n "$FRAG" ]  && [ -f "$FRAG" ]  && OVERLAYS="$OVERLAYS $FRAG"
[ -n "$FORCE" ] && [ -f "$FORCE" ] && OVERLAYS="$OVERLAYS $FORCE"

# Stamp = pinned SDK revision + the content of both overlays.  A change in any
# (SDK bump, or editing either overlay) invalidates it and forces a regen.
sdkrev=$(git -C "$SDK" rev-parse HEAD 2>/dev/null || echo nogit)
if [ -n "$OVERLAYS" ]; then
  ovhash=$(cat $OVERLAYS 2>/dev/null | sha1sum | cut -d' ' -f1)
else
  ovhash=nooverlay
fi
want="$sdkrev:$ovhash"

if [ -f "$MC" ] && [ "$(cat "$STAMP" 2>/dev/null)" = "$want" ]; then
  exit 0
fi

# ameba.py resolves build_<SOC>/ relative to CWD, so run from the SDK root.
cd "$SDK" || { echo "ameba_sdk_config.sh: cannot cd to $SDK" >&2; exit 1; }

"$VENV_PY" "$SDK/ameba.py" soc "$SOC" >/dev/null
if [ -n "$OVERLAYS" ]; then
  # --apply-file resets to the SDK default and layers the overlays on top in
  # order (forced last), re-resolving dependencies (kconfiglib), so an overlay
  # only needs to carry intent (e.g. CONFIG_SHELL=n); dependents cascade.
  "$VENV_PY" "$SDK/ameba.py" menuconfig "$SOC" --apply-file $OVERLAYS
else
  "$VENV_PY" "$SDK/ameba.py" menuconfig "$SOC" -r
fi

echo "$want" > "$STAMP"
echo "GEN: SDK .config <- default${OVERLAYS:+ +$(for f in $OVERLAYS; do printf ' %s' "$(basename "$f")"; done)} ($SOC)"
