#!/usr/bin/env bash
############################################################################
# tools/ameba/ameba_menuconfig.sh
#
# Edit the Realtek Ameba SDK menuconfig for the currently-configured board,
# using the SDK's own native menuconfig UI, and persist the result as the
# board's prj.conf-style overlay fragment (boards/.../<board>/ameba_sdk.conf).
#
# This is the SDK-configuration counterpart to NuttX's own `make menuconfig`:
# whatever the vendor SDK exposes is shown (no hand-maintained option mapping),
# and only the diff-vs-default is saved back to the committed fragment.  The
# next build picks up the change automatically (the fragment is hashed into the
# SDK-config stamp; see ameba_sdk_config.sh), regenerates the SDK .config and
# rebuilds.
#
# Normally invoked via `make ameba_menuconfig` (the target lives in the board's
# tools/ameba/Config.mk).  Can also be run directly from the NuttX top-level
# directory after configuring a board:
#
#   ./tools/configure.sh pke8721daf:nsh
#   make ameba_menuconfig            # or: ./tools/ameba/ameba_menuconfig.sh
#
# The logic is kept here (rather than inline in the make target) so it stays
# readable and directly runnable; the target is a thin wrapper.
############################################################################

set -e

TOPDIR=$(cd "$(dirname "$0")/../.." && pwd)
CONFIG="$TOPDIR/.config"

if [ ! -f "$CONFIG" ]; then
  echo "error: $CONFIG not found -- configure a board first" \
       "(./tools/configure.sh <board>:<config>)" >&2
  exit 1
fi

getcfg() { sed -n "s/^$1=\"\?\([^\"]*\)\"\?$/\1/p" "$CONFIG" | head -1; }
CHIP=$(getcfg CONFIG_ARCH_CHIP)
BOARD=$(getcfg CONFIG_ARCH_BOARD)

case "$CHIP" in
  rtl8721dx|rtl8720f) ;;
  *) echo "error: not an Ameba board (CONFIG_ARCH_CHIP='$CHIP')" >&2; exit 1;;
esac

BOARDMK="$TOPDIR/arch/arm/src/$CHIP/ameba_board.mk"
FRAG="$TOPDIR/boards/arm/$CHIP/$BOARD/ameba_sdk.conf"

# The ameba.py SoC id (RTL8721Dx / RTL8720F) is defined once per IC in the
# board.mk; read it there instead of hardcoding a mapping.
SOC=$(sed -n 's/^AMEBA_PY_SOC[[:space:]]*=[[:space:]]*\([^[:space:]]*\).*/\1/p' "$BOARDMK" | head -1)
if [ -z "$SOC" ]; then
  echo "error: could not read AMEBA_PY_SOC from $BOARDMK" >&2
  exit 1
fi

# Resolve the SDK: explicit AMEBA_SDK, else the in-tree auto-fetched checkout.
SDK="${AMEBA_SDK:-$TOPDIR/arch/arm/src/common/ameba/ameba-rtos}"
if [ ! -f "$SDK/ameba.py" ]; then
  echo "error: SDK not found at $SDK (build the board once to auto-fetch it," \
       "or export AMEBA_SDK)" >&2
  exit 1
fi

# SDK python + isolated Kconfig env, matching the other ameba tool scripts.
PYDIR=$(cat "$SDK/.amebapy/bindir" 2>/dev/null)
VENV_PY="$PYDIR/python"
[ -x "$VENV_PY" ] || VENV_PY=python3
PATH="${PYDIR:+$PYDIR:}$PATH"
export PATH
unset srctree BINDIR APPSDIR APPSBINDIR EXTERNALDIR EXTERNDIR KCONFIG_CONFIG \
      ARCHDIR DRIVERDIR BOARDDIR MAKEFLAGS MAKELEVEL MFLAGS MAKEOVERRIDES \
      2>/dev/null || true

cd "$SDK"
"$VENV_PY" "$SDK/ameba.py" soc "$SOC" >/dev/null

# Forced overlay (same dir, fixed name): options that must stay at a fixed value
# regardless of what is done in the TUI (e.g. CONFIG_SHELL=n).  Applied LAST at
# load and re-applied after the TUI saves, so they cannot be turned on here.
FORCE="$(dirname "$FRAG")/ameba_sdk_force.conf"

OVERLAYS=""
[ -f "$FRAG" ]  && OVERLAYS="$OVERLAYS $FRAG"
[ -f "$FORCE" ] && OVERLAYS="$OVERLAYS $FORCE"

# 1. Materialize default + overlays (user then forced), so the TUI opens on the
#    board's current effective config with the forced options in their state.
if [ -n "$OVERLAYS" ]; then
  # shellcheck disable=SC2086
  "$VENV_PY" "$SDK/ameba.py" menuconfig "$SOC" --apply-file $OVERLAYS >/dev/null
else
  "$VENV_PY" "$SDK/ameba.py" menuconfig "$SOC" -r >/dev/null
fi

# 2. Native interactive menuconfig (shows everything the SDK exposes).
"$VENV_PY" "$SDK/ameba.py" menuconfig "$SOC"

# 3. Re-apply the forced options on top of whatever the TUI saved, so a user who
#    toggled one of them back on is corrected.  --set edits the current .config
#    in place (unlike --apply-file, which would reset all TUI edits).
if [ -f "$FORCE" ]; then
  forced=$(grep -E '^[[:space:]]*CONFIG_[A-Za-z0-9_]+=' "$FORCE" | sed 's/^[[:space:]]*//')
  if [ -n "$forced" ]; then
    # shellcheck disable=SC2086
    "$VENV_PY" "$SDK/ameba.py" menuconfig "$SOC" --set $forced >/dev/null
  fi
fi

# 4. Save the diff-vs-default back to the USER overlay (forced options already
#    corrected in step 3, so what is saved matches the effective config).
mkdir -p "$(dirname "$FRAG")"
"$VENV_PY" "$SDK/ameba.py" menuconfig "$SOC" --save-file "$FRAG" >/dev/null

echo "Saved SDK overlay -> $FRAG"
if [ -f "$FORCE" ]; then
  echo "Forced options ($FORCE) are always re-applied and cannot be overridden."
fi
echo "Rebuild the board (make / make flash) to apply; unchanged options are a no-op."
