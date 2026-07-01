#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_setup_env.sh
#
# Ensure the ameba-rtos build prerequisites are available, WITHOUT hard-
# requiring the SDK python venv (.venv).  Two things are needed:
#
#   * ninja/cmake -- shipped in the prebuilts bundle the SDK env.sh fetches;
#   * a python interpreter (reachable as both `python` and `python3`) that has
#     the SDK tool deps json5/click used by the cmake setconfig + axf2bin steps.
#
# Resolution is unified across developer machines and CI -- it uses whatever
# already works and only bootstraps when nothing does.  It resolves a *bin
# directory* to prepend to PATH and records it in $SDK/.amebapy/bindir; the
# rest of the build (board.mk PATH prepend, ameba_gen_autoconf.sh /
# ameba_build_np.sh) reads that file instead of hard-coding $SDK/.venv/bin:
#
#   1. the SDK venv ($SDK/.venv/bin), if its python imports the deps -- a
#      developer who ran `source env.sh`, or a cached venv.  The venv python
#      MUST be used by its real path (so its site-packages stay active), so
#      this case publishes $SDK/.venv/bin directly.
#   2. otherwise the system python3, if it imports the deps (CI installs them;
#      env.sh also pip-installs them to the system when its venv is pip-less).
#      A thin $SDK/.amebapy/bin is created with python/python3 symlinks to the
#      system interpreter (it is not a venv, so the symlinks keep the system
#      site-packages, and a bare `python` exists for the SDK cmake steps).
#   3. otherwise run the SDK env.sh once to bootstrap prebuilts + venv, then
#      re-resolve.
#
# This is why the build no longer fails on a broken/missing .venv (e.g. a CI
# image without python3-venv/ensurepip): as long as the system python3 carries
# the deps, the build proceeds against it.
#
# Usage: ameba_setup_env.sh <sdk_dir> [toolchain_dir]
#
# Idempotent: a no-op once ninja and a deps-carrying python are resolved.
############################################################################

set -e

SDK="$1"
TCDIR="${2:-$HOME/rtk-toolchain}"

[ -f "$SDK/env.sh" ] || { echo "ameba_setup_env.sh: no env.sh in $SDK" >&2; exit 1; }

PBVER=$(sed -n 's/^PREBUILTS_VERSION=//p' "$SDK/env.sh" | head -1)
PB="$TCDIR/prebuilts-linux-$PBVER"
SHIMBIN="$SDK/.amebapy/bin"
BINFILE="$SDK/.amebapy/bindir"

# env.sh installs the full tools/requirements.txt into whichever python it can
# (the SDK .venv on a dev box; the system python3 in CI, where its venv is
# pip-less).  This sentinel just confirms that install landed in the python we
# are about to use.  It checks json5 + click (the ameba.py / setconfig deps)
# AND Crypto (pycryptodome) -- a *compiled* wheel: the deps that fail on a
# flaky PyPI / a toolchain-less image are the compiled ones, so a successful
# `import Crypto` is a reliable signal the whole set (incl. cryptography/ecdsa/
# pyelftools/... used by the axf2bin image-signing step) installed too.
# kconfiglib is vendored in the SDK (put on sys.path by ameba.py), not gated.
py_has_deps() {
  "$1" -c 'import json5, click, Crypto' 2>/dev/null
}

# Echo the bin directory whose python/python3 carry the deps, or return 1.
resolve_bindir() {
  # 1. real SDK venv -- used directly so its site-packages stay active.
  if [ -x "$SDK/.venv/bin/python" ] && py_has_deps "$SDK/.venv/bin/python"; then
    printf '%s\n' "$SDK/.venv/bin"
    return 0
  fi

  # 2. system python3 -- expose it via a non-venv shim dir so a bare `python`
  #    exists and the system site-packages (with the deps) stay visible.
  if command -v python3 >/dev/null 2>&1 && py_has_deps python3; then
    sp=$(command -v python3)
    mkdir -p "$SHIMBIN"
    ln -sf "$sp" "$SHIMBIN/python"
    ln -sf "$sp" "$SHIMBIN/python3"
    printf '%s\n' "$SHIMBIN"
    return 0
  fi

  return 1
}

publish_bindir() {
  mkdir -p "$SDK/.amebapy"
  printf '%s\n' "$1" > "$BINFILE"
}

# Fast path: ninja present and a deps-carrying python already resolves.
if [ -x "$PB/bin/ninja" ] && BINDIR=$(resolve_bindir); then
  publish_bindir "$BINDIR"
  exit 0
fi

echo "Setting up ameba-rtos build prerequisites (prebuilts + python deps) ..."

# Bootstrap via the SDK env.sh (fetches the prebuilts bundle; on a developer
# box also builds the .venv and pip-installs the deps).  Best-effort: on CI it
# may only manage to install the deps to the system python3, which is fine.
RTK_TOOLCHAIN_DIR="$TCDIR" bash -c "cd '$SDK' && . ./env.sh" </dev/null || true

# ninja/cmake come only from the prebuilts bundle -- still mandatory.
[ -x "$PB/bin/ninja" ] || { \
  echo "ameba_setup_env.sh: prebuilts (ninja) missing at $PB after env.sh." >&2; \
  echo "  Check network access to the ameba prebuilts release." >&2; \
  exit 1; }

if BINDIR=$(resolve_bindir); then
  publish_bindir "$BINDIR"
  echo "ameba-rtos prerequisites ready (prebuilts $PBVER, python dir: $BINDIR)."
  exit 0
fi

echo "ameba_setup_env.sh: no python carries the SDK tool deps." >&2
echo "  Provide them for the system python3 (the full SDK tool set), e.g.:" >&2
echo "    python3 -m pip install -r $SDK/tools/requirements.txt" >&2
echo "  or 'apt install python3-venv' so env.sh can build the SDK .venv." >&2
exit 1
