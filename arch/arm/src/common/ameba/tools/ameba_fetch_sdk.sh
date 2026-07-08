#!/bin/sh
############################################################################
# arch/arm/src/common/ameba/tools/ameba_fetch_sdk.sh
#
# Idempotently clone the ameba-rtos SDK source at the pinned commit.  One
# checkout under arch/arm/src/common/ameba/ serves every ameba ARM IC.
#
# This ONLY fetches source.  The dev environment (prebuilts = ninja/cmake, and
# the python venv) is provisioned separately by ameba_setup_env.sh, which runs
# the SDK's own env.sh -- the canonical, version-matched setup.
#
# The pinned upstream SDK natively supports the out-of-SDK build NuttX needs
# (single-core `--core <mcu>` and `-g boot` entry points, and the
# AMEBA_AP_ASM-driven WiFi "noused" branch), so NO SDK patching is required.
# Any *.patch dropped in ../patches/ is still applied (extension point), but the
# tree currently ships none.
#
# The clone is SHALLOW (depth 1) at the pinned commit: no history is fetched,
# which keeps the download small/fast (the checked-out source tree is the same
# size either way).  GitHub serves fetch-by-SHA, so the exact pin lands.
#
# Usage: ameba_fetch_sdk.sh <url> <version> <dest>
#
# Safe to call repeatedly: the clone is created only when missing.
############################################################################

set -e

URL="$1"
VERSION="$2"
DEST="$3"

if [ -z "$URL" ] || [ -z "$VERSION" ] || [ -z "$DEST" ]; then
  echo "ameba_fetch_sdk.sh: missing argument (url version dest)" >&2
  exit 1
fi

# patches/ sits next to this script's parent (tools/../patches).
PATCH_DIR="$(CDPATH= cd "$(dirname "$0")/../patches" 2>/dev/null && pwd || true)"

if [ ! -d "$DEST/component/soc" ]; then
  echo "Auto-fetching ameba-rtos SDK $VERSION into $DEST (shallow) ..."
  # Clear any partial/failed previous fetch so a retry is idempotent (a half
  # fetch leaves a .git with 'origin' already added, which would otherwise make
  # 'git remote add' below fail).
  rm -rf "$DEST"
  # Shallow fetch of the exact pinned commit -- no history, smaller/faster than
  # a full clone.  (`git clone --depth 1` only works for a branch/tag tip, so
  # init + fetch <sha> is used to land an arbitrary pinned commit.)
  git init --quiet "$DEST"
  git -C "$DEST" remote add origin "$URL"
  git -C "$DEST" fetch --quiet --depth 1 origin "$VERSION"
  git -C "$DEST" checkout --quiet FETCH_HEAD

  # Apply the NuttX build patches on top of the pinned commit.  Done only on a
  # fresh clone so re-runs do not double-apply.
  if [ -n "$PATCH_DIR" ] && [ -d "$PATCH_DIR" ]; then
    for p in "$PATCH_DIR"/*.patch; do
      [ -f "$p" ] || continue
      echo "Applying SDK patch: $(basename "$p")"
      git -C "$DEST" apply "$p" || {
        echo "ameba_fetch_sdk.sh: failed to apply $(basename "$p")" >&2
        exit 1
      }
    done
  fi
fi
