#!/usr/bin/env bash
# =============================================================================
# fetch_ele_fw.sh — download the pinned NXP EdgeLock Enclave firmware AHAB
#                   container into the shared build cache.  Used both by
#                   build_flash_image.sh (for the AHAB-side integration) and
#                   as a make dependency for the driver-side .incbin.
# =============================================================================
#
# On success the file lands at:
#
#   tools/imxrt1180/.cache/ele-fw/mxrt1180b0-ahab-container.img
#
# and is SHA-256 verified against the pinned value.  Running this script
# implies acceptance of NXP's LA_OPT NXP Software License (same convention
# as the iMX9 --auto-accept flow).
#
# =============================================================================

set -euo pipefail

# Pinned versions — bump only via code review.
ELE_FW_PIN_SHA="6f3fd257cdcf978a4d26e7d6e9eed9240037422b"
ELE_FW_NAME="mxrt1180b0-ahab-container.img"
ELE_FW_URL="https://raw.githubusercontent.com/nxp-mcuxpresso/mcux-sdk/${ELE_FW_PIN_SHA}/firmware/edgelock/${ELE_FW_NAME}"
ELE_FW_LICENSE_URL="https://raw.githubusercontent.com/nxp-mcuxpresso/mcux-sdk/${ELE_FW_PIN_SHA}/LICENSE.txt"
ELE_FW_SHA256="51be3d3bc23ced3026a69a187d44e80e51a0d19355f37f20f0dad1161d690589"

err() { printf "error: %s\n" "$*" >&2; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CACHE_DIR="${SCRIPT_DIR}/.cache/ele-fw"
ELE_FW_FILE="${CACHE_DIR}/${ELE_FW_NAME}"

mkdir -p "${CACHE_DIR}"

if [ -f "${ELE_FW_FILE}" ] && \
   [ "$(sha256sum "${ELE_FW_FILE}" | awk '{print $1}')" = "${ELE_FW_SHA256}" ]
then
    # Print the resolved path so callers (Makefile rule) can capture it.
    echo "${ELE_FW_FILE}"
    exit 0
fi

cat >&2 <<EOF
==============================================================================
Fetching NXP EdgeLock Enclave firmware AHAB container
------------------------------------------------------------------------------
File:    ${ELE_FW_NAME}
Source:  ${ELE_FW_URL}
License: ${ELE_FW_LICENSE_URL}
         (LA_OPT NXP Software License v56, April 2024)

The ELE FW is proprietary NXP object code.  Running this script implies
acceptance of NXP's LA_OPT NXP Software License
==============================================================================
EOF

rm -f "${ELE_FW_FILE}"

if command -v curl >/dev/null 2>&1; then
    curl --fail --location --silent --show-error \
         --output "${ELE_FW_FILE}.tmp" "${ELE_FW_URL}"
elif command -v wget >/dev/null 2>&1; then
    wget --quiet -O "${ELE_FW_FILE}.tmp" "${ELE_FW_URL}"
else
    err "neither curl nor wget available for downloading ${ELE_FW_URL}"
fi

got="$(sha256sum "${ELE_FW_FILE}.tmp" | awk '{print $1}')"
if [ "${got}" != "${ELE_FW_SHA256}" ]; then
    rm -f "${ELE_FW_FILE}.tmp"
    err "ELE FW SHA256 mismatch (expected ${ELE_FW_SHA256}, got ${got})"
fi
mv "${ELE_FW_FILE}.tmp" "${ELE_FW_FILE}"

echo "${ELE_FW_FILE}"
