#!/usr/bin/env sh
# tools/update_romfs_password.sh
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Usage:
#   update_romfs_password.sh <path-to-.config>
#
# When CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y and the root password is not
# set in .config, copy NUTTX_ROMFS_PASSWD_PASSWORD into .config.  If an extra
# ROMFS user is enabled without using the root password, also copy
# NUTTX_ROMFS_PASSWD_EXTRA_PASSWORD.  This is the supported way to supply
# build-time credentials that must not live in defconfig (CI, automation,
# local scripts).  No-op when the password is already set or ROMFS passwd
# generation is disabled.

set -e

CONFIG="${1}"
PASSWD_ENV="${NUTTX_ROMFS_PASSWD_PASSWORD:-}"

if [ -z "${CONFIG}" ]; then
  printf 'Usage: update_romfs_password.sh <path-to-.config>\n' >&2
  exit 1
fi

if [ ! -f "${CONFIG}" ]; then
  exit 0
fi

if ! grep -q '^CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE=y' "${CONFIG}"; then
  exit 0
fi

# Apply a Kconfig string password if the symbol is empty and ENVVAL is set.

apply_password() {
  symbol=$1
  envval=$2
  envname=$3
  cur=$(grep -E "^${symbol}=" "${CONFIG}" 2>/dev/null \
    | tail -n 1 \
    | sed 's/^[^=]*=//' \
    | tr -d '"')
  if [ -n "${cur}" ]; then
    return 0
  fi

  if [ -z "${envval}" ]; then
    return 0
  fi

  if [ "${#envval}" -lt 8 ]; then
    printf 'update_romfs_password: %s must be at least 8 characters\n' \
      "${envname}" >&2
    exit 1
  fi

  if command -v kconfig-tweak >/dev/null 2>&1; then
    kconfig-tweak --file "${CONFIG}" --set-str "${symbol}" "${envval}"
  else
    sed -i.bak -e "/^${symbol}=/d" "${CONFIG}"
    rm -f "${CONFIG}.bak"
    printf '%s="%s"\n' "${symbol}" "${envval}" >> "${CONFIG}"
  fi
}

apply_password CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD \
  "${PASSWD_ENV}" NUTTX_ROMFS_PASSWD_PASSWORD

if grep -q '^CONFIG_BOARD_ETC_ROMFS_PASSWD_EXTRA_ENABLE=y' "${CONFIG}" &&
   ! grep -q '^CONFIG_BOARD_ETC_ROMFS_PASSWD_EXTRA_USE_ROOT_PASSWORD=y' \
     "${CONFIG}"; then
  apply_password CONFIG_BOARD_ETC_ROMFS_PASSWD_EXTRA_PASSWORD \
    "${NUTTX_ROMFS_PASSWD_EXTRA_PASSWORD:-}" \
    NUTTX_ROMFS_PASSWD_EXTRA_PASSWORD
fi
