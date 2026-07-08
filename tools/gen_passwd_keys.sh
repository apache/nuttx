#!/usr/bin/env sh
# tools/gen_passwd_keys.sh
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
#   gen_passwd_keys.sh <path-to-.config>
#
# Generates four random 32-bit TEA key values from /dev/urandom and
# writes/replaces CONFIG_FSUTILS_PASSWD_KEY1..4 in the target .config file.
# Handles both "line exists (replace)" and "line missing (append)" cases.
#
# Key values are NOT printed to stdout or stderr.  Search the .config file
# for CONFIG_FSUTILS_PASSWD_KEY to view them if needed.
#
# Exit 0 on success, non-zero with a message on failure.

set -e

CONFIG="${1}"
if [ -z "${CONFIG}" ]; then
  printf 'Usage: gen_passwd_keys.sh <path-to-.config>\n' >&2
  exit 1
fi

if [ ! -f "${CONFIG}" ]; then
  printf 'gen_passwd_keys: file not found: %s\n' "${CONFIG}" >&2
  exit 1
fi

# Generate a random 32-bit hex value via /dev/urandom.
# Uses od -tx4 which always produces exactly 8 hex digits.
rand_key() {
  dd if=/dev/urandom bs=4 count=1 2>/dev/null \
    | od -An -tx4 \
    | tr -d ' \n'
}

K1="0x$(rand_key)"
K2="0x$(rand_key)"
K3="0x$(rand_key)"
K4="0x$(rand_key)"

# Remove any existing definitions (assigned or "is not set") everywhere.
sed -i \
  '/^# CONFIG_FSUTILS_PASSWD_KEY[1-4] is not set/d;
   /^CONFIG_FSUTILS_PASSWD_KEY[1-4]=/d' \
  "${CONFIG}" || {
  printf 'gen_passwd_keys: ERROR: failed to edit %s (check permissions)\n' \
    "${CONFIG}" >&2
  exit 1
}

# Insert keys in the FSUTILS_PASSWD section when it exists so menuconfig
# and mkconfig see a single canonical copy (not orphaned lines at EOF).
if grep -q '^CONFIG_FSUTILS_PASSWD_IOBUFFER_SIZE=' "${CONFIG}"; then
  sed -i "/^CONFIG_FSUTILS_PASSWD_IOBUFFER_SIZE=/a\\
CONFIG_FSUTILS_PASSWD_KEY1=${K1}\\
CONFIG_FSUTILS_PASSWD_KEY2=${K2}\\
CONFIG_FSUTILS_PASSWD_KEY3=${K3}\\
CONFIG_FSUTILS_PASSWD_KEY4=${K4}" "${CONFIG}" || {
    printf 'gen_passwd_keys: ERROR: failed to insert keys into %s\n' \
      "${CONFIG}" >&2
    exit 1
  }
else
  printf 'CONFIG_FSUTILS_PASSWD_KEY1=%s\nCONFIG_FSUTILS_PASSWD_KEY2=%s\nCONFIG_FSUTILS_PASSWD_KEY3=%s\nCONFIG_FSUTILS_PASSWD_KEY4=%s\n' \
    "${K1}" "${K2}" "${K3}" "${K4}" >> "${CONFIG}" || {
    printf 'gen_passwd_keys: ERROR: failed to append keys to %s\n' \
      "${CONFIG}" >&2
    exit 1
  }
fi

# User-visible notice (stderr): key values are never printed.
printf 'WARNING: [passwd] TEA keys auto-generated in %s\n' "${CONFIG}" >&2
printf 'WARNING: [passwd] View: search .config for CONFIG_FSUTILS_PASSWD_KEY\n' >&2
printf 'WARNING: [passwd] Change: make menuconfig -> Application Configuration\n' >&2
printf 'WARNING: [passwd]         -> File System Utilities -> Password file support\n' >&2
