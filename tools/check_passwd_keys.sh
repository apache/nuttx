#!/usr/bin/env sh
# tools/check_passwd_keys.sh
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
#   check_passwd_keys.sh <path-to-.config>
#
# Prints "yes" to stdout when CONFIG_FSUTILS_PASSWD_KEY1..4 need setup:
#   - any key is absent or "is not set"
#   - all four equal the known-insecure Kconfig defaults
# Prints "no" when all four are present and non-default.
# Idempotent; no side effects.

set -e

CONFIG="${1}"
if [ -z "${CONFIG}" ]; then
  printf 'Usage: check_passwd_keys.sh <path-to-.config>\n' >&2
  exit 1
fi

if [ ! -f "${CONFIG}" ]; then
  printf 'check_passwd_keys: file not found: %s\n' "${CONFIG}" >&2
  exit 1
fi

# Known-insecure placeholder/default values — must not be used in a build.
DEFAULT1=0x12345678
DEFAULT2=0x9abcdef0
DEFAULT3=0x12345678
DEFAULT4=0x9abcdef0

# Return the assigned value of CONFIG_FSUTILS_PASSWD_KEY<n>, or empty
# string when the line is absent or commented as "is not set".
get_val() {
  grep -E "^CONFIG_FSUTILS_PASSWD_KEY${1}=" "${CONFIG}" 2>/dev/null \
    | tail -n 1 \
    | sed 's/^[^=]*=//'
}

# True when the value is unset or still at the Kconfig placeholder (0).
is_placeholder() {
  case "$1" in
  ''|0|0x0|0x00000000) return 0 ;;
  *) return 1 ;;
  esac
}

K1=$(get_val 1)
K2=$(get_val 2)
K3=$(get_val 3)
K4=$(get_val 4)

# Any key absent, empty, or placeholder → needs setup
if is_placeholder "${K1}" || is_placeholder "${K2}" || \
   is_placeholder "${K3}" || is_placeholder "${K4}"; then
  echo yes
  exit 0
fi

# All four at the known insecure defaults → needs setup
if [ "${K1}" = "${DEFAULT1}" ] && [ "${K2}" = "${DEFAULT2}" ] && \
   [ "${K3}" = "${DEFAULT3}" ] && [ "${K4}" = "${DEFAULT4}" ]; then
  echo yes
  exit 0
fi

echo no
