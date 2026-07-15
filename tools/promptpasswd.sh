#!/usr/bin/env bash
# tools/promptpasswd.sh
#
# SPDX-License-Identifier: Apache-2.0
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
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Prompt for a Kconfig password when it is unset or invalid.  If no
# terminal is available, print an error and exit.
#
# Usage:
#   promptpasswd.sh --min <n> [--config <symbol>] [--config-file <file>]
#                   [--update-config] [--prompt <text>] [--output-file <file>]

set -e

MIN=8
VALUE=""
PROMPT="Password: "
CONFIG_SYMBOL=""
CONFIG_FILE=".config"
UPDATE_CONFIG=0
OUTPUT_FILE=""

while [ $# -gt 0 ]; do
  case "$1" in
    --min)
      MIN=$2
      shift 2
      ;;
    --value)
      VALUE=$2
      shift 2
      ;;
    --prompt)
      PROMPT=$2
      shift 2
      ;;
    --config)
      CONFIG_SYMBOL=$2
      shift 2
      ;;
    --config-file)
      CONFIG_FILE=$2
      shift 2
      ;;
    --update-config)
      UPDATE_CONFIG=1
      shift
      ;;
    --output-file)
      OUTPUT_FILE=$2
      shift 2
      ;;
    *)
      echo "promptpasswd.sh: unknown option: $1" >&2
      exit 1
      ;;
  esac
done

validate_password() {
  local pw="$1"
  local ok=0

  if [ ${#pw} -lt "${MIN}" ]; then
    echo "Error: password must be at least ${MIN} characters" >&2
    ok=1
  fi

  if ! printf '%s' "$pw" | grep -q '[A-Z]'; then
    echo "Error: password must contain at least one uppercase letter (A-Z)" >&2
    ok=1
  fi

  if ! printf '%s' "$pw" | grep -q '[a-z]'; then
    echo "Error: password must contain at least one lowercase letter (a-z)" >&2
    ok=1
  fi

  if ! printf '%s' "$pw" | grep -q '[0-9]'; then
    echo "Error: password must contain at least one digit (0-9)" >&2
    ok=1
  fi

  if ! printf '%s' "$pw" | grep -q '[^a-zA-Z0-9]'; then
    echo "Error: password must contain at least one special character" \
         "(!@#\$%^&*()_+-=[]{}|;:,.<>?)" >&2
    ok=1
  fi

  return "${ok}"
}

if [ -n "${CONFIG_SYMBOL}" ] && [ -z "${VALUE}" ] && [ -f "${CONFIG_FILE}" ]; then
  VALUE=$(grep "^${CONFIG_SYMBOL}=" "${CONFIG_FILE}" 2>/dev/null | cut -d= -f2- | tr -d '"')
fi

if [ -n "${VALUE}" ] && validate_password "${VALUE}"; then
  if [ -n "${OUTPUT_FILE}" ]; then
    umask 077
    printf '%s' "${VALUE}" > "${OUTPUT_FILE}"
  else
    printf '%s' "${VALUE}"
  fi
  exit 0
fi

# Make recipe shells are not connected to the terminal on stdin, so test /dev/tty
# instead of [ -t 0 ] when deciding whether an interactive prompt is possible.

INTERACTIVE=0
if [ -r /dev/tty ] && [ -w /dev/tty ]; then
  INTERACTIVE=1
fi

if [ "${INTERACTIVE}" -eq 0 ]; then
  echo "" >&2
  if [ -n "${CONFIG_SYMBOL}" ]; then
    echo "ERROR: ${CONFIG_SYMBOL} must be at least ${MIN} characters and" >&2
    echo "contain uppercase, lowercase, digit, and special character." >&2
  else
    echo "ERROR: Password must be at least ${MIN} characters and contain" >&2
    echo "uppercase, lowercase, digit, and special character." >&2
  fi
  echo "Set it with 'make menuconfig' or edit .config, then rebuild." >&2
  exit 1
fi

PASSWORD=""
while true; do
  printf '%s' "${PROMPT}" >/dev/tty
  IFS= read -r -s PASSWORD </dev/tty
  echo "" >/dev/tty
  if ! validate_password "${PASSWORD}"; then
    echo "Please try again." >&2
    PASSWORD=""
    continue
  fi

  while true; do
    printf 'Confirm password: ' >/dev/tty
    IFS= read -r -s PASSWORD2 </dev/tty
    echo "" >/dev/tty
    if [ "${PASSWORD}" = "${PASSWORD2}" ]; then
      break
    fi
    echo "Passwords do not match. Please try again." >&2
    PASSWORD=""
    break
  done

  if [ -n "${PASSWORD}" ]; then
    break
  fi
done

if [ "${UPDATE_CONFIG}" -eq 1 ] && [ -n "${CONFIG_SYMBOL}" ]; then
  kconfig-tweak --file "${CONFIG_FILE}" --set-str "${CONFIG_SYMBOL}" "${PASSWORD}"
fi

if [ -n "${OUTPUT_FILE}" ]; then
  umask 077
  printf '%s' "${PASSWORD}" > "${OUTPUT_FILE}"
else
  printf '%s' "${PASSWORD}"
fi
