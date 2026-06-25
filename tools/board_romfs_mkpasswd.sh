#!/usr/bin/env bash
# tools/board_romfs_mkpasswd.sh
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
# Ensure the ROMFS root password is configured, then run mkpasswd.
# Arguments:
#   board_romfs_mkpasswd.sh <nuttx-topdir> <passfile> <mkpasswd> <output> [mkpasswd args...]

set -e

TOPDIR=$1
PASSFILE=$2
MKPASSWD=$3
OUTPUT=$4
shift 4

CONFIG_FILE="${TOPDIR}/.config"

read_int_config() {
  local symbol=$1
  local default=$2
  local value

  value=$(grep "^${symbol}=" "${CONFIG_FILE}" 2>/dev/null | cut -d= -f2- | tr -d '"')
  if [ -z "${value}" ]; then
    echo "${default}"
  else
    echo "${value}"
  fi
}

ITERATIONS=$(read_int_config CONFIG_FSUTILS_PASSWD_PBKDF2_ITERATIONS 10000)

"${TOPDIR}/tools/promptpasswd.sh" \
  --min 8 \
  --config CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD \
  --config-file "${CONFIG_FILE}" \
  --update-config \
  --prompt "ROMFS root password (min 8 characters): " \
  --output-file "${PASSFILE}"

PASSWORD=$(cat "${PASSFILE}")
"${MKPASSWD}" --password "${PASSWORD}" \
  --iterations "${ITERATIONS}" \
  "$@" -o "${OUTPUT}"
rm -f "${PASSFILE}"
