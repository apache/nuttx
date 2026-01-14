#!/usr/bin/env bash
############################################################################
# tools/btdecode.sh
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
############################################################################

# This script can be used to decode the backtrace that's dumped on assertions.
#
# On assertions we can find the raw backtrace dump similar to:
# ...
# sched_dumpstack: backtrace| 0: 0x400e1a2a 0x40082912
# sched_dumpstack: backtrace| 1: 0x400e39ac 0x400ef7c3 0x400ef7fc 0x400e8116 0x400e7910 0x400e7be8 0x400e6c5c 0x400e6ad6
# sched_dumpstack: backtrace| 1: 0x400e6a99 0x400e4005 0x400e2754
# sched_dumpstack: backtrace| 2: 0x400f13ee 0x400e4005 0x400e2754
# ...
#
# Copy that to a file and call this script as:
#    ./tools/btdecode.sh esp32 backtrace_file
#
# The result should be similar to the following:
#    0x400e1a2a: function_name at file.c:line
#    0x40082912: function_name at file.c:line

USAGE="USAGE: ${0} chip|toolchain-addr2line backtrace_file [elf_file]
If the first argument contains 'addr2line', it will be used as the toolchain's addr2line tool.
Otherwise, the script will try to identify the toolchain based on the chip name."

GREP=${GREP:-grep}

VALID_CHIPS=(
  "esp32"
  "esp32s2"
  "esp32s3"
  "esp32c3"
  "esp32c6"
  "esp32h2"
)

# Make sure we have the required argument(s)

if [ -z "$2" ]; then
  echo "No backtrace supplied!"
  echo "$USAGE"
  exit 1
fi

elf_file="nuttx"

if [ -n "$3" ]; then
  elf_file=$3
fi

# Check if the first argument is an addr2line tool or a chip

chip_or_tool=$1
if [[ $chip_or_tool == *addr2line* ]]; then
  ADDR2LINE_TOOL=$chip_or_tool
else
  chip=$chip_or_tool
  if [[ ! " ${VALID_CHIPS[@]} " =~ " ${chip} " ]]; then
    echo "Invalid chip specified! Valid options are: ${VALID_CHIPS[*]}"
    echo "$USAGE"
    exit 4
  fi

  # Set the appropriate addr2line tool based on the chip
  case $chip in
    esp32)
      ADDR2LINE_TOOL="xtensa-esp32-elf-addr2line"
      ;;
    esp32s2)
      ADDR2LINE_TOOL="xtensa-esp32s2-elf-addr2line"
      ;;
    esp32s3)
      ADDR2LINE_TOOL="xtensa-esp32s3-elf-addr2line"
      ;;
    esp32c3)
      ADDR2LINE_TOOL="riscv-none-elf-addr2line"
      ;;
    esp32c6)
      ADDR2LINE_TOOL="riscv-none-elf-addr2line"
      ;;
    esp32h2)
      ADDR2LINE_TOOL="riscv-none-elf-addr2line"
      ;;
  esac
fi

# Make sure the elf file is accessible

if [ ! -f ${elf_file} ]; then
  echo "NuttX binaries not found!"
  exit 2
fi

# Check that the toolchain is in the PATH

if [ ! -x "$(command -v $ADDR2LINE_TOOL)" ]; then
  echo "Toolchain for $chip_or_tool not found!"
  exit 3
fi

# Decode backtrace

declare -A backtraces_before
declare -A backtraces_after
in_dump_tasks_section=false

while read -r line; do
  if [[ $line =~ (\[CPU[0-9]+\]\ )?dump_tasks: ]]; then
    in_dump_tasks_section=true
  fi

  if [[ $line =~ (\[CPU[0-9]+\]\ )?sched_dumpstack: ]]; then
    task_id=$(echo $line | ${GREP} -oP 'backtrace\|\s*\K\d+')
    addresses=$(echo $line | ${GREP} -oP '0x[0-9a-fA-F]+')
    if $in_dump_tasks_section; then
      if [[ -n "${backtraces_after[$task_id]}" ]]; then
        backtraces_after[$task_id]="${backtraces_after[$task_id]} $addresses"
      else
        backtraces_after[$task_id]="$addresses"
      fi
    else
      if [[ -n "${backtraces_before[$task_id]}" ]]; then
        backtraces_before[$task_id]="${backtraces_before[$task_id]} $addresses"
      else
        backtraces_before[$task_id]="$addresses"
      fi
    fi
  fi
done < "$2"

for task_id in "${!backtraces_before[@]}"; do
  echo "Backtrace for task $task_id:"
  for bt in ${backtraces_before[$task_id]}; do
    $ADDR2LINE_TOOL -pfiaCs -e ${elf_file} $bt
  done
  echo ""
done

if $in_dump_tasks_section; then
  echo "Backtrace dump for all tasks:"
  echo ""
  for task_id in "${!backtraces_after[@]}"; do
    echo "Backtrace for task $task_id:"
    for bt in ${backtraces_after[$task_id]}; do
      $ADDR2LINE_TOOL -pfiaCs -e ${elf_file} $bt
    done
    echo ""
  done
fi
