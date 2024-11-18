#!/usr/bin/env python3
############################################################################
# tools/gcov_convert.py
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

import argparse
import re

# Parse gcda data from stdout dump format to binary format
# The stdout dump format is:
# gcov start filename:<filename> size: <size>Byte
# <hex dump data>
# gcov end filename:<filename> checksum: <checksum>
#
# The hex dump data will be converted to binary and written to <filename>
# The checksum is calculated by summing all bytes and taking modulo 65536


def parse_gcda_data(input_file):
    with open(input_file, "r") as file:
        lines = file.read().strip().splitlines()

    for line in lines:
        if not line.startswith("gcov start"):
            continue

        match = re.search(r"filename:(.*?)\s+size:\s*(\d+)Byte", line)
        if not match:
            continue

        hex_dump = ""
        filename = match.group(1)
        size = int(match.group(2))

        # Read the hex dump until the end of the file
        next_line_index = lines.index(line) + 1
        while next_line_index < len(lines) and not lines[next_line_index].startswith(
            "gcov end"
        ):
            hex_dump += lines[next_line_index].strip()
            next_line_index += 1

        if size != len(hex_dump) // 2:
            print(
                f"Size mismatch for {filename}: expected {size} bytes, got {len(hex_dump) // 2} bytes"
            )

        checksum_match = (
            re.search(r"checksum:\s*(0x[0-9a-fA-F]+)", lines[next_line_index])
            if next_line_index < len(lines)
            else None
        )
        if not checksum_match:
            continue

        checksum = int(checksum_match.group(1), 16)
        calculated_checksum = sum(bytearray.fromhex(hex_dump)) % 65536
        if calculated_checksum != checksum:
            print(
                f"Checksum mismatch for {filename}: expected {checksum}, got {calculated_checksum}"
            )
            continue

        with open(filename, "wb") as fp:
            fp.write(bytes.fromhex(hex_dump))
            print(f"write {filename} success")

    print(
        "Execute the 'nuttx/tools/gcov.sh -t arm-none-eabi-gcov' command to view the coverage report"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="Input dump data")
    args = parser.parse_args()

    parse_gcda_data(args.input)
