#!/usr/bin/env python3
# tools/hexdump2bin.py
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

"""
Extract hexdump data from a file and convert to binary.

Usage:
    python3 hexdump2bin.py <input_file> <output_file>

The script parses hexdump format like:
    gmon.out at 00029400:
    0000: 07 12 00 00 98 bf 02 00 54 bf 02 00 07 12 00 00 ........T.......
    0010: a0 bf 02 00 b0 72 00 00 07 12 00 00 a8 c6 02 00 .....r..........
"""

import re
import sys


def extract_hex_bytes(input, output):
    """Extract hex bytes from hexdump format and write to binary file."""

    pattern = re.compile(r"^[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{2}\s+)+)")

    with open(input, "r") as f_in, open(output, "wb") as f_out:
        written = 0

        for line in f_in:
            line = line.strip()

            # Skip empty lines and header lines
            if not line or "gmon.out at" in line:
                continue

            # Match hexdump line format
            match = pattern.match(line)
            if match:
                hex_bytes_str = match.group(1)
                # Split hex bytes and convert to binary
                hex_bytes = hex_bytes_str.split()

                for hex_byte in hex_bytes:
                    try:
                        byte_val = int(hex_byte, 16)
                        f_out.write(bytes([byte_val]))
                        written += 1
                    except ValueError:
                        print(
                            f"Warning: Invalid hex byte '{hex_byte}' in line: {line}",
                            file=sys.stderr,
                        )

        print(f"Successfully converted {written} bytes to {output}")


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 hexdump2bin.py <input> <output>")
        print("\nExample:")
        print("  python3 hexdump2bin.py hexdump.log output.bin")
        sys.exit(1)

    input = sys.argv[1]
    output = sys.argv[2]

    try:
        extract_hex_bytes(input, output)
    except FileNotFoundError:
        print(f"Error: Input file '{input}' not found", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
