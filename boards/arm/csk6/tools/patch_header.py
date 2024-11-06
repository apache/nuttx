#!/usr/bin/env python3
############################################################################
# boards/arm/csk6/tools/patch_header.py
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
############################################################################

import argparse

_IMAGE_START = 192
_IMAGE_SIZE_OFFSET = _IMAGE_START + 4
_IMAGE_HDR_CHK_OFFSET = _IMAGE_START + 60
_IMAGE_HEADER_SIZE = _IMAGE_HDR_CHK_OFFSET + 4

DEFAULT_INPUT_FILE = "nuttx.bin"
DEFAULT_OUTPUT_FILE = "nuttx.bin"


def parse_args():
    parser = argparse.ArgumentParser(description="Patch binary file header.")
    parser.add_argument(
        "--input",
        help=f"The binary file to be patched (default: {DEFAULT_INPUT_FILE})",
        default=DEFAULT_INPUT_FILE,
    )
    parser.add_argument(
        "--output",
        help=f"The output patched binary file (default: {DEFAULT_OUTPUT_FILE})",
        default=DEFAULT_OUTPUT_FILE,
    )
    return parser.parse_args()


def _get_header_sum(header_bytes: bytearray):
    header_sum = 0
    for pos in range(_IMAGE_START, _IMAGE_HDR_CHK_OFFSET):
        header_sum += header_bytes[pos]

    vector_sum = 0
    for pos in range(_IMAGE_HDR_CHK_OFFSET):
        vector_sum += header_bytes[pos]
    vector_sum += header_sum & 0xFF
    vector_sum += (header_sum >> 8) & 0xFF

    return header_sum, vector_sum


def patch_header(header_bytes: bytearray, image_size: int):
    header_bytes[_IMAGE_SIZE_OFFSET : _IMAGE_SIZE_OFFSET + 4] = image_size.to_bytes(
        4, "little"
    )

    header_sum, vector_sum = _get_header_sum(header_bytes)
    header_bytes[_IMAGE_HDR_CHK_OFFSET : _IMAGE_HDR_CHK_OFFSET + 2] = (
        header_sum.to_bytes(2, "little")
    )
    header_bytes[_IMAGE_HDR_CHK_OFFSET + 2 : _IMAGE_HDR_CHK_OFFSET + 4] = (
        vector_sum.to_bytes(2, "little")
    )


def patch_bin(input_filename: str, output_filename: str):
    with open(input_filename, "rb") as bin_file:
        content = bytearray(bin_file.read())
        header = content[:_IMAGE_HEADER_SIZE]
        image_size = len(content)
        patch_header(header, image_size)
        content[:_IMAGE_HEADER_SIZE] = header

        with open(output_filename, "wb") as out_file:
            out_file.write(content)
        print(f"csk6 rom boot tag added successfully, file: {output_filename}")


def main():
    args = parse_args()
    patch_bin(args.input, args.output)


if __name__ == "__main__":
    main()
