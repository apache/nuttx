#!/usr/bin/env python3
############################################################################
# tools/mkpasswd.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
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

"""Generate a NuttX /etc/passwd entry with a TEA-encrypted password hash.

This script implements the same Tiny Encryption Algorithm (TEA) and base64
encoding used by NuttX at runtime (libs/libc/misc/lib_tea_encrypt.c and
apps/fsutils/passwd) to produce a passwd line suitable for embedding in a
ROMFS image at build time.

Usage:
    python3 mkpasswd.py --user admin --password secret
    python3 mkpasswd.py --user admin --password secret -o /path/to/passwd
"""

import argparse
import os
import struct
import sys

# The TEA key schedule constant (0x9e3779b9 is the golden ratio derived value)
TEA_KEY_SCHEDULE_CONSTANT = 0x9E3779B9
MASK32 = 0xFFFFFFFF


def tea_encrypt(v0, v1, key):
    """Encrypt two 32-bit values using TEA with the given 128-bit key.

    This matches the C implementation in libs/libc/misc/lib_tea_encrypt.c.
    """
    sum_val = 0
    for _ in range(32):
        sum_val = (sum_val + TEA_KEY_SCHEDULE_CONSTANT) & MASK32
        v0 = (v0 + ((((v1 << 4) & MASK32) + key[0]) ^ ((v1 + sum_val) & MASK32) ^ (((v1 >> 5) & MASK32) + key[1]))) & MASK32
        v1 = (v1 + ((((v0 << 4) & MASK32) + key[2]) ^ ((v0 + sum_val) & MASK32) ^ (((v0 >> 5) & MASK32) + key[3]))) & MASK32
    return v0, v1


def passwd_base64(binary):
    """Encode a 6-bit value as a base64 character.

    This matches the custom base64 encoding in apps/fsutils/passwd/passwd_encrypt.c.
    The mapping is: A-Z (0-25), a-z (26-51), 0-9 (52-61), + (62), / (63).
    """
    binary &= 63
    if binary < 26:
        return chr(ord("A") + binary)
    binary -= 26
    if binary < 26:
        return chr(ord("a") + binary)
    binary -= 26
    if binary < 10:
        return chr(ord("0") + binary)
    binary -= 10
    if binary == 0:
        return "+"
    return "/"


def hash_password(password, key):
    """Hash a password string using TEA encryption, returning a custom base64 string.

    This replicates the exact algorithm from apps/fsutils/passwd/passwd_encrypt.c:
    1. Process password in 8-byte chunks, padding short chunks with spaces.
    2. TEA-encrypt each chunk as two uint32_t values (little-endian).
    3. Read the encrypted result as four uint16_t values (little-endian).
    4. Stream-encode the uint16_t values into custom base64 (6 bits at a time).
    """
    pwd_bytes = password.encode("utf-8")
    remaining = len(pwd_bytes)
    src_offset = 0
    dest = []
    remainder = 0
    nbits = 0

    while remaining > 0:
        gulpsize = min(8, remaining)
        # Copy bytes into 8-byte block, pad with spaces
        block = pwd_bytes[src_offset : src_offset + gulpsize]
        block += b" " * (8 - gulpsize)
        src_offset += gulpsize
        remaining -= gulpsize

        # TEA encrypt (interpret as two little-endian uint32_t)
        v0, v1 = struct.unpack("<II", block)
        v0, v1 = tea_encrypt(v0, v1, key)
        encrypted_block = struct.pack("<II", v0, v1)

        # Read as four little-endian uint16_t values
        h = struct.unpack("<HHHH", encrypted_block)

        # Stream base64 encoding
        tmp = remainder
        for i in range(4):
            tmp = (h[i] << nbits) | tmp
            nbits += 16
            while nbits >= 6:
                dest.append(passwd_base64(tmp & 0x3F))
                tmp >>= 6
                nbits -= 6
        remainder = tmp & 0xFF

    # Handle any remaining bits
    if nbits > 0:
        dest.append(passwd_base64(remainder))

    return "".join(dest)


def parse_hex(value):
    """Parse a hex string (with or without 0x prefix) as a 32-bit unsigned int."""
    return int(value, 0) & MASK32


def main():
    parser = argparse.ArgumentParser(
        description="Generate a NuttX /etc/passwd entry with TEA-encrypted password."
    )
    parser.add_argument(
        "--user", required=True, help="Username for the passwd entry."
    )
    parser.add_argument(
        "--password", required=True, help="Plaintext password to encrypt."
    )
    parser.add_argument(
        "--uid", type=int, default=0, help="User ID (default: 0)."
    )
    parser.add_argument(
        "--gid", type=int, default=0, help="Group ID (default: 0)."
    )
    parser.add_argument(
        "--home", default="/", help="Home directory (default: /)."
    )
    parser.add_argument(
        "--key1",
        type=parse_hex,
        default=0x12345678,
        help="TEA key word 1 (default: 0x12345678).",
    )
    parser.add_argument(
        "--key2",
        type=parse_hex,
        default=0x9ABCDEF0,
        help="TEA key word 2 (default: 0x9abcdef0).",
    )
    parser.add_argument(
        "--key3",
        type=parse_hex,
        default=0x12345678,
        help="TEA key word 3 (default: 0x12345678).",
    )
    parser.add_argument(
        "--key4",
        type=parse_hex,
        default=0x9ABCDEF0,
        help="TEA key word 4 (default: 0x9abcdef0).",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output file path. If omitted, prints to stdout.",
    )

    args = parser.parse_args()

    if not args.password:
        print("ERROR: --password must not be empty.", file=sys.stderr)
        sys.exit(1)

    key = [args.key1, args.key2, args.key3, args.key4]
    encrypted = hash_password(args.password, key)
    line = f"{args.user}:{encrypted}:{args.uid}:{args.gid}:{args.home}\n"

    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, "w") as f:
            f.write(line)
    else:
        sys.stdout.write(line)


if __name__ == "__main__":
    main()
