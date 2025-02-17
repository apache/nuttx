#!/usr/bin/env python3
############################################################################
# tools/coredump.py
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
import base64
import binascii
import mmap
import os
import shutil
import struct
import sys

import lzf


def decompress(lzffile, outfile):
    chunk_number = 1

    while True:
        prefix = lzffile.read(2)

        if len(prefix) == 0:
            break
        elif prefix != b"ZV":
            break

        typ = lzffile.read(1)
        clen = struct.unpack(">H", lzffile.read(2))[0]

        if typ == b"\x00":
            chunk = lzffile.read(clen)
        elif typ == b"\x01":
            uncompressed_len = struct.unpack(">H", lzffile.read(2))[0]
            cdata = lzffile.read(clen)
            chunk = lzf.decompress(cdata, uncompressed_len)
        else:
            return

        outfile.write(chunk)
        chunk_number += 1


def unhexlify(infile, outfile):
    while True:
        line = infile.readline()
        if not line:
            break
        line = line.replace(b"\n", b"").strip()
        if line == b"":
            continue
        index = line.rfind(b" ")
        if index > 0:
            line = line[index + 1 :]

        outfile.write(binascii.unhexlify(line))


def unbase64file(infile, outfile):
    while True:
        line = infile.readline()
        if not line:
            break
        line = line.replace(b"\n", b"").strip()
        if line == b"":
            continue
        index = line.rfind(b" ")
        if index > 0:
            line = line[index + 1 :]

        outfile.write(base64.b64decode(line))


def mmap_file(file_path, size=None):
    with open(file_path, "rb") as f:
        fd = f.fileno()
        if size is None:
            size = os.fstat(fd).st_size
        return mmap.mmap(fd, size, access=mmap.ACCESS_READ)


def parse_args():
    global args
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False,
    )
    parser.add_argument("input")
    parser.add_argument("-o", "--output", help="Output file in hex.")
    parser.add_argument(
        "--base64",
        action="store_true",
        default=False,
        help="Set when input file is base64 encoded.",
    )
    parser.add_argument(
        "-b",
        "--binary",
        action="store_true",
        default=False,
        help="Treat input file as binary data and write directly to output.",
    )
    parser.add_argument(
        "--size",
        type=int,
        default=None,
        help="Size of memory to dump (default: mmap whole file).",
    )
    args = parser.parse_args()


def main():
    parse_args()

    if not os.path.exists(args.input):
        print(f"Error: Input file {args.input} does not exist.")
        sys.exit(1)

    tmp = os.path.splitext(args.input)[0] + ".tmp"

    if args.output is None:
        args.output = os.path.splitext(args.input)[0] + ".core"

    try:
        infile = mmap_file(args.input, args.size)
    except Exception as e:
        print(f"Failed to mmap input file: {e}")
        sys.exit(1)

    tmpfile = open(tmp, "wb+")

    if args.binary:
        tmpfile.write(infile)
    elif args.base64:
        unbase64file(infile, tmpfile)
    else:
        unhexlify(infile, tmpfile)

    infile.close()

    tmpfile.seek(0, 0)

    lzfhdr = tmpfile.read(2)

    if lzfhdr == b"ZV":
        outfile = open(args.output, "wb")
        tmpfile.seek(0, 0)
        decompress(tmpfile, outfile)
        tmpfile.close()
        outfile.close()
        os.unlink(tmp)
    else:
        tmpfile.close()
        shutil.copy(tmp, args.output)
        os.unlink(tmp)

    print("Core file conversion completed: " + args.output)


if __name__ == "__main__":
    main()
