#!/usr/bin/env python3
############################################################################
# tools/coredump.py
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
import binascii
import os
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
    for line in infile.readlines():
        line = line.strip()
        if line == "":
            break
        index = line.rfind(" ")
        if index > 0:
            line = line[index + 1 :]

        outfile.write(binascii.unhexlify(line))


def parse_args():
    global args
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False,
    )
    parser.add_argument("input")
    parser.add_argument("-o", "--output", help="Output file in hex.")
    args = parser.parse_args()


def main():
    parse_args()
    if not os.path.isfile(args.input):
        sys.exit(1)

    tmp = os.path.splitext(args.input)[0] + ".tmp"

    if args.output is None:
        args.output = os.path.splitext(args.input)[0] + ".core"

    infile = open(args.input, "r")
    tmpfile = open(tmp, "wb+")

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
        tmpfile.rename(args.output)
        tmpfile.close()

    print("Core file conversion completed: " + args.output)


if __name__ == "__main__":
    main()
