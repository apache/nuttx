#!/usr/bin/env python3

import os
import re
import sys

apache = r"""
/****************************************************************************
 * PATH
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/"""


def apachize(path, header):
    relpath = os.path.relpath(path, os.environ["TOPDIR"])
    header = re.sub("PATH", relpath, header)

    with open(path) as f:
        s = f.read()
        s = re.sub("(?i)/\*\*\*.+?(?:Copyright).+?\*\*\*+/", header, s, 1, re.DOTALL)
        print(s)


if len(sys.argv) != 2:
    print("Usage: ./apachize.py <file>", file=sys.stderr)
    print(
        "This will replace the license header of the passed file to that of Apache 2.0 and print it to stdout",
        file=sys.stderr,
    )
    sys.exit(2)

if "TOPDIR" not in os.environ:
    print(
        "Please define the TOPDIR environment variable to the full path to nuttx/ root",
        file=sys.stderr,
    )
    sys.exit(2)

apachize(sys.argv[1], apache)
