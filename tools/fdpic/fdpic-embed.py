#!/usr/bin/env python3
############################################################################
# tools/fdpic/fdpic-embed
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

#
# fdpic-embed -- turn a built module into a C header the firmware can carry.
#
# The demo apps have to load a module before there is any way to put files
# on the target, so they embed one and write it to the filesystem at run
# time.  This generates that header.
#
#   fdpic-embed libshape.so g_libshape > libshape_bin.h
#
# The second argument is the symbol base: the array is <base> and its
# length is <base>_len.

import os
import sys

LICENSE = """\
/****************************************************************************
 * %s
 *
 * SPDX-License-Identifier: Apache-2.0
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
 ****************************************************************************/
"""


def main(argv):
    if len(argv) not in (3, 4):
        sys.stderr.write("usage: fdpic-embed <module> <symbol> [header]\n")
        return 1

    path = argv[1]
    symbol = argv[2]

    # Pass the header's own path relative to the repository root
    # ("apps/testing/fs/xipfs/foo_bin.h") as the third argument when the
    # result is committed: that is what nxstyle wants on line 2, and the
    # module's basename -- the default -- fails the check.

    header = argv[3] if len(argv) > 3 else os.path.basename(path)

    with open(path, "rb") as f:
        blob = f.read()

    out = sys.stdout
    out.write(LICENSE % header)
    out.write(
        "\n/* Generated from %s -- do not edit.\n *\n"
        " * An FDPIC module, embedded so the demo has something to "
        "load without\n"
        " * needing a filesystem populated from the host first.\n */\n"
        % os.path.basename(path)
    )
    out.write(
        "\n/*****************************************************"
        "***********************\n"
        " * Public Data\n"
        " ****************************************************"
        "************************/\n\n"
    )

    # static, because this is a header that defines data.  More than one app
    # embeds the same module, and with external linkage the two copies
    # collide at link time as soon as both are enabled.

    out.write("static const unsigned char %s[] =\n{\n" % symbol)
    for i in range(0, len(blob), 12):
        end = i + 12
        row = ", ".join("0x%02x" % b for b in blob[i:end])
        out.write("  %s%s\n" % (row, "," if end < len(blob) else ""))
    out.write("};\n\n")
    out.write("static const unsigned int %s_len = %d;\n" % (symbol, len(blob)))

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
