############################################################################
# tools/gdb/macros.py
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

# NOTE: GDB stores macro information based on the current stack frame's scope,
# including the source file and line number. Therefore, there may be missing
# macro definitions when you are at different stack frames.
#
# To resolve this issue, we need to retrieve all macro information from the ELF file
# then parse and evaluate it by ourselves.
#
# There might be two ways to achieve this, one is to leverage the C preprocessor
# to directly preprocess all the macros interpreted into python constants
# gcc -E -x c -P <file_with_macros> -I/path/to/nuttx/include
#
# While the other way is to leverage the dwarf info stored in the ELF file,
# with -g3 switch, we have a `.debug_macro` section containing all the information
# about the macros.
#
# Currently, we are using the second method.

import os
import re
import subprocess
import tempfile

PUNCTUATORS = [
    "\[",
    "\]",
    "\(",
    "\)",
    "\{",
    "\}",
    "\?",
    ";",
    ",",
    "~",
    "\.\.\.",
    "\.",
    "\-\>",
    "\-\-",
    "\-\=",
    "\-",
    "\+\+",
    "\+\=",
    "\+",
    "\*\=",
    "\*",
    "\!\=",
    "\!",
    "\&\&",
    "\&\=",
    "\&",
    "\/\=",
    "\/",
    "\%\>",
    "%:%:",
    "%:",
    "%=",
    "%",
    "\^\=",
    "\^",
    "\#\#",
    "\#",
    "\:\>",
    "\:",
    "\|\|",
    "\|\=",
    "\|",
    "<<=",
    "<<",
    "<=",
    "<:",
    "<%",
    "<",
    ">>=",
    ">>",
    ">=",
    ">",
    "\=\=",
    "\=",
]


def parse_macro(line, macros, pattern):
    # grep name, value
    # the first group matches the token, the second matches the replacement
    m = pattern.match(line)
    if not m:
        return False

    name, value = m.group(1), m.group(2)

    if name in macros:
        # FIXME: what should we do if we got a redefinition/duplication here?
        # for now I think it's ok just overwrite the old value
        pass

    # emplace, for all undefined macros we evalute it to zero
    macros[name] = value if value else "0"

    return True


def fetch_macro_info(file):
    if not os.path.isfile(file):
        raise FileNotFoundError("No given ELF target found")

    # FIXME: we don't use subprocess here because
    # it's broken on some GDB distribution :(, I haven't
    # found a solution to it.

    with tempfile.NamedTemporaryFile(delete=False) as f1:

        # # os.system(f"readelf -wm {file} > {output}")
        process = subprocess.Popen(
            f"readelf -wm {file}", shell=True, stdout=f1, stderr=subprocess.STDOUT
        )

        process.communicate()
        errcode = process.returncode

        f1.close()

        if errcode != 0:
            return {}

        p = re.compile(".*macro[ ]*:[ ]*([\S]+\(.*?\)|[\w]+)[ ]*(.*)")
        macros = {}

        with open(f1.name, "rb") as f2:
            for line in f2.readlines():
                line = line.decode("utf-8")
                if not line.startswith(" DW_MACRO_define") and not line.startswith(
                    " DW_MACRO_undef"
                ):
                    continue

                if not parse_macro(line, macros, p):
                    print(f"Failed to parse {line}")

    return macros


def split_tokens(expr):
    p = "(" + "|".join(PUNCTUATORS) + ")"
    res = list(
        filter(lambda e: e != "", map(lambda e: e.rstrip().lstrip(), re.split(p, expr)))
    )
    return res


def do_expand(expr, macro_map):
    if expr in PUNCTUATORS:
        return expr

    tokens = split_tokens(expr)

    res = []

    for t in tokens:
        if t not in macro_map:
            res.append(t)
            continue
        res += do_expand(macro_map[t], macro_map)

    return res


# NOTE: Implement a fully functional parser which can
# preprocess all the C macros according to ISO 9899 standard
# may be an overkill, what we really care about are those
# macros that can be evaluated to a constant value.
#
# #define A (B + C + D)
# #define B 1
# #define C 2
# #define D 3
# invoking try_expand('A', macro_map) will give you "(1 + 2 + 3)"
#
# However,
# #define SUM(B,C,D) (B + C + D)
# invoking try_expand('SUM(1,2,3)', macro_map) will give you "SUM(1,2,3)"
#
# We have not implemented this feature as we have not found a practical
# use case for it in our GDB plugin.
#
# However, you can switch to the correct stack frame that has this macro defined
# and let GDB expand and evaluate it for you if you really want to evaluate some very
# complex macros.


def try_expand(expr, macro):
    res = []

    res += do_expand(expr, macro)

    return "".join(res)
