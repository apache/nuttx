############################################################################
# tools/gdb/nuttxgdb/macros.py
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

import hashlib
import json
import os
import re
import tempfile
import time
from os import path

PUNCTUATORS = [
    r"\[",
    r"\]",
    r"\(",
    r"\)",
    r"\{",
    r"\}",
    r"\?",
    r";",
    r",",
    r"~",
    r"\.\.\.",
    r"\.",
    r"\-\>",
    r"\-\-",
    r"\-\=",
    r"\-",
    r"\+\+",
    r"\+\=",
    r"\+",
    r"\*\=",
    r"\*",
    r"\!\=",
    r"\!",
    r"\&\&",
    r"\&\=",
    r"\&",
    r"\/\=",
    r"\/",
    r"\%\>",
    r"%:%:",
    r"%:",
    r"%=",
    r"%",
    r"\^\=",
    r"\^",
    r"\#\#",
    r"\#",
    r"\:\>",
    r"\:",
    r"\|\|",
    r"\|\=",
    r"\|",
    r"<<=",
    r"<<",
    r"<=",
    r"<:",
    r"<%",
    r"<",
    r">>=",
    r">>",
    r">=",
    r">",
    r"\=\=",
    r"\=",
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
    if not path.isfile(file):
        raise FileNotFoundError("No given ELF target found")

    # FIXME: we don't use subprocess here because
    # it's broken on some GDB distribution :(, I haven't
    # found a solution to it.

    with open(file, "rb") as f:
        hash = hashlib.md5(f.read()).hexdigest()

    macros = {}
    p = re.compile(r".*macro[ ]*:[ ]*([\S]+\(.*?\)|[\w]+)[ ]*(.*)")
    cache = path.join(tempfile.gettempdir(), f"{hash}.json")
    print(f"Load macro: {cache}")
    if not path.isfile(cache):
        t = time.time()
        os.system(f'readelf -wm "{file}" > "{cache}"')
        print(f"readelf took {time.time() - t:.1f} seconds")

        t = time.time()
        with open(cache, "r") as f2:
            for line in f2.readlines():
                if not line.startswith(" DW_MACRO_define") and not line.startswith(
                    " DW_MACRO_undef"
                ):
                    continue

                if not parse_macro(line, macros, p):
                    print(f"Failed to parse {line}")

        print(f"Parse macro took {time.time() - t:.1f} seconds")

        with open(cache, "w") as f2:
            dump = json.dumps(macros, indent=4, sort_keys=True)
            f2.write(dump)

        print(f"Cache macro info to {cache}")
    else:
        with open(cache, "r") as f2:
            macros = json.load(f2)

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


class Macro:
    """
    This is a singleton class which only initializes once to
    cache a context of macro definition which can be queried later
    TODO: we only deal with single ELF at the moment for simplicity
    If you load more object files while debugging, only the first one gets loaded
    will be used to retrieve macro information

    Usage:
        macro = Macro("nuttx/nuttx")
        print(macro.CONFIG_MM_BACKTRACE)
        if macro.CONFIG_MM_BACKTRACE:
            print("mm backtrace is enabled")
        else:
            print("mm backtrace is disabled")
    """

    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, "instance"):
            cls.instance = super(Macro, cls).__new__(cls)
        return cls.instance

    def __init__(self, filename):
        self._macro_map = {}
        self._file = filename
        self._macro_map = fetch_macro_info(filename)

    def is_defined(self, macro_name):
        """
        Check if a macro is defined
        """
        return macro_name in self._macro_map

    def get_value(self, macro_name, default=None):
        """
        Get the value of a macro, return default if macro is not defined
        """
        if not self.is_defined(macro_name):
            return default

        value = self._macro_map[macro_name]
        # Try to convert to numeric type
        try:
            # Handle hexadecimal
            if isinstance(value, str) and value.startswith("0x"):
                return int(value, 16)
            # Handle integer
            return int(value)
        except (ValueError, TypeError):
            # Return original value if conversion fails
            return value

    def __getattr__(self, name):
        """
        Allow using dot notation to access macros
        """
        return self.get_value(name)

    def __call__(self, macro_name):
        """
        Allow using function call syntax to get macro
        """
        return self.get_value(macro_name)
