############################################################################
# tools/pynuttx/nxgdb/wdog.py
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

from __future__ import annotations

from typing import List

import gdb

from . import lists, utils
from .protocols import wdog as p
from .utils import Value


class WDog(Value, p.WDog):
    def __init__(self, wdog: p.WDog):
        if wdog.type.code == gdb.TYPE_CODE_PTR:
            wdog = wdog.dereference()
        super().__init__(wdog)

    def __repr__(self) -> str:
        return (
            f"WDog@{self.address:#x}: tick: {self.expired: <16}"
            f" {self.func.format_string(styling=True)} arg: {self.arg:#x}"
        )

    def __str__(self) -> str:
        return self.__repr__()


def get_wdog_list() -> List[WDog]:
    wdogs = []
    active = utils.parse_and_eval("g_wdactivelist")
    for wdog in lists.NxList(active, "struct wdog_s", "node"):
        wdogs.append(WDog(wdog))

    return wdogs


class WDogDump(gdb.Command):
    """Show wdog timer information"""

    def __init__(self):
        super().__init__("wdog", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        for wdog in get_wdog_list():
            print(wdog)
