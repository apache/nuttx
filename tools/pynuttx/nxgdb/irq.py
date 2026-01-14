############################################################################
# tools/pynuttx/nxgdb/irq.py
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

from . import utils
from .protocols import irq as p

g_irqvector = utils.parse_and_eval("g_irqvector")
NR_IRQS = utils.nitems(g_irqvector)
CONFIG_SCHED_IRQMONITOR = utils.has_field(g_irqvector, "count")


class IRQInfo(utils.Value, p.IRQInfo):
    def __init__(self, irq: gdb.Value):
        super().__init__(irq)

    @property
    def count(self) -> int:
        return self["count"] if CONFIG_SCHED_IRQMONITOR else -1

    @property
    def time(self) -> int:
        return self["time"] if CONFIG_SCHED_IRQMONITOR else -1

    @property
    def start(self) -> int:
        return self["start"] if CONFIG_SCHED_IRQMONITOR else -1


def get_irqs() -> List[IRQInfo]:
    return (IRQInfo(irq) for irq in utils.ArrayIterator(g_irqvector))


class IRQInfoDump(gdb.Command):
    """Dump irqinfo"""

    formatter = "{:<4} {:<10} {:<6} {:<6} {:<48} {} "
    header = ("IRQ", "COUNT", "TIME", "RATE", "HANDLER", "ARGUMENT")

    def __init__(self):
        super().__init__("irqinfo", gdb.COMMAND_USER)

    def invoke(self, arg: str, from_tty: bool) -> None:
        irq_unexpected_isr = utils.gdb_eval_or_none("irq_unexpected_isr")

        print(self.formatter.format(*self.header))
        for i, irq in enumerate(get_irqs()):
            if not irq.handler or (int(irq.handler) & ~0x01) == irq_unexpected_isr:
                continue

            handler = irq.handler.format_string(styling=True, address=False).strip("<>")
            irq_arg = irq.arg.format_string(styling=True)

            print(
                self.formatter.format(i, irq.count, irq.time, "N/A", handler, irq_arg)
            )
