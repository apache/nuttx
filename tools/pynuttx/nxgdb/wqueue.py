############################################################################
# tools/pynuttx/nxgdb/wqueue.py
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
from .protocols import wqueue as p
from .utils import Value


class Work(Value, p.Work):
    def __init__(self, work: p.Work):
        if work.type.code == gdb.TYPE_CODE_PTR:
            work = work.dereference()
        super().__init__(work)

    @property
    def wqueue(self) -> WorkQueue:
        return WorkQueue(self.wq)

    def __repr__(self) -> str:
        return f"work_s@{self.address:#x}: {self.worker.format_string(styling=True)} arg={self.arg}"

    def __str__(self) -> str:
        return self.__repr__()


class KWorker(Value, p.KWorker):
    """Worker thread information"""

    def __init__(self, worker: p.KWorker, wqueue=None):
        if worker.type.code == gdb.TYPE_CODE_PTR:
            worker = worker.dereference()
        super().__init__(worker)
        self.wqueue = wqueue

    @property
    def is_running(self):
        return self.work

    @property
    def work(self) -> Work:
        work = self["work"]
        return Work(work) if work else None

    @property
    def name(self):
        return utils.get_task_name(utils.get_tcb(self.pid))

    def __repr__(self):
        return f"kworker_s@{self.address:#x} {self.work or 'idle'}"

    def __str__(self):
        return self.__repr__()


class WorkQueue(Value, p.KWorkQueue):
    """Work queue information"""

    def __init__(self, queue: p.KWorkQueue, name: str = None):
        if queue.type.code == gdb.TYPE_CODE_PTR:
            queue = queue.dereference()
        super().__init__(queue)
        self.name = name or "<noname>"

    @property
    def workers(self) -> List[Work]:
        work_s = utils.lookup_type("struct work_s")
        return [
            Work(worker.cast(work_s.pointer())) for worker in lists.NxDQueue(self.q)
        ]

    @property
    def threads(self) -> List[KWorker]:
        return [
            KWorker(thread, wqueue=self)
            for thread in utils.ArrayIterator(self.worker, self.nthreads)
        ]

    @property
    def nthreads(self):
        return int(self["nthreads"])

    @property
    def is_running(self):
        return any(thread.is_running for thread in self.threads)

    @property
    def is_exiting(self):
        return self.exit

    def __repr__(self):
        state = "running" if self.is_running else "idle"
        return f"{self.name}@{self.address:#x}, {state}, {self.nthreads} threads, {len(self.workers)} work"

    def __str__(self):
        return self.__repr__()


def get_work_queues() -> List[WorkQueue]:
    entry = gdb.parse_and_eval("work_thread")
    kwork_wqueue_s = utils.lookup_type("struct kwork_wqueue_s")

    tcbs = utils.get_tcbs()
    #  The function address may be or'ed with 0x01
    tcbs = filter(lambda tcb: int(tcb.entry.main) & ~0x01 == entry, tcbs)
    queue = []
    for tcb in tcbs:
        if not (args := utils.get_task_argvstr(tcb)):
            continue
        #   wqueue  = (FAR struct kwork_wqueue_s *)
        #             ((uintptr_t)strtoul(argv[1], NULL, 16));
        #   kworker = (FAR struct kworker_s *)
        #             ((uintptr_t)strtoul(argv[2], NULL, 16));
        wqueue = gdb.Value(int(args[1], 16)).cast(kwork_wqueue_s.pointer())
        wqueue = WorkQueue(wqueue, name=utils.get_task_name(tcb))
        if wqueue not in queue:
            queue.append(wqueue)

    return queue


class WorkQueueDump(gdb.Command):
    """Show work queue information"""

    def __init__(self):
        if not utils.get_symbol_value("CONFIG_SCHED_WORKQUEUE"):
            return

        super().__init__("worker", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        queues = get_work_queues()
        for queue in queues:
            print(f"{queue}")
            if not queue.is_running:
                continue

            print("    Running:")  # Dump the work that is running
            running = [thread for thread in queue.threads if thread.is_running]
            print("\n".join(f"    {thread}" for thread in running))

            if not queue.workers:
                continue

            print("    Queued:")
            print("\n".join(f"    {work}" for work in queue.workers))
