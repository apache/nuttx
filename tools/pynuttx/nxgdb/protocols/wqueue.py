############################################################################
# tools/pynuttx/nxgdb/protocols/wqueue.py
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

from .value import Value


class Work(Value):
    """struct work_s"""

    class U(Value):
        class S(Value):
            dq: Value
            qtime: Value

        s: S
        timer: Value  # wdog_s

    u: U
    worker: Value  # void (*worker_t)(FAR void *arg);
    arg: Value
    wq: KWorkQueue


class KWorker(Value):
    """struct kworker_s"""

    pid: Value
    work: Value
    wait: Value


class KWorkQueue(Value):
    """struct kwork_wqueue_s"""

    q: Value
    sem: Value
    exsem: Value
    nthreads: int
    exit: bool
    worker: List[Value]


class HPWorkQueue(KWorkQueue):
    """struct hp_wqueue_s"""


class LPWorkQueue(KWorkQueue):
    """struct lp_wqueue_s"""
