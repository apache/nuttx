############################################################################
# tools/gdb/nuttxgdb/protocols/thread.py
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

from .fs import FileList
from .value import Value


class Group(Value):
    """struct group_s"""

    tg_pid: Value
    tg_ppid: Value
    tg_flags: Value
    tg_uid: Value
    tg_gid: Value
    tg_euid: Value
    tg_egid: Value
    tg_members: Value
    tg_bininfo: Value
    tg_children: Value
    tg_nchildren: Value
    tg_exitcode: Value
    tg_nwaiters: Value
    tg_waitflags: Value
    tg_exitsem: Value
    tg_statloc: Value
    tg_joinlock: Value
    tg_joinqueue: Value
    tg_info: Value
    tg_sigactionq: Value
    tg_sigpendingq: Value
    tg_sigdefault: Value
    tg_envp: Value
    tg_envc: Value
    itimer: Value
    tg_filelist: FileList
    tg_mm_map: Value


class Tcb(Value):
    """struct tcb_s"""

    flink: Value
    blink: Value
    group: Group
    member: Value
    join_queue: Value
    join_entry: Value
    join_sem: Value
    join_val: Value
    addrenv_own: Value
    addrenv_curr: Value
    pid: Value
    sched_priority: Value
    init_priority: Value
    start: Value
    entry: Value
    task_state: Value
    boost_priority: Value
    base_priority: Value
    holdsem: Value
    cpu: Value
    affinity: Value
    flags: Value
    lockcount: Value
    irqcount: Value
    errcode: Value
    timeslice: Value
    sporadic: Value
    waitdog: Value
    adj_stack_size: Value
    stack_alloc_ptr: Value
    stack_base_ptr: Value
    dspace: Value
    waitobj: Value
    sigprocmask: Value
    sigwaitmask: Value
    sigpendactionq: Value
    sigpostedq: Value
    sigunbinfo: Value
    mhead: Value
    ticks: Value
    run_start: Value
    run_max: Value
    run_time: Value
    premp_start: Value
    premp_max: Value
    premp_caller: Value
    premp_max_caller: Value
    crit_start: Value
    crit_max: Value
    crit_caller: Value
    crit_max_caller: Value
    perf_event_ctx: Value
    perf_event_mutex: Value
    xcp: Value
    sigdeliver: Value
    name: Value
    stackrecord_pc: Value
    stackrecord_sp: Value
    stackrecord_pc_deepest: Value
    stackrecord_sp_deepest: Value
    sp_deepest: Value
    caller_deepest: Value
    level_deepest: Value
    level: Value
