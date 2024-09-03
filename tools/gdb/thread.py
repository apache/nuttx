############################################################################
# tools/gdb/thread.py
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

import argparse
from enum import Enum, auto

import gdb
import utils
from stack import Stack

UINT16_MAX = 0xFFFF
SEM_TYPE_MUTEX = 4

saved_regs = None


def save_regs():
    global saved_regs
    tcbinfo = gdb.parse_and_eval("g_tcbinfo")

    if saved_regs:
        return
    arch = gdb.selected_frame().architecture()
    saved_regs = []
    i = 0
    for reg in arch.registers():
        if i >= tcbinfo["regs_num"]:
            break

        saved_regs.append(gdb.parse_and_eval("$%s" % reg.name))
        i += 1


def restore_regs():
    tcbinfo = gdb.parse_and_eval("g_tcbinfo")
    global saved_regs

    if not saved_regs:
        return

    arch = gdb.selected_frame().architecture()
    i = 0
    for reg in arch.registers():
        if i >= tcbinfo["regs_num"]:
            break

        gdb.execute(f"set ${reg.name}={int(saved_regs[i])}")
        i += 1

    saved_regs = None


class SetRegs(gdb.Command):
    """Set registers to the specified values.
    Usage: setregs [regs]

    Etc: setregs
         setregs tcb->xcp.regs
         setregs g_pidhash[0]->xcp.regs

    Default regs is tcbinfo_current_regs(),if regs is NULL, it will not set registers.
    """

    def __init__(self):
        super(SetRegs, self).__init__("setregs", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        parser = argparse.ArgumentParser(
            description="Set registers to the specified values"
        )

        parser.add_argument(
            "regs",
            nargs="?",
            default="",
            help="The registers to set, use tcbinfo_current_regs() if not specified",
        )

        try:
            args = parser.parse_args(gdb.string_to_argv(arg))
        except SystemExit:
            return

        if args and args.regs:
            regs = gdb.parse_and_eval(f"{args.regs}").cast(
                utils.lookup_type("char").pointer()
            )
        else:
            current_regs = gdb.parse_and_eval("tcbinfo_current_regs()")
            regs = current_regs.cast(utils.lookup_type("char").pointer())

        if regs == 0:
            gdb.write("regs is NULL\n")
            return

        tcbinfo = gdb.parse_and_eval("g_tcbinfo")
        save_regs()
        arch = gdb.selected_frame().architecture()

        regoffset = [
            int(tcbinfo["reg_off"]["p"][i])
            for i in range(tcbinfo["regs_num"])
            if tcbinfo["reg_off"]["p"][i] != UINT16_MAX
        ]

        i = 0
        for reg in arch.registers():
            if i >= len(regoffset):
                return

            gdb.execute("select-frame 0")
            value = gdb.Value(regs + regoffset[i]).cast(
                utils.lookup_type("uintptr_t").pointer()
            )[0]
            gdb.execute(f"set ${reg.name} = {value}")
            i += 1


class Nxinfothreads(gdb.Command):
    """Display information of all threads"""

    def __init__(self):
        super(Nxinfothreads, self).__init__("info threads", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        npidhash = gdb.parse_and_eval("g_npidhash")
        pidhash = gdb.parse_and_eval("g_pidhash")
        statenames = gdb.parse_and_eval("g_statenames")

        if utils.is_target_smp():
            gdb.write(
                "%-5s %-4s %-4s %-4s %-21s %-80s %-30s\n"
                % ("Index", "Tid", "Pid", "Cpu", "Thread", "Info", "Frame")
            )
        else:
            gdb.write(
                "%-5s %-4s %-4s %-21s %-80s %-30s\n"
                % ("Index", "Tid", "Pid", "Thread", "Info", "Frame")
            )

        for i in range(0, npidhash):
            tcb = pidhash[i]
            if not tcb:
                continue

            pid = tcb["group"]["tg_pid"]
            tid = tcb["pid"]

            if tcb["task_state"] == gdb.parse_and_eval("TSTATE_TASK_RUNNING"):
                index = f"*{i}"
                pc = utils.get_pc()
            else:
                index = f" {i}"
                pc = utils.get_pc(tcb=tcb)

            thread = f"Thread {hex(tcb)}"

            statename = statenames[tcb["task_state"]].string()
            statename = f'\x1b{"[32;1m" if statename == "Running" else "[33;1m"}{statename}\x1b[m'

            if tcb["task_state"] == gdb.parse_and_eval("TSTATE_WAIT_SEM"):
                mutex = tcb["waitobj"].cast(utils.lookup_type("sem_t").pointer())
                if mutex["flags"] & SEM_TYPE_MUTEX:
                    mutex = tcb["waitobj"].cast(utils.lookup_type("mutex_t").pointer())
                    statename = f"Waiting,Mutex:{mutex['holder']}"

            try:
                """Maybe tcb not have name member, or name is not utf-8"""
                info = (
                    "(Name: \x1b[31;1m%s\x1b[m, State: %s, Priority: %d, Stack: %d)"
                    % (
                        tcb["name"].string(),
                        statename,
                        tcb["sched_priority"],
                        tcb["adj_stack_size"],
                    )
                )
            except gdb.error and UnicodeDecodeError:
                info = "(Name: Not utf-8, State: %s, Priority: %d, Stack: %d)" % (
                    statename,
                    tcb["sched_priority"],
                    tcb["adj_stack_size"],
                )

            line = gdb.find_pc_line(pc)
            if line.symtab:
                func = gdb.execute(f"info symbol {pc} ", to_string=True)
                frame = "\x1b[34;1m0x%x\x1b[\t\x1b[33;1m%s\x1b[m at %s:%d" % (
                    pc,
                    func.split()[0] + "()",
                    line.symtab,
                    line.line,
                )
            else:
                frame = "No symbol with pc"

            if utils.is_target_smp():
                cpu = f"{tcb['cpu']}"
                gdb.write(
                    "%-5s %-4s %-4s %-4s %-21s %-80s %-30s\n"
                    % (index, tid, pid, cpu, thread, info, frame)
                )
            else:
                gdb.write(
                    "%-5s %-4s %-4s %-21s %-80s %-30s\n"
                    % (index, tid, pid, thread, info, frame)
                )


class Nxthread(gdb.Command):
    """Switch to a specified thread"""

    def __init__(self):
        super(Nxthread, self).__init__("thread", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        npidhash = gdb.parse_and_eval("g_npidhash")
        pidhash = gdb.parse_and_eval("g_pidhash")
        arg = args.split(" ")
        arglen = len(arg)

        if arg[0] == "":
            pass
        elif arg[0] == "apply":
            if arglen <= 1:
                gdb.write("Please specify a thread ID list\n")
            elif arglen <= 2:
                gdb.write("Please specify a command following the thread ID list\n")

            elif arg[1] == "all":
                for i in range(0, npidhash):
                    if pidhash[i] == 0:
                        continue
                    try:
                        gdb.write(f"Thread {i} {pidhash[i]['name'].string()}\n")
                    except gdb.error and UnicodeDecodeError:
                        gdb.write(f"Thread {i}\n")

                    gdb.execute(f"setregs g_pidhash[{i}]->xcp.regs")
                    cmd_arg = ""
                    for cmd in arg[2:]:
                        cmd_arg += cmd + " "

                    gdb.execute(f"{cmd_arg}\n")
                    restore_regs()
            else:
                threadlist = []
                i = 0
                cmd = ""
                for i in range(1, arglen):
                    if arg[i].isnumeric():
                        threadlist.append(int(arg[i]))
                    else:
                        cmd += arg[i] + " "

                if len(threadlist) == 0 or cmd == "":
                    gdb.write("Please specify a thread ID list and command\n")
                else:
                    for i in threadlist:
                        if i >= npidhash:
                            break

                        if pidhash[i] == 0:
                            continue

                        try:
                            gdb.write(f"Thread {i} {pidhash[i]['name'].string()}\n")
                        except gdb.error and UnicodeDecodeError:
                            gdb.write(f"Thread {i}\n")

                        gdb.execute(f"setregs g_pidhash[{i}]->xcp.regs")
                        gdb.execute(f"{cmd}\n")
                        restore_regs()

        else:
            if arg[0].isnumeric() and pidhash[int(arg[0])] != 0:
                if pidhash[int(arg[0])]["task_state"] == gdb.parse_and_eval(
                    "TSTATE_TASK_RUNNING"
                ):
                    restore_regs()
                else:
                    gdb.execute("setregs g_pidhash[%s]->xcp.regs" % arg[0])
            else:
                gdb.write(f"Invalid thread id {arg[0]}\n")


class Nxcontinue(gdb.Command):
    """Restore the registers and continue the execution"""

    def __init__(self):
        super(Nxcontinue, self).__init__("nxcontinue", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        restore_regs()
        gdb.execute("continue")


class Nxstep(gdb.Command):
    """Restore the registers and step the execution"""

    def __init__(self):
        super(Nxstep, self).__init__("nxstep", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        restore_regs()
        gdb.execute("step")


class TaskType(Enum):
    TASK = 0
    PTHREAD = 1
    KTHREAD = 2


class TaskSchedPolicy(Enum):
    FIFO = 0
    RR = 1
    SPORADIC = 2


class TaskState(Enum):
    Invalid = 0
    Waiting_Unlock = auto()
    Ready = auto()
    if utils.get_symbol_value("CONFIG_SMP"):
        Assigned = auto()
    Running = auto()
    Inactive = auto()
    Waiting_Semaphore = auto()
    Waiting_Signal = auto()
    if not utils.get_symbol_value(
        "CONFIG_DISABLE_MQUEUE"
    ) or not utils.get_symbol_value("CONFIG_DISABLE_MQUEUE_SYSV"):
        Waiting_MQEmpty = auto()
        Waiting_MQFull = auto()
    if utils.get_symbol_value("CONFIG_PAGING"):
        Waiting_PagingFill = auto()
    if utils.get_symbol_value("CONFIG_SIG_SIGSTOP_ACTION"):
        Stopped = auto()


class Ps(gdb.Command):
    def __init__(self):
        super(Ps, self).__init__("ps", gdb.COMMAND_USER)
        self._fmt_wxl = "{0: <{width}}"
        # By default we align to the right, whcih respects the nuttx foramt
        self._fmt_wx = "{0: >{width}}"

    def parse_and_show_info(self, tcb):
        def get_macro(x):
            return utils.get_symbol_value(x)

        def eval2str(cls, x):
            return cls(int(x)).name

        def cast2ptr(x, t):
            return x.cast(utils.lookup_type(t).pointer())

        pid = int(tcb["pid"])
        group = int(tcb["group"]["tg_pid"])
        priority = int(tcb["sched_priority"])

        policy = eval2str(
            TaskSchedPolicy,
            (tcb["flags"] & get_macro("TCB_FLAG_POLICY_MASK"))
            >> get_macro("TCB_FLAG_POLICY_SHIFT"),
        )

        task_type = eval2str(
            TaskType,
            (tcb["flags"] & get_macro("TCB_FLAG_TTYPE_MASK"))
            >> get_macro("TCB_FLAG_TTYPE_SHIFT"),
        )

        npx = "P" if (tcb["flags"] & get_macro("TCB_FLAG_EXIT_PROCESSING")) else "-"

        waiter = (
            str(int(cast2ptr(tcb["waitobj"], "mutex_t")["holder"]))
            if tcb["waitobj"]
            and cast2ptr(tcb["waitobj"], "sem_t")["flags"] & get_macro("SEM_TYPE_MUTEX")
            else ""
        )
        state_and_event = eval2str(TaskState, (tcb["task_state"])) + (
            "@Mutex_Holder: " + waiter if waiter else ""
        )
        state_and_event = state_and_event.split("_")

        # Append a null str here so we don't need to worry
        # about the number of elements as we only want the first two
        state, event = (
            state_and_event if len(state_and_event) > 1 else state_and_event + [""]
        )

        sigmask = "{0:#0{1}x}".format(
            sum(
                int(tcb["sigprocmask"]["_elem"][i] << i)
                for i in range(get_macro("_SIGSET_NELEM"))
            ),
            get_macro("_SIGSET_NELEM") * 8 + 2,
        )[
            2:
        ]  # exclude "0x"

        st = Stack(
            tcb["name"].string(),
            hex(tcb["entry"]["pthread"]),  # should use main?
            int(tcb["stack_base_ptr"]),
            int(tcb["stack_alloc_ptr"]),
            int(tcb["adj_stack_size"]),
            utils.get_sp(tcb),
            4,
        )

        stacksz = st._stack_size
        used = st.max_usage()
        filled = "{0:.2%}".format(st.max_usage() / st._stack_size)

        cpu = int(tcb["cpu"]) if get_macro("CONFIG_SMP") else 0

        # For a task we need to display its cmdline arguments, while for a thread we display
        # pointers to its entry and argument
        cmd = ""
        name = tcb["name"].string()

        if int(tcb["flags"] & get_macro("TCB_FLAG_TTYPE_MASK")) == int(
            get_macro("TCB_FLAG_TTYPE_PTHREAD")
        ):
            entry = tcb["entry"]["main"]
            ptcb = cast2ptr(tcb, "struct pthread_tcb_s")
            arg = ptcb["arg"]
            cmd = " ".join((name, hex(entry), hex(arg)))
        else:
            argv = tcb["group"]["tg_info"]["ta_argv"] + 1

            args = []
            parg = argv
            while parg.dereference():
                args.append(parg.dereference().string())
                parg += 1

            cmd = " ".join([name] + args)

        if not utils.get_symbol_value("CONFIG_SCHED_CPULOAD_NONE"):
            load = "{0:.1%}".format(
                int(tcb["ticks"]) / int(gdb.parse_and_eval("g_cpuload_total"))
            )
        else:
            load = "Dis."

        gdb.write(
            " ".join(
                (
                    self._fmt_wx.format(pid, width=5),
                    self._fmt_wx.format(group, width=5),
                    self._fmt_wx.format(cpu, width=3),
                    self._fmt_wx.format(priority, width=3),
                    self._fmt_wxl.format(policy, width=8),
                    self._fmt_wxl.format(task_type, width=7),
                    self._fmt_wx.format(npx, width=3),
                    self._fmt_wxl.format(state, width=8),
                    self._fmt_wxl.format(event, width=9),
                    self._fmt_wxl.format(sigmask, width=8),
                    self._fmt_wx.format(stacksz, width=7),
                    self._fmt_wx.format(used, width=7),
                    self._fmt_wx.format(filled, width=6),
                    self._fmt_wx.format(load, width=6),
                    cmd,
                )
            )
        )
        gdb.write("\n")

    def invoke(self, args, from_tty):
        gdb.write(
            " ".join(
                (
                    self._fmt_wx.format("PID", width=5),
                    self._fmt_wx.format("GROUP", width=5),
                    self._fmt_wx.format("CPU", width=3),
                    self._fmt_wx.format("PRI", width=3),
                    self._fmt_wxl.format("POLICY", width=8),
                    self._fmt_wxl.format("TYPE", width=7),
                    self._fmt_wx.format("NPX", width=3),
                    self._fmt_wxl.format("STATE", width=8),
                    self._fmt_wxl.format("EVENT", width=9),
                    self._fmt_wxl.format(
                        "SIGMASK", width=utils.get_symbol_value("_SIGSET_NELEM") * 8
                    ),
                    self._fmt_wx.format("STACK", width=7),
                    self._fmt_wx.format("USED", width=7),
                    self._fmt_wx.format("FILLED", width=3),
                    self._fmt_wx.format("LOAD", width=6),
                    "COMMAND",
                )
            )
        )
        gdb.write("\n")

        for tcb in utils.get_tcbs():
            self.parse_and_show_info(tcb)


def register_commands():
    SetRegs()
    Ps()

    # Disable thread commands for core dump and gdb-stub.
    # In which case the recognized threads count is less or equal to the number of cpus
    ncpus = utils.get_symbol_value("CONFIG_SMP_NCPUS") or 1
    nthreads = len(gdb.selected_inferior().threads())
    if nthreads <= ncpus:
        SetRegs()
        Nxinfothreads()
        Nxthread()
        Nxcontinue()
        Nxstep()

        # We can't use a user command to rename continue it will recursion
        gdb.execute("define c\n nxcontinue \n end\n")
        gdb.execute("define s\n nxstep \n end\n")
        gdb.write(
            "\n\x1b[31;1m if use thread command, please don't use 'continue', use 'c' instead !!!\x1b[m\n"
        )
        gdb.write(
            "\x1b[31;1m if use thread command, please don't use 'step', use 's' instead !!!\x1b[m\n"
        )


register_commands()
