############################################################################
# tools/gdb/nuttxgdb/thread.py
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
import re
from enum import Enum, auto

import gdb

from . import utils
from .stack import Stack

UINT16_MAX = 0xFFFF
SEM_TYPE_MUTEX = 4
TSTATE_TASK_RUNNING = utils.get_symbol_value("TSTATE_TASK_RUNNING")
CONFIG_SMP_NCPUS = utils.get_symbol_value("CONFIG_SMP_NCPUS") or 1


def is_thread_command_supported():
    # Check if the native thread command is available by compare the number of threads.
    # It should have at least CONFIG_SMP_NCPUS of idle threads.
    return len(gdb.selected_inferior().threads()) > CONFIG_SMP_NCPUS


class Registers:
    saved_regs = None
    reginfo = None

    def __init__(self):
        if not Registers.reginfo:
            reginfo = {}

            # Switch to second inferior to get the original remote-register layout
            state = utils.suppress_cli_notifications(True)
            utils.switch_inferior(2)

            natural_size = gdb.lookup_type("long").sizeof
            tcb_info = gdb.parse_and_eval("g_tcbinfo")
            reg_off = tcb_info["reg_off"]["p"]  # Register offsets in tcbinfo
            packet_size = tcb_info["regs_num"] * natural_size

            lines = gdb.execute("maint print remote-registers", to_string=True)
            for line in lines.splitlines()[1:]:
                if not line:
                    continue

                # Name         Nr  Rel Offset    Size  Type            Rmt Nr  g/G Offset
                match = re.match(
                    r"\s(\S+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\S+)(?:\s+(\d+)\s+(\d+))?",
                    line,
                )
                if not match:
                    continue

                name, _, _, _, size, _, rmt_nr, offset = match.groups()

                # We only need those registers that have a remote register
                if rmt_nr is None:
                    continue

                rmt_nr = int(rmt_nr)
                offset = int(offset)
                size = int(size)

                # We only have limited number of registers in packet
                if offset + size > packet_size:
                    continue

                index = offset // natural_size
                tcb_reg_off = int(reg_off[index])
                if tcb_reg_off == UINT16_MAX:
                    # This register is not saved in tcb context
                    continue

                reginfo[name] = {
                    "rmt_nr": rmt_nr,  # The register number in remote-registers, Aka the one we saved in g_tcbinfo.
                    "tcb_reg_off": tcb_reg_off,
                }

            Registers.reginfo = reginfo
            utils.switch_inferior(1)  # Switch back
            utils.suppress_cli_notifications(state)

    def load(self, regs):
        """Load registers from context register address"""
        regs = int(regs)
        for name, info in Registers.reginfo.items():
            addr = regs + info["tcb_reg_off"]
            # value = *(uintptr_t *)addr
            value = (
                gdb.Value(addr)
                .cast(utils.lookup_type("uintptr_t").pointer())
                .dereference()
            )
            gdb.execute(f"set ${name}={int(value)}")

    def switch(self, pid):
        """Switch to the specified thread"""
        tcb = utils.get_tcb(pid)
        if not tcb:
            gdb.write(f"Thread {pid} not found\n")
            return

        if tcb["task_state"] == TSTATE_TASK_RUNNING:
            # If the thread is running, then register is not in context but saved temporarily
            self.restore()
            return

        # Save current if this is the running thread, which is the case we never saved it before
        if not self.saved_regs:
            self.save()

        self.load(tcb["xcp"]["regs"])

    def save(self):
        """Save current registers"""
        if Registers.saved_regs:
            # Already saved
            return

        registers = {}
        frame = gdb.newest_frame()
        for name, _ in Registers.reginfo.items():
            value = frame.read_register(name)
            registers[name] = value

        Registers.saved_regs = registers

    def restore(self):
        if not Registers.saved_regs:
            return

        for name, value in Registers.saved_regs.items():
            gdb.execute(f"set ${name}={int(value)}")

        Registers.saved_regs = None


g_registers = Registers()


class SetRegs(gdb.Command):
    """Set registers to the specified values.
    Usage: setregs [regs]

    Etc: setregs
         setregs tcb->xcp.regs
         setregs g_pidhash[0]->xcp.regs

    Default regs is tcbinfo_current_regs(),if regs is NULL, it will not set registers.
    """

    def __init__(self):
        super().__init__("setregs", gdb.COMMAND_USER)

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

        g_registers.save()
        g_registers.load(regs)


class Nxinfothreads(gdb.Command):
    """Display information of all threads"""

    def __init__(self):
        super().__init__("info nxthreads", gdb.COMMAND_USER)
        if not is_thread_command_supported():
            gdb.execute("define info threads\n info nxthreads \n end\n")

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

        for i, tcb in enumerate(utils.ArrayIterator(pidhash, npidhash)):
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
                        utils.get_task_name(tcb),
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
        if not is_thread_command_supported():
            super().__init__("thread", gdb.COMMAND_USER)
        else:
            super().__init__("nxthread", gdb.COMMAND_USER)

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
                for i, tcb in enumerate(utils.ArrayIterator(pidhash, npidhash)):
                    if tcb == 0:
                        continue
                    try:
                        gdb.write(f"Thread {i} {tcb['name'].string()}\n")
                    except gdb.error and UnicodeDecodeError:
                        gdb.write(f"Thread {i}\n")

                    gdb.execute(f"setregs g_pidhash[{i}]->xcp.regs")
                    cmd_arg = ""
                    for cmd in arg[2:]:
                        cmd_arg += cmd + " "

                    gdb.execute(f"{cmd_arg}\n")
                    g_registers.restore()
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
                        g_registers.restore()

        else:
            if (
                arg[0].isnumeric()
                and int(arg[0]) < npidhash
                and pidhash[int(arg[0])] != 0
            ):
                if pidhash[int(arg[0])]["task_state"] == gdb.parse_and_eval(
                    "TSTATE_TASK_RUNNING"
                ):
                    g_registers.restore()
                else:
                    gdb.execute("setregs g_pidhash[%s]->xcp.regs" % arg[0])
            else:
                gdb.write(f"Invalid thread id {arg[0]}\n")


class Nxcontinue(gdb.Command):
    """Restore the registers and continue the execution"""

    def __init__(self):
        super().__init__("nxcontinue", gdb.COMMAND_USER)
        if not is_thread_command_supported():
            gdb.execute("define c\n nxcontinue \n end\n")
            gdb.write(
                "\n\x1b[31;1m if use thread command, please don't use 'continue', use 'c' instead !!!\x1b[m\n"
            )

    def invoke(self, args, from_tty):
        g_registers.restore()
        gdb.execute("continue")


class Nxstep(gdb.Command):
    """Restore the registers and step the execution"""

    def __init__(self):
        super().__init__("nxstep", gdb.COMMAND_USER)
        if not is_thread_command_supported():
            gdb.execute("define s\n nxstep \n end\n")
            gdb.write(
                "\x1b[31;1m if use thread command, please don't use 'step', use 's' instead !!!\x1b[m\n"
            )

    def invoke(self, args, from_tty):
        g_registers.restore()
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
        super().__init__("ps", gdb.COMMAND_USER)
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
            utils.get_task_name(tcb),
            hex(tcb["entry"]["pthread"]),  # should use main?
            int(tcb["stack_base_ptr"]),
            int(tcb["stack_alloc_ptr"]),
            int(tcb["adj_stack_size"]),
            utils.get_sp(tcb if tcb["task_state"] != TSTATE_TASK_RUNNING else None),
            4,
        )

        stacksz = st._stack_size
        used = st.max_usage()
        filled = "{0:.2%}".format(st.max_usage() / st._stack_size)

        cpu = int(tcb["cpu"]) if get_macro("CONFIG_SMP") else 0

        # For a task we need to display its cmdline arguments, while for a thread we display
        # pointers to its entry and argument
        cmd = ""
        name = utils.get_task_name(tcb)

        if int(tcb["flags"] & get_macro("TCB_FLAG_TTYPE_MASK")) == int(
            get_macro("TCB_FLAG_TTYPE_PTHREAD")
        ):
            entry = tcb["entry"]["main"]
            ptcb = cast2ptr(tcb, "struct pthread_tcb_s")
            arg = ptcb["arg"]
            cmd = " ".join((name, hex(entry), hex(arg)))
        elif tcb["pid"] < get_macro("CONFIG_SMP_NCPUS"):
            # This must be the Idle Tasks, hence we just get its name
            cmd = name
        else:
            # For tasks other than pthreads, hence need to get its command line
            # arguments from
            argv = (
                tcb["stack_alloc_ptr"]
                + cast2ptr(tcb["stack_alloc_ptr"], "struct tls_info_s")["tl_size"]
            )
            args = []
            parg = argv.cast(gdb.lookup_type("char").pointer().pointer()) + 1
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


class DeadLock(gdb.Command):
    """Detect and report if threads have deadlock."""

    def __init__(self):
        super().__init__("deadlock", gdb.COMMAND_USER)

    def has_deadlock(self, pid):
        """Check if the thread has a deadlock"""
        tcb = utils.get_tcb(pid)
        if not tcb or not tcb["waitobj"]:
            return False

        sem = tcb["waitobj"].cast(utils.lookup_type("sem_t").pointer())
        if not sem["flags"] & SEM_TYPE_MUTEX:
            return False

        # It's waiting on a mutex
        mutex = tcb["waitobj"].cast(utils.lookup_type("mutex_t").pointer())
        holder = mutex["holder"]
        if holder in self.holders:
            return True

        self.holders.append(holder)
        return self.has_deadlock(holder)

    def collect(self, tcbs):
        """Collect the deadlock information"""

        detected = []
        collected = []
        for tcb in tcbs:
            self.holders = []  # Holders for this tcb
            pid = tcb["pid"]
            if pid in detected or not self.has_deadlock(tcb["pid"]):
                continue

            # Deadlock detected
            detected.append(pid)
            detected.extend(self.holders)
            collected.append((pid, self.holders))

        return collected

    def diagnose(self, *args, **kwargs):
        collected = self.collect(utils.get_tcbs())

        return {
            "title": "Deadlock Report",
            "summary": f"{'No' if not collected else len(collected)} deadlocks",
            "command": "deadlock",
            "deadlocks": {int(pid): [i for i in h] for pid, h in collected},
        }

    def invoke(self, args, from_tty):
        collected = self.collect(utils.get_tcbs())
        if not collected:
            gdb.write("No deadlock detected.")
            return

        for pid, holders in collected:
            gdb.write(f'Thread {pid} "{utils.get_task_name(pid)}" has deadlocked!\n')
            gdb.write(f"  holders: {pid}->")
            gdb.write("->".join(str(pid) for pid in holders))
            gdb.write("\n")
