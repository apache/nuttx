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

import gdb
import utils

UINT16_MAX = 0xFFFF
SEM_TYPE_MUTEX = 4

saved_regs = None


def save_regs():
    global saved_regs
    tcbinfo = gdb.parse_and_eval("g_tcbinfo")

    if saved_regs is not None:
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

    if saved_regs is None:
        return

    arch = gdb.selected_frame().architecture()
    i = 0
    for reg in arch.registers():
        if i >= tcbinfo["regs_num"]:
            break

        gdb.execute("set $%s=%d" % (reg.name, int(saved_regs[i])))
        i += 1

    saved_regs = None


class Nxsetregs(gdb.Command):
    """
    Set registers to the specified values.
    Usage: nxsetregs [regs]

    Etc: nxsetregs
         nxsetregs g_current_regs[0]
         nxsetregs tcb->xcp.regs
         Nxsetregs g_pidhash[0].tcb->xcp.regs

    Default regs is g_current_regs[0],if regs is NULL, it will not set registers.

    """

    def __init__(self):
        super(Nxsetregs, self).__init__("nxsetregs", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        gdb.execute("set $_current_regs=tcbinfo_running_regs()")
        current_regs = gdb.parse_and_eval("$_current_regs")
        tcbinfo = gdb.parse_and_eval("g_tcbinfo")
        arg = args.split(" ")

        if arg[0] != "":
            regs = gdb.parse_and_eval("%s" % arg[0]).cast(
                gdb.lookup_type("char").pointer()
            )
        else:
            regs = current_regs.cast(gdb.lookup_type("char").pointer())

        if regs == 0:
            gdb.write("regs is NULL\n")
            return

        save_regs()
        arch = gdb.selected_frame().architecture()
        i = 0
        for reg in arch.registers():
            if i >= tcbinfo["regs_num"]:
                return

            gdb.execute("select-frame 0")
            if tcbinfo["reg_off"]["p"][i] != UINT16_MAX:
                value = gdb.Value(regs + tcbinfo["reg_off"]["p"][i]).cast(
                    gdb.lookup_type("uintptr_t").pointer()
                )[0]
                gdb.execute("set $%s = 0x%x" % (reg.name, value))

            i += 1


def get_pc_value(tcb):
    arch = gdb.selected_frame().architecture()
    tcbinfo = gdb.parse_and_eval("g_tcbinfo")

    i = 0
    for reg in arch.registers():
        if reg.name == "pc" or reg.name == "rip" or reg.name == "eip":
            break
        i += 1

    regs = tcb["xcp"]["regs"].cast(gdb.lookup_type("char").pointer())
    value = gdb.Value(regs + tcbinfo["reg_off"]["p"][i]).cast(
        gdb.lookup_type("uintptr_t").pointer()
    )[0]

    return int(value)


class Nxinfothreads(gdb.Command):
    def __init__(self):
        super(Nxinfothreads, self).__init__("info threads", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        npidhash = gdb.parse_and_eval("g_npidhash")
        pidhash = gdb.parse_and_eval("g_pidhash")
        statenames = gdb.parse_and_eval("g_statenames")

        if utils.is_target_smp():
            gdb.write(
                "%-4s %-4s %-4s %-21s %-80s %-30s\n"
                % ("Tid", "Pid", "Cpu", "Thread", "Info", "Frame")
            )
        else:
            gdb.write("%-5s %-4s %-4s %-21s %-80s %-30s\n" % ("Index", "Tid", "Pid", "Thread", "Info", "Frame"))

        for i in range(0, npidhash):
            if pidhash[i] == 0:
                continue

            pid = pidhash[i]["group"]["tg_pid"]
            tid = pidhash[i]["pid"]

            if pidhash[i]["task_state"] == gdb.parse_and_eval("TSTATE_TASK_RUNNING"):
                index = "*%s" % i
                pc = int(gdb.parse_and_eval("$pc"))
            else:
                index = " %s" % i
                pc = get_pc_value(pidhash[i])

            thread = "Thread 0x%x" % pidhash[i]

            statename = statenames[pidhash[i]["task_state"]].string()

            if pidhash[i]["task_state"] == gdb.parse_and_eval("TSTATE_WAIT_SEM"):
                mutex = pidhash[i]["waitobj"].cast(gdb.lookup_type("sem_t").pointer())
                if mutex["flags"] & SEM_TYPE_MUTEX:
                    mutex = pidhash[i]["waitobj"].cast(gdb.lookup_type("mutex_t").pointer())
                    statename = "Waiting,Mutex:%d" % (mutex["holder"])

            try:
                """Maybe tcb not have name member, or name is not utf-8"""
                info = "(Name: %s, State: %s, Priority: %d, Stack: %d)" % (
                    pidhash[i]["name"].string(),
                    statename,
                    pidhash[i]["sched_priority"],
                    pidhash[i]["adj_stack_size"],
                )
            except gdb.error and UnicodeDecodeError:
                info = "(Name: Not utf-8, State: %s, Priority: %d, Stack: %d)" % (
                    statename,
                    pidhash[i]["sched_priority"],
                    pidhash[i]["adj_stack_size"],
                )

            line = gdb.find_pc_line(pc)
            if line.symtab:
                func = gdb.execute("info symbol %d " % pc, to_string=True)
                frame = "0x%x %s at %s:%d" % (
                    pc,
                    func.split()[0] + "()",
                    line.symtab,
                    line.line,
                )
            else:
                frame = "No symbol with pc"

            if utils.is_target_smp():
                cpu = "%d" % pidhash[i]["cpu"]
                gdb.write(
                    "%-5s %-4s %-4s %-4s %-21s %-80s %-30s\n" % (index, tid, pid, cpu, thread, info, frame)
                )
            else:
                gdb.write("%-5s %-4s %-4s %-21s %-80s %-30s\n" % (index, tid, pid, thread, info, frame))


class Nxthread(gdb.Command):
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
                        gdb.write("Thread %d %s\n" % (i, pidhash[i]["name"].string()))
                    except gdb.error and UnicodeDecodeError:
                        gdb.write("Thread %d\n" % (i))

                    gdb.execute("nxsetregs g_pidhash[%d]->xcp.regs" % i)
                    cmd_arg = ""
                    for cmd in arg[2:]:
                        cmd_arg += cmd + " "

                    gdb.execute("%s\n" % cmd_arg)
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
                            gdb.write(
                                "Thread %d %s\n" % (i, pidhash[i]["name"].string())
                            )
                        except gdb.error and UnicodeDecodeError:
                            gdb.write("Thread %d\n" % (i))

                        gdb.execute("nxsetregs g_pidhash[%d]->xcp.regs" % i)
                        gdb.execute("%s\n" % cmd)
                        restore_regs()

        else:
            if arg[0].isnumeric() and pidhash[int(arg[0])] != 0:
                if pidhash[int(arg[0])]["task_state"] == gdb.parse_and_eval("TSTATE_TASK_RUNNING"):
                    restore_regs()
                else:
                    gdb.execute("nxsetregs g_pidhash[%s]->xcp.regs" % arg[0])
            else:
                gdb.write("Invalid thread id %s\n" % arg[0])


class Nxcontinue(gdb.Command):
    def __init__(self):
        super(Nxcontinue, self).__init__("nxcontinue", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        restore_regs()
        gdb.execute("continue")

class Nxstep(gdb.Command):
    def __init__(self):
        super(Nxstep, self).__init__("nxstep", gdb.COMMAND_USER)

    def invoke(self, args, from_tty):
        restore_regs()
        gdb.execute("step")

# We can't use a user command to rename continue it will recursion

gdb.execute("define c\n nxcontinue \n end\n")
gdb.execute("define s\n nxstep \n end\n")
gdb.write("\nif use thread command, please don't use 'continue', use 'c' instead !!!\n")
gdb.write("if use thread command, please don't use 'step', use 's' instead !!!\n")

Nxsetregs()
Nxinfothreads()
Nxthread()
Nxcontinue()
Nxstep()
