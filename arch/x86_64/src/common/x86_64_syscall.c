/****************************************************************************
 * arch/x86_64/src/common/x86_64_syscall.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <syscall.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/addrenv.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Syscall function */

typedef uintptr_t (*syscall_stub_t)(int nbr,
                                    uintptr_t parm1, uintptr_t parm2,
                                    uintptr_t parm3, uintptr_t parm4,
                                    uintptr_t parm5, uintptr_t parm6);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dump_syscall
 *
 * Description:
 *   Dump the syscall registers
 *
 ****************************************************************************/

static void dump_syscall(const char *tag, uint64_t *regs)
{
  unsigned int cmd = regs[REG_RAX];

#ifdef CONFIG_LIB_SYSCALL
  if (cmd >= CONFIG_SYS_RESERVED)
    {
      svcinfo("SYSCALL %s: cmd: %d name: %s\n", tag,
              cmd, g_funcnames[cmd - CONFIG_SYS_RESERVED]);
    }
  else
#endif
    {
      svcinfo("SYSCALL %s: cmd: %d\n", tag, cmd);
    }

  svcinfo("  RSP: %" PRIx64 " RCX: %" PRIx64 "\n",
          regs[REG_RSP], regs[REG_RCX]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_syscall
 *
 * Description:
 *   Syscall handler called from x86_64_syscall_entry().
 *   Current registers stored in regs argument.
 *   The syscall is called with:
 *
 *     - RAX = system call command, and
 *     - RDI, RSI, RDX, R10, R8, R9 = variable number of arguments depending
 *       on the system call.
 *
 ****************************************************************************/

uint64_t *x86_64_syscall(uint64_t *regs)
{
  unsigned int cmd  = regs[REG_RAX];
  uint64_t     arg1 = regs[REG_RDI];
  uint64_t     arg2 = regs[REG_RSI];
  uint64_t     arg3 = regs[REG_RDX];
  uint64_t     arg4 = regs[REG_R10];
  uint64_t     arg5 = regs[REG_R8];
  uint64_t     arg6 = regs[REG_R9];
  uintptr_t    ret  = 0;

  /* The syscall command is in RAX on entry */

  dump_syscall("Entry", regs);

  /* Handle the syscall according to the command in RAX */

  switch (cmd)
    {
      /* This is not an architecture-specific system call.  If NuttX is
       * built as a standalone kernel with a system call interface, then
       * all of the additional system calls must be handled as in the
       * default case.
       */

      default:
        {
          int             nbr  = cmd - CONFIG_SYS_RESERVED;
          struct tcb_s   *rtcb = nxsched_self();
          syscall_stub_t  stub = (syscall_stub_t)g_stublookup[nbr];

          DEBUGASSERT(nbr < SYS_nsyscalls);
          DEBUGASSERT(rtcb->xcp.nsyscalls < CONFIG_SYS_NNEST);

          /* Setup nested syscall */

          rtcb->xcp.syscall[rtcb->xcp.nsyscalls].sysreturn = regs[REG_RCX];
          rtcb->xcp.nsyscalls += 1;

          /* Call syscall function */

          ret = stub(nbr, arg1, arg2, arg3, arg4, arg5, arg6);

          /* Setup return from nested syscall */

          rtcb->xcp.nsyscalls -= 1;
          regs[REG_RCX] = rtcb->xcp.syscall[rtcb->xcp.nsyscalls].sysreturn;

          break;
        }
    }

  dump_syscall("Exit", regs);

  /* Store return value in RAX register */

  regs[REG_RAX] = ret;

  /* Return pointer to regs */

  return regs;
}
