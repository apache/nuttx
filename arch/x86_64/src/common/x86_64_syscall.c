/****************************************************************************
 * arch/x86_64/src/common/x86_64_syscall.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#ifdef CONFIG_BUILD_KERNEL
      /* cmd=SYS_task_start:  This a user task start
       *
       *   void up_task_start(main_t taskentry, int argc, char *argv[])
       *     noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   cmd  = SYS_task_start
       *   arg1 = taskentry
       *   arg2 = argc
       *   arg3 = argv
       */

      case SYS_task_start:
        {
          /* Set up to return to the user-space _start function in
           * unprivileged mode.  We need:
           *
           *   RDI = argc
           *   RSI = argv
           *   RCX = taskentry (SYSRETQ return address)
           *
           */

          regs[REG_RDI] = arg2;
          regs[REG_RSI] = arg3;
          regs[REG_RCX] = arg1;

          break;
        }

      /* cmd=SYS_pthread_start:  This a user pthread start
       *
       *   void up_pthread_start(pthread_startroutine_t entrypt,
       *                         pthread_addr_t arg) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   cmd  = SYS_pthread_start
       *   arg1 = startup
       *   arg2 = entrypt
       *   arg3 = arg
       */

      case SYS_pthread_start:
        {
          /* Set up to enter the user-space pthread start-up function in
           * unprivileged mode. We need:
           *
           *   RDI   = entrypt
           *   RSI   = arg
           *   RCX   = startup (SYSRETQ return address)
           */

          regs[REG_RDI] = arg2;
          regs[REG_RSI] = arg3;
          regs[REG_RCX] = arg1;

          break;
        }

      /* cmd=SYS_signal_handler:  This a user signal handler callback
       *
       * void signal_handler(_sa_sigaction_t sighand, int signo,
       *                     siginfo_t *info, void *ucontext);
       *
       * At this point, the following values are saved in context:
       *
       *   cmd  = SYS_signal_handler
       *   arg1 = sighand
       *   arg2 = signo
       *   arg3 = info
       *   arg4 = ucontext (on the stack)
       */

      case SYS_signal_handler:
        {
          struct tcb_s *rtcb = nxsched_self();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn = regs[REG_RCX];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

          regs[REG_RCX] = (uint64_t)ARCH_DATA_RESERVE->ar_sigtramp;

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_RDI] = arg2; /* signal */
          regs[REG_RSI] = arg3; /* info */
          regs[REG_RDX] = arg4; /* ucontext */
          regs[REG_R10] = arg1; /* sighand */

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* If we are signalling a user process, then we must be operating
           * on the kernel stack now.  We need to switch back to the user
           * stack before dispatching the signal handler to the user code.
           * The existence of an allocated kernel stack is sufficient
           * information to make this decision.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              uint64_t usp;

              /* Copy "info" into user stack */

              usp = rtcb->xcp.saved_ursp - 8;

              /* Create a frame for info and copy the kernel info */

              usp = usp - sizeof(siginfo_t);
              memcpy((void *)usp, (void *)regs[REG_RSI], sizeof(siginfo_t));

              /* Now set the updated SP and user copy of "info" to RSI */

              regs[REG_RSP] = usp;
              regs[REG_RSI] = usp;
            }
#endif

          break;
        }

      /* cmd=SYS_signal_handler_return:  This a user signal handler callback
       *
       *   void signal_handler_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   cmd = SYS_signal_handler_return
       */

      case SYS_signal_handler_return:
        {
          /* Set up to return to the user-space. We need:
           *
           *   RCX = taskentry (SYSRETQ return address)
           *
           */

          struct tcb_s *rtcb = nxsched_self();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);

          regs[REG_RCX]       = rtcb->xcp.sigreturn;
          regs[REG_RSP]       = rtcb->xcp.saved_rsp;
          rtcb->xcp.sigreturn = 0;

          /* For kernel mode, we should be already on a correct kernel stack
           * which was recovered in x86_64_syscall_entry.
           */

          break;
        }
#endif  /* CONFIG_BUILD_KERNEL */

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

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* Store reference to user RSP for signals */

          rtcb->xcp.saved_ursp = regs[REG_RSP];
#endif

          /* Re-enable interrupts if enabled before.
           * Current task RFLAGS are stored in R11.
           */

          if (regs[REG_R11] & X86_64_RFLAGS_IF)
            {
              up_irq_restore(X86_64_RFLAGS_IF);
            }

          /* Call syscall function */

          ret = stub(nbr, arg1, arg2, arg3, arg4, arg5, arg6);

          break;
        }
    }

  dump_syscall("Exit", regs);

  /* Store return value in RAX register */

  regs[REG_RAX] = ret;

  /* Return pointer to regs */

  return regs;
}
