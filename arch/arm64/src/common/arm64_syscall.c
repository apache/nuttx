/****************************************************************************
 * arch/arm64/src/common/arm64_syscall.c
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

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/addrenv.h>

#include "addrenv.h"
#include "arch/irq.h"
#include "arm64_internal.h"
#include "arm64_fatal.h"
#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef uintptr_t (*syscall_t)(unsigned int, ...);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void  arm64_dump_syscall(const char *tag, uint64_t cmd,
                                const uint64_t *regs)
{
  svcinfo("SYSCALL %s: regs: %p cmd: %" PRId64 "\n", tag, regs, cmd);

  svcinfo("x0:  0x%-16lx  x1:  0x%lx\n",
          regs[REG_X0], regs[REG_X1]);
  svcinfo("x2:  0x%-16lx  x3:  0x%lx\n",
          regs[REG_X2], regs[REG_X3]);
  svcinfo("x4:  0x%-16lx  x5:  0x%lx\n",
          regs[REG_X4], regs[REG_X5]);
  svcinfo("x6:  0x%-16lx  x7:  0x%lx\n",
          regs[REG_X6], regs[REG_X7]);
}

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dispatch_syscall
 *
 * Description:
 *   Call the stub function corresponding to the system call.  NOTE the non-
 *   standard parameter passing:
 *
 *     R0 = SYS_ call number
 *     R1 = parm0
 *     R2 = parm1
 *     R3 = parm2
 *     R4 = parm3
 *     R5 = parm4
 *     R6 = parm5
 *     R7 = context (aka SP)
 *
 ****************************************************************************/

uintptr_t dispatch_syscall(unsigned int nbr, uintptr_t parm1,
                           uintptr_t parm2, uintptr_t parm3,
                           uintptr_t parm4, uintptr_t parm5,
                           uintptr_t parm6)
{
  struct tcb_s *rtcb         = this_task();
  register long x0 asm("x0") = (long)(nbr);
  register long x1 asm("x1") = (long)(parm1);
  register long x2 asm("x2") = (long)(parm2);
  register long x3 asm("x3") = (long)(parm3);
  register long x4 asm("x4") = (long)(parm4);
  register long x5 asm("x5") = (long)(parm5);
  register long x6 asm("x6") = (long)(parm6);
  syscall_t do_syscall;
  uintptr_t ret;

  /* Valid system call ? */

  if (x0 > SYS_maxsyscall)
    {
      /* Nope, get out */

      return -ENOSYS;
    }

  /* Indicate that we are in a syscall handler */

  rtcb->flags |= TCB_FLAG_SYSCALL;

  /* Offset a0 to account for the reserved syscalls */

  x0 -= CONFIG_SYS_RESERVED;

  /* Find the system call from the lookup table */

  do_syscall = (syscall_t)g_stublookup[x0];

  /* Run the system call, save return value locally */

  ret = do_syscall(x0, x1, x2, x3, x4, x5, x6);

  /* System call is now done */

  rtcb->flags &= ~TCB_FLAG_SYSCALL;

  /* Unmask any pending signals now */

  nxsig_unmask_pendingsignal();

  return ret;
}
#endif

/****************************************************************************
 * Name: arm64_syscall
 *
 * Description:
 *   task switch syscall
 *
 ****************************************************************************/

uint64_t *arm64_syscall(uint64_t *regs)
{
  int cpu = this_cpu();
  struct tcb_s **running_task = &g_running_tasks[cpu];
  struct tcb_s *tcb = this_task();
  uint64_t cmd;
#ifdef CONFIG_BUILD_KERNEL
  uint64_t             spsr;
#endif

  /* Nested interrupts are not supported */

  DEBUGASSERT(regs);

  /* The SYSCALL command is in x0 on entry.  Parameters follow in x1..x7 */

  cmd = regs[REG_X0];

  /* if cmd == SYS_restore_context (*running_task)->xcp.regs is valid
   * should not be overwriten
   */

  if (cmd != SYS_restore_context)
    {
      (*running_task)->xcp.regs = regs;
    }

  arm64_dump_syscall(__func__, cmd, regs);

  switch (cmd)
    {
      case SYS_restore_context:

        /* Update scheduler parameters */

        nxsched_resume_scheduler(tcb);

        /* Restore the cpu lock */

        restore_critical_section(tcb, cpu);
#ifdef CONFIG_ARCH_ADDRENV
        addrenv_switch(tcb);
#endif
        break;

      case SYS_switch_context:

        /* Update scheduler parameters */

        nxsched_suspend_scheduler(*running_task);
        nxsched_resume_scheduler(tcb);
        *running_task = tcb;

        /* Restore the cpu lock */

        restore_critical_section(tcb, cpu);
#ifdef CONFIG_ARCH_ADDRENV
        addrenv_switch(tcb);
#endif
        break;

#ifdef CONFIG_BUILD_KERNEL
      /* R0=SYS_signal_handler:  This a user signal handler callback
       *
       * void signal_handler(_sa_sigaction_t sighand, int signo,
       *                     siginfo_t *info, void *ucontext);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_signal_handler
       *   R1 = sighand
       *   R2 = signo
       *   R3 = info
       *        ucontext (on the stack)
       */

      case SYS_signal_handler:
        {
          struct tcb_s *rtcb = this_task();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn  = regs[REG_ELR];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

          regs[REG_ELR]  = (uint64_t)ARCH_DATA_RESERVE->ar_sigtramp;
          spsr           = regs[REG_SPSR] & ~SPSR_MODE_MASK;
          regs[REG_SPSR] = spsr | SPSR_MODE_EL0T;

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_X0]   = regs[REG_X1]; /* sighand */
          regs[REG_X1]   = regs[REG_X2]; /* signal */
          regs[REG_X2]   = regs[REG_X3]; /* info */
          regs[REG_X3]   = regs[REG_X4]; /* ucontext */

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

              /* Create a frame for info and copy the kernel info */

              rtcb->xcp.ustkptr = (uintptr_t *)read_sysreg(sp_el0);
              usp = (uintptr_t)rtcb->xcp.ustkptr - sizeof(siginfo_t);
              memcpy((void *)usp, (void *)regs[REG_X2], sizeof(siginfo_t));

              /* Now set the updated SP and user copy of "info" to R2 */

              write_sysreg(usp, sp_el0);
              regs[REG_X2] = usp;
            }
#endif
        }
        break;
#endif

#ifdef CONFIG_BUILD_KERNEL
      /* R0=SYS_signal_handler_return:  This a user signal handler callback
       *
       *   void signal_handler_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_signal_handler_return
       */

      case SYS_signal_handler_return:
        {
          struct tcb_s *rtcb = this_task();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);

          regs[REG_ELR]        = rtcb->xcp.sigreturn;
          spsr                 = regs[REG_SPSR] & ~SPSR_MODE_MASK;
          regs[REG_SPSR]       = spsr | SPSR_MODE_EL1H;
          rtcb->xcp.sigreturn  = 0;

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* Restore the original user SP, destroying the signal frame */

          write_sysreg(rtcb->xcp.ustkptr, sp_el0);
#endif
        }
        break;
#endif

      default:
        {
          svcerr("ERROR: Bad SYS call: 0x%" PRIx64 "\n", cmd);
          return 0;
        }
        break;
    }

  return tcb->xcp.regs;
}
