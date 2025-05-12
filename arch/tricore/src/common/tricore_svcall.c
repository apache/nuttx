/****************************************************************************
 * arch/tricore/src/common/tricore_svcall.c
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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <nuttx/userspace.h>
#include <syscall.h>

#include <arch/barriers.h>
#include <arch/irq.h>
#include <sched/sched.h>
#include <nuttx/sched.h>

#include "tricore_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL
static void dispatch_syscall(void)
{
  __asm__ __volatile__ (
#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
    "mov.aa    a14, %0\n\t"
    "sh        d15, d4, #2\n\t"
    "addsc.a   a14, a14, d15, #0\n\t"
    "sub.a     sp, #12\n\t"
    "st.w      [sp]8, d10\n\t"
    "st.w      [sp]4, d9\n\t"
    "st.w      [sp]0, d8\n\t"
    "ld.a      a15, [a14]0\n\t"
    "calli     a15\n\t"
    "addih.a   sp, sp, #12\n\t"
#else
    "mov.aa    %%a14, %0\n\t"
    "sh        %%d15, %%d4, 2\n\t"
    "addsc.a   %%a14, %%a14, %%d15, 0\n\t"
    "sub.a     %%sp, 12\n\t"
    "st.w      [%%sp]8, %%d10\n\t"
    "st.w      [%%sp]4, %%d9\n\t"
    "st.w      [%%sp]0, %%d8\n\t"
    "ld.a      %%a15, [%%a14]0\n\t"
    "calli     %%a15\n\t"
    "addih.a   %%sp, %%sp, 12\n\t"
#endif
    ::"a"(g_stublookup)
    : "memory"
  );
  sys_call0(SYS_syscall_return);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

void tricore_svcall(volatile void *trap)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  struct tcb_s *tcb = this_task();
  uintptr_t *plregs;
  uintptr_t *puregs;
  uint32_t cmd;

  /* DSYNC instruction should be executed immediately prior to the MFCR */

  UP_DSB();

  plregs = tricore_csa2addr(tricore_mfcr(TRICORE_CPU_PCXI));
  puregs = tricore_csa2addr(plregs[REG_LPCXI]);

  cmd = puregs[REG_D8];

  if (cmd != SYS_restore_context)
    {
      (*running_task)->xcp.regs = plregs;
    }
  else
    {
      tricore_reclaim_csa(tricore_addr2csa(plregs));
    }

  /* Handle the SVCall according to the command in R0 */

  switch (cmd)
    {
      case SYS_switch_context:
        nxsched_switch_context(*running_task, tcb);

      case SYS_restore_context:
        *running_task = tcb;
        tricore_mtcr(TRICORE_CPU_PCXI, tricore_addr2csa(tcb->xcp.regs));
        UP_ISB();
        break;

#ifdef CONFIG_LIB_SYSCALL
      case SYS_syscall_return:
        {
          struct tcb_s *rtcb = this_task();
          int index = rtcb->xcp.nsyscalls - 1;
          int ret = plregs[REG_D2];

          puregs[REG_UPCXI] = 0;
          tricore_reclaim_csa(tricore_addr2csa(plregs));

          plregs = rtcb->xcp.syscall_regs[index];
          puregs = tricore_csa2addr(plregs[REG_LPCXI]);
          puregs[REG_D8] = ret;
          rtcb->xcp.nsyscalls = index;
          rtcb->xcp.regs = plregs;
          tricore_mtcr(TRICORE_CPU_PCXI,
                       tricore_addr2csa(rtcb->xcp.regs));
          UP_ISB();
        }
        break;
#endif

      /* REG_D8 = SYS_task_start:  This a user task start
       *
       * void up_task_start(main_t taskentry, int argc, char *argv[])
       *   noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   REG_D8  = SYS_task_start
       *   REG_D9  = taskentry
       *   REG_D10 = argc
       *   REG_D11 = argv
       */

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_task_start:
        {
          /* Set up to return to the user-space _start function in
           * unprivileged mode.  We need:
           *
           *   PC  = task_startup
           *   PSW = user mode
           *   A4  = taskentry
           *   D4  = argc
           *   A4  = argv
           */

          /* Updata PC */

          plregs[REG_LPC] = (uintptr_t)USERSPACE->task_startup;

          /* Updata function args */

          plregs[REG_A4] = puregs[REG_D9];
          plregs[REG_D4] = puregs[REG_D10];
          plregs[REG_A5] = puregs[REG_D11];

          /* Return unprivileged mode */

          puregs[REG_PSW] =
            (puregs[REG_PSW] & (~PSW_MODE_MASK) & (~PSW_PRS_MASK)) |
            (PSW_IO_USER0 | PSW_PRS_USER);
        }
        break;
#endif

      /* REG_D8 = SYS_pthread_start:  This a user pthread start
       *
       * void up_pthread_start(pthread_startroutine_t entrypt,
       *                       pthread_addr_t arg) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   REG_D8  = SYS_pthread_start
       *   REG_D9  = startup (trampoline)
       *   REG_D10 = entrypt
       *   REG_D11 = arg
       */

#if !defined(CONFIG_BUILD_FLAT) && !defined(CONFIG_DISABLE_PTHREAD)
      case SYS_pthread_start:
        {
          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode.
           *
           * PC  = startup
           * PSW = user mode
           * A4  = entrypt
           * A5  = arg
           */

          /* Updata PC */

          plregs[REG_LPC] = puregs[REG_D9];

          /* Updata function args */

          plregs[REG_A4] = puregs[REG_D10];
          plregs[REG_A5] = puregs[REG_D11];

          /* Return unprivileged mode */

          puregs[REG_PSW] =
            (puregs[REG_PSW] & (~PSW_MODE_MASK) & (~PSW_PRS_MASK)) |
            (PSW_IO_USER0 | PSW_PRS_USER);
        }
        break;
#endif

      /* REG_D8 = SYS_signal_handler:  This a user signal handler callback
       *
       * void signal_handler(_sa_sigaction_t sighand, int signo,
       *                     siginfo_t *info, void *ucontext);
       *
       * At this point, the following values are saved in context:
       *
       *   D8  = SYS_signal_handler
       *   D9  = sighand
       *   D10 = signo
       *   D11 = info
       *   D12 = ucontext
       */

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_signal_handler:
        {
          struct tcb_s *rtcb = this_task();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn = plregs[REG_LPC];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

          plregs[REG_LPC] = (uintptr_t)USERSPACE->signal_handler;

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          plregs[REG_A4] = puregs[REG_D9];  /* sighand */
          plregs[REG_D4] = puregs[REG_D10]; /* signal */
          plregs[REG_A5] = puregs[REG_D11]; /* info */
          plregs[REG_A6] = puregs[REG_D12]; /* ucontext */

          /* Return unprivileged mode */

          puregs[REG_PSW] =
            (puregs[REG_PSW] & (~PSW_MODE_MASK) & (~PSW_PRS_MASK)) |
            (PSW_IO_USER0 | PSW_PRS_USER);
        }
        break;
#endif

      /* REG_D8 = SYS_signal_handler_return:  This a user signal handler
       * callback
       */

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_signal_handler_return:
        {
          struct tcb_s *rtcb = this_task();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);

          plregs[REG_LPC] = rtcb->xcp.sigreturn;

          rtcb->xcp.sigreturn = 0;

          /* Return privileged mode */

          puregs[REG_PSW] =
            (puregs[REG_PSW] & (~PSW_MODE_MASK) & (~PSW_PRS_MASK)) |
            (PSW_IO_SUPERVISOR);
        }
        break;
#endif

      default:
        {
#ifdef CONFIG_LIB_SYSCALL
          struct tcb_s *rtcb = nxsched_self();
          uintptr_t *regs = plregs;
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(cmd >= CONFIG_SYS_RESERVED && cmd < SYS_maxsyscall);

          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to dispatch_syscall in privileged mode. */

          rtcb->xcp.syscall_regs[index] = plregs;
          rtcb->xcp.nsyscalls = index + 1;

          /* New plregs and puregs to dispatch_syscall */

          plregs = tricore_alloc_csa(
            (uintptr_t)dispatch_syscall,
            STACK_ALIGN_DOWN(puregs[REG_SP]),
            (puregs[REG_PSW] & (~PSW_MODE_MASK) & (~PSW_PRS_MASK)) |
            (PSW_IO_SUPERVISOR),
            !(plregs[REG_LPCXI] & PCXI_PIE));
          puregs = tricore_csa2addr(plregs[REG_LPCXI]);

          /* Args passed to dispatch_syscall */

          puregs[REG_UPCXI] = tricore_addr2csa(regs);
          regs = tricore_csa2addr(regs[REG_LPCXI]);
          plregs[REG_D4] = regs[REG_D8] - CONFIG_SYS_RESERVED;
          plregs[REG_D5] = regs[REG_D9];
          plregs[REG_D6] = regs[REG_D10];
          plregs[REG_D7] = regs[REG_D11];
          puregs[REG_D8] = regs[REG_D12];
          puregs[REG_D9] = regs[REG_D13];
          puregs[REG_D10] = regs[REG_D14];

          rtcb->xcp.regs = plregs;
          tricore_mtcr(TRICORE_CPU_PCXI, tricore_addr2csa(plregs));
          UP_ISB();
#else
          svcerr("ERROR: Bad SYS call: %d\n", (int)puregs[REG_D8]);
#endif
        }
        break;
    }

  /* Return to the caller, tricore_svcall is entered by a jump */

  tricore_jump_backto_link();
}
