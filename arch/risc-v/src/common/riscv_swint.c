/****************************************************************************
 * arch/risc-v/src/common/riscv_swint.c
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

#include <arch/irq.h>
#include <nuttx/addrenv.h>
#include <nuttx/sched.h>
#include <nuttx/userspace.h>

#ifdef CONFIG_LIB_SYSCALL
#  include <syscall.h>
#endif

#include "sched/sched.h"
#include "signal/signal.h"
#include "riscv_internal.h"
#include "addrenv.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef uintptr_t (*syscall_t)(unsigned int, ...);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *     A0 = SYS_ call number
 *     A1 = parm0
 *     A2 = parm1
 *     A3 = parm2
 *     A4 = parm3
 *     A5 = parm4
 *     A6 = parm5
 *     A7 = context (aka SP)
 *
 ****************************************************************************/

uintptr_t dispatch_syscall(unsigned int nbr, uintptr_t parm1,
                           uintptr_t parm2, uintptr_t parm3,
                           uintptr_t parm4, uintptr_t parm5,
                           uintptr_t parm6, void *context)
{
  struct tcb_s *rtcb         = this_task();
  register long a0 asm("a0") = (long)(nbr);
  register long a1 asm("a1") = (long)(parm1);
  register long a2 asm("a2") = (long)(parm2);
  register long a3 asm("a3") = (long)(parm3);
  register long a4 asm("a4") = (long)(parm4);
  register long a5 asm("a5") = (long)(parm5);
  register long a6 asm("a6") = (long)(parm6);
  syscall_t do_syscall;
  uintptr_t ret;

  /* Valid system call ? */

  if (a0 > SYS_maxsyscall)
    {
      /* Nope, get out */

      return -ENOSYS;
    }

  /* Set the user register context to TCB */

  rtcb->xcp.sregs = context;

  /* Indicate that we are in a syscall handler */

  rtcb->flags |= TCB_FLAG_SYSCALL;

  /* Offset a0 to account for the reserved syscalls */

  a0 -= CONFIG_SYS_RESERVED;

  /* Find the system call from the lookup table */

  do_syscall = (syscall_t)g_stublookup[a0];

  /* Run the system call, save return value locally */

  ret = do_syscall(a0, a1, a2, a3, a4, a5, a6);

  /* System call is now done */

  rtcb->flags &= ~TCB_FLAG_SYSCALL;

  /* Unmask any pending signals now */

  nxsig_unmask_pendingsignal();

  return ret;
}
#endif

/****************************************************************************
 * Name: riscv_swint
 *
 * Description:
 *   This is software interrupt exception handler that performs context
 *   switching and manages system calls
 *
 ****************************************************************************/

int riscv_swint(int irq, void *context, void *arg)
{
  uintreg_t *regs = (uintreg_t *)context;
  struct tcb_s *tcb = this_task();
  int cpu = this_cpu();

  /* Software interrupt 0 is invoked with REG_A0 (REG_X10) = system call
   * command and REG_A1-6 = variable number of
   * arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  svcinfo("Entry: regs: %p cmd: %d\n", regs, regs[REG_A0]);
  up_dump_register(regs);
#endif

  /* Handle the SWInt according to the command in $a0 */

  switch (regs[REG_A0])
    {
      case SYS_restore_context:
        {
          riscv_restorecontext(tcb);
          restore_critical_section(tcb, cpu);
        }
        break;

      case SYS_switch_context:
        {
          riscv_savecontext(g_running_tasks[cpu]);
          riscv_restorecontext(tcb);
          restore_critical_section(tcb, cpu);
        }
        break;

      /* R0=SYS_signal_handler:  This a user signal handler callback
       *
       * void signal_handler(_sa_sigaction_t sighand, int signo,
       *                     siginfo_t *info, void *ucontext);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_signal_handler
       *   A1 = sighand
       *   A2 = signo
       *   A3 = info
       *   A4 = ucontext
       */

#ifndef CONFIG_BUILD_FLAT
      case SYS_signal_handler:
        {
          struct tcb_s *rtcb   = this_task();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn  = regs[REG_EPC];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

#if defined (CONFIG_BUILD_PROTECTED)
          regs[REG_EPC]        = (uintptr_t)USERSPACE->signal_handler;
#else
          regs[REG_EPC]        = (uintptr_t)ARCH_DATA_RESERVE->ar_sigtramp;
#endif
          regs[REG_INT_CTX]   &= ~STATUS_PPP; /* User mode */

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_A0]         = regs[REG_A1]; /* sighand */
          regs[REG_A1]         = regs[REG_A2]; /* signal */
          regs[REG_A2]         = regs[REG_A3]; /* info */
          regs[REG_A3]         = regs[REG_A4]; /* ucontext */

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* If we are signalling a user process, then we must be operating
           * on the kernel stack now.  We need to switch back to the user
           * stack before dispatching the signal handler to the user code.
           * The existence of an allocated kernel stack is sufficient
           * information to make this decision.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              uintptr_t usp;
              uintptr_t *usr_regs;

              /* Store the current kernel stack pointer so it is not lost */

              rtcb->xcp.kstkptr = (uintptr_t *)regs[REG_SP];

              /* Copy "info" into user stack */

              usr_regs = (uintptr_t *)((uintptr_t)rtcb->xcp.ktopstk -
                                                  XCPTCONTEXT_SIZE);
              usp = usr_regs[REG_SP];

              /* Create a frame for info and copy the kernel info */

              usp = usp - sizeof(siginfo_t);
              memcpy((void *)usp, (void *)regs[REG_A2], sizeof(siginfo_t));

              /* Now set the updated SP and user copy of "info" to A2 */

              regs[REG_SP] = usp;
              regs[REG_A2] = usp;
            }
#endif
        }
        break;
#endif

      /* R0=SYS_signal_handler_return:  This a user signal handler callback
       *
       *   void signal_handler_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_signal_handler_return
       */

#ifndef CONFIG_BUILD_FLAT
      case SYS_signal_handler_return:
        {
          struct tcb_s *rtcb   = this_task();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);
          regs[REG_EPC]        = rtcb->xcp.sigreturn;
          regs[REG_INT_CTX]   |= STATUS_PPP; /* Privileged mode */

          rtcb->xcp.sigreturn  = 0;

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* We must restore the original kernel stack pointer before
           * returning to the kernel mode signal trampoline.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              DEBUGASSERT(rtcb->xcp.kstkptr != NULL);

              regs[REG_SP]      = (uintptr_t)rtcb->xcp.kstkptr;
              rtcb->xcp.kstkptr = rtcb->xcp.ktopstk;
            }
#endif
        }
        break;
#endif

      default:
        DEBUGPANIC();
        break;
    }

  /* Report what happened.  That might difficult in the case of a context
   * switch
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  if (cmd <= SYS_switch_context)
    {
      svcinfo("SWInt Return: Context switch!\n");
      up_dump_register(tcb.xcp.regs);
    }
  else
    {
      svcinfo("SWInt Return: %" PRIxPTR "\n", regs[REG_A0]);
    }
#endif

  return OK;
}
