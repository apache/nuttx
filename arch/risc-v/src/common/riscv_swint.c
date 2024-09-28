/****************************************************************************
 * arch/risc-v/src/common/riscv_swint.c
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
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * Name: do_syscall
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
 *
 * Note:
 *   Do not allow the compiler to inline this function, as it does a jump to
 *   another procedure which can clobber any register and the compiler will
 *   not understand it happens.
 *
 ****************************************************************************/

static uintptr_t do_syscall(unsigned int nbr, uintptr_t parm1,
                            uintptr_t parm2, uintptr_t parm3,
                            uintptr_t parm4, uintptr_t parm5,
                            uintptr_t parm6) noinline_function;
static uintptr_t do_syscall(unsigned int nbr, uintptr_t parm1,
                            uintptr_t parm2, uintptr_t parm3,
                            uintptr_t parm4, uintptr_t parm5,
                            uintptr_t parm6)
{
  register long a0 asm("a0") = (long)(nbr);
  register long a1 asm("a1") = (long)(parm1);
  register long a2 asm("a2") = (long)(parm2);
  register long a3 asm("a3") = (long)(parm3);
  register long a4 asm("a4") = (long)(parm4);
  register long a5 asm("a5") = (long)(parm5);
  register long a6 asm("a6") = (long)(parm6);

  asm volatile
    (
     "la   t0, g_stublookup\n" /* t0=The base of the stub lookup table */
#ifdef CONFIG_ARCH_RV32
     "slli a0, a0, 2\n"        /* a0=Offset for the stub lookup table */
#else
     "slli a0, a0, 3\n"        /* a0=Offset for the stub lookup table */
#endif
     "add  t0, t0, a0\n"       /* t0=The address in the table */
     REGLOAD " t0, 0(t0)\n"    /* t0=The address of the stub for this syscall */
     "jalr ra, t0\n"           /* Call the stub (modifies ra) */
     : "+r"(a0)
     : "r"(a1), "r"(a2), "r"(a3), "r"(a4), "r"(a5), "r"(a6)
     : "t0", "ra", "memory"
  );

  return a0;
}

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
  register long a0 asm("a0") = (long)(nbr);
  register long a1 asm("a1") = (long)(parm1);
  register long a2 asm("a2") = (long)(parm2);
  register long a3 asm("a3") = (long)(parm3);
  register long a4 asm("a4") = (long)(parm4);
  register long a5 asm("a5") = (long)(parm5);
  register long a6 asm("a6") = (long)(parm6);
  register struct tcb_s *rtcb asm("tp");
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
  uintreg_t *new_regs = regs;

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
      /* A0=SYS_restore_context: This a restore context command:
       *
       * void
       * void riscv_fullcontextrestore(struct tcb_s *prev) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_restore_context
       *   A1 = next
       */

      case SYS_restore_context:
        {
          struct tcb_s *next = (struct tcb_s *)(uintptr_t)regs[REG_A1];

          DEBUGASSERT(regs[REG_A1] != 0);
          new_regs = next->xcp.regs;
          riscv_restorecontext(next);
        }
        break;

      /* A0=SYS_switch_context: This a switch context command:
       *
       * void
       * riscv_switchcontext(struct tcb_s *prev, struct tcb_s *next);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_switch_context
       *   A1 = prev
       *   A2 = next
       *
       * In this case, we save the context registers to the save register
       * area referenced by the saved contents of R5.
       */

      case SYS_switch_context:
        {
          struct tcb_s *prev = (struct tcb_s *)(uintptr_t)regs[REG_A1];
          struct tcb_s *next = (struct tcb_s *)(uintptr_t)regs[REG_A2];

          DEBUGASSERT(regs[REG_A1] != 0 && regs[REG_A2] != 0);
          riscv_savecontext(prev);
          new_regs = next->xcp.regs;
          riscv_restorecontext(next);
        }
        break;

      /* R0=SYS_task_start:  This a user task start
       *
       *   void up_task_start(main_t taskentry, int argc,
       *                      char *argv[]) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_task_start
       *   A1 = taskentry
       *   A2 = argc
       *   A3 = argv
       */

#ifndef CONFIG_BUILD_FLAT
      case SYS_task_start:
        {
          /* Set up to return to the user-space task start-up function in
           * unprivileged mode.
           */

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* Set the user stack pointer as we are about to return to user */

          struct tcb_s *tcb  = this_task();
          regs[REG_SP]       = (uintptr_t)tcb->xcp.ustkptr;
          tcb->xcp.ustkptr   = NULL;
#endif

#if defined (CONFIG_BUILD_PROTECTED)
          /* Use the nxtask_startup trampoline function */

          regs[REG_EPC]      = (uintptr_t)USERSPACE->task_startup;
          regs[REG_A0]       = regs[REG_A1]; /* Task entry */
          regs[REG_A1]       = regs[REG_A2]; /* argc */
          regs[REG_A2]       = regs[REG_A3]; /* argv */
#else
          /* Start the user task directly */

          regs[REG_EPC]      = (uintptr_t)regs[REG_A1];
          regs[REG_A0]       = regs[REG_A2]; /* argc */
          regs[REG_A1]       = regs[REG_A3]; /* argv */
#endif
          regs[REG_INT_CTX] &= ~STATUS_PPP; /* User mode */
        }
        break;
#endif

      /* R0=SYS_pthread_start:  This a user pthread start
       *
       *   void up_pthread_start(pthread_startroutine_t entrypt,
       *                         pthread_addr_t arg) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_pthread_start
       *   A1 = entrypt
       *   A2 = arg
       */

#if !defined(CONFIG_BUILD_FLAT) && !defined(CONFIG_DISABLE_PTHREAD)
      case SYS_pthread_start:
        {
          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode.
           */

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* Set the user stack pointer as we are about to return to user */

          struct tcb_s *tcb  = this_task();
          regs[REG_SP]       = (uintptr_t)tcb->xcp.ustkptr;
          tcb->xcp.ustkptr   = NULL;
#endif

          regs[REG_EPC]      = (uintptr_t)regs[REG_A1];  /* startup */

          /* Change the parameter ordering to match the expectation of the
           * user space pthread_startup:
           */

          regs[REG_A0]       = regs[REG_A2];  /* pthread entry */
          regs[REG_A1]       = regs[REG_A3];  /* arg */
          regs[REG_INT_CTX] &= ~STATUS_PPP;   /* User mode */
        }
        break;
#endif

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
  if (regs != new_regs)
    {
      svcinfo("SWInt Return: Context switch!\n");
      up_dump_register(new_regs);
    }
  else
    {
      svcinfo("SWInt Return: %" PRIxPTR "\n", regs[REG_A0]);
    }
#endif

  if (regs != new_regs)
    {
      restore_critical_section(this_task(), this_cpu());
    }

  return OK;
}
