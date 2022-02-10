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

#include <arch/irq.h>
#include <nuttx/sched.h>
#include <nuttx/userspace.h>

#ifdef CONFIG_LIB_SYSCALL
#  include <syscall.h>
#endif

#include "signal/signal.h"
#include "svcall.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_registerdump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SYSCALL_INFO
static void riscv_registerdump(const uintptr_t *regs)
{
  svcinfo("EPC: %" PRIxREG "\n",
          regs[REG_EPC]);
  svcinfo("A0: %" PRIxREG " A1: %" PRIxREG " A2: %" PRIxREG " A3: %" PRIxREG
          " A4: %" PRIxREG " A5: %" PRIxREG " A6: %" PRIxREG " A7: %" PRIxREG
          "\n",
          regs[REG_A0], regs[REG_A1], regs[REG_A2], regs[REG_A3],
          regs[REG_A4], regs[REG_A5], regs[REG_A6], regs[REG_A7]);
  svcinfo("T0: %" PRIxREG " T1: %" PRIxREG " T2: %" PRIxREG " T3: %" PRIxREG
          " T4: %" PRIxREG " T5: %" PRIxREG " T6: %" PRIxREG "\n",
          regs[REG_T0], regs[REG_T1], regs[REG_T2], regs[REG_T3],
          regs[REG_T4], regs[REG_T5], regs[REG_T6]);
  svcinfo("S0: %" PRIxREG " S1: %" PRIxREG " S2: %" PRIxREG " S3: %" PRIxREG
          " S4: %" PRIxREG " S5: %" PRIxREG " S6: %" PRIxREG " S7: %" PRIxREG
          "\n",
          regs[REG_S0], regs[REG_S1], regs[REG_S2], regs[REG_S3],
          regs[REG_S4], regs[REG_S5], regs[REG_S6], regs[REG_S7]);
  svcinfo("S8: %" PRIxREG " S9: %" PRIxREG " S10: %"PRIxREG
          " S11: %" PRIxREG "\n",
          regs[REG_S8], regs[REG_S9], regs[REG_S10], regs[REG_S11]);
#ifdef RISCV_SAVE_GP
  svcinfo("GP: %" PRIxREG " SP: %" PRIxREG " FP: %" PRIxREG
          " TP: %" PRIxREG " RA: %" PRIxREG "\n",
          regs[REG_GP], regs[REG_SP], regs[REG_FP], regs[REG_TP],
          regs[REG_RA]);
#else
  svcinfo("SP: %" PRIxREG " FP: %" PRIxREG " TP: %" PRIxREG " RA: %" PRIxREG
          "\n",
          regs[REG_SP], regs[REG_FP], regs[REG_TP], regs[REG_RA]);
#endif
}
#endif

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
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL
static void dispatch_syscall(void) naked_function;
#ifdef CONFIG_ARCH_RV64
static void dispatch_syscall(void)
{
  asm volatile
    (
     " addi sp, sp, -8\n"         /* Create a stack frame to hold ra */
     " sd   ra, 0(sp)\n"          /* Save ra in the stack frame */
     " la   t0, g_stublookup\n"   /* t0=The base of the stub lookup table */
     " slli a0, a0, 3\n"          /* a0=Offset for the stub lookup table */
     " add  t0, t0, a0\n"         /* t0=The address in the table */
     " ld   t0, 0(t0)\n"          /* t0=The address of the stub for this syscall */
     " jalr ra, t0\n"             /* Call the stub (modifies ra) */
     " ld   ra, 0(sp)\n"          /* Restore ra */
     " addi sp, sp, 8\n"          /* Destroy the stack frame */
     " mv   a2, a0\n"             /* a2=Save return value in a0 */
     " li   a0, 3\n"              /* a0=SYS_syscall_return (3) */
     " ecall"                     /* Return from the syscall */
  );
}
#else
static void dispatch_syscall(void)
{
  asm volatile
    (
     " addi sp, sp, -4\n"         /* Create a stack frame to hold ra */
     " sw   ra, 0(sp)\n"          /* Save ra in the stack frame */
     " la   t0, g_stublookup\n"   /* t0=The base of the stub lookup table */
     " slli a0, a0, 3\n"          /* a0=Offset for the stub lookup table */
     " add  t0, t0, a0\n"         /* t0=The address in the table */
     " lw   t0, 0(t0)\n"          /* t0=The address of the stub for this syscall */
     " jalr ra, t0\n"             /* Call the stub (modifies ra) */
     " lw   ra, 0(sp)\n"          /* Restore ra */
     " addi sp, sp, 4\n"          /* Destroy the stack frame */
     " mv   a2, a0\n"             /* a2=Save return value in a0 */
     " li   a0, 3\n"              /* a0=SYS_syscall_return (3) */
     " ecall"                     /* Return from the syscall */
  );
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  uintptr_t *regs = (uintptr_t *)context;

  DEBUGASSERT(regs && regs == CURRENT_REGS);

  /* Software interrupt 0 is invoked with REG_A0 (REG_X10) = system call
   * command and REG_A1-6 = variable number of
   * arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  svcinfo("Entry: regs: %p cmd: %d\n", regs, regs[REG_A0]);
  riscv_registerdump(regs);
#endif

  /* Handle the SWInt according to the command in $a0 */

  switch (regs[REG_A0])
    {
      /* A0=SYS_restore_context: This a restore context command:
       *
       * void
       *   riscv_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_restore_context
       *   A1 = restoreregs
       *
       * In this case, we simply need to set CURRENT_REGS to restore register
       * area referenced in the saved R1. context == CURRENT_REGS is the
       * normal exception return.  By setting CURRENT_REGS = context[R1], we
       * force the return to the saved context referenced in $a1.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0);
          CURRENT_REGS = (uintptr_t *)regs[REG_A1];
        }
        break;

      /* A0=SYS_switch_context: This a switch context command:
       *
       * void riscv_switchcontext(uint64_t *saveregs, uint64_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_switch_context
       *   A1 = saveregs
       *   A2 = restoreregs
       *
       * In this case, we save the context registers to the save register
       * area referenced by the saved contents of R5 and then set
       * CURRENT_REGS to the save register area referenced by the saved
       * contents of R6.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0 && regs[REG_A2] != 0);
          riscv_copystate((uintptr_t *)regs[REG_A1], regs);
          CURRENT_REGS = (uintptr_t *)regs[REG_A2];
        }
        break;

      /* A0=SYS_syscall_return: This is a SYSCALL return command:
       *
       *   void up_sycall_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_syscall_return
       *
       * We need to restore the saved return address and return in
       * unprivileged thread mode.
       */

#ifdef CONFIG_LIB_SYSCALL
      case SYS_syscall_return:
        {
          struct tcb_s *rtcb = nxsched_self();
          int index = (int)rtcb->xcp.nsyscalls - 1;

          /* Make sure that there is a saved syscall return address. */

          DEBUGASSERT(index >= 0);

          /* Setup to return to the saved syscall return address in
           * the original mode.
           */

          regs[REG_EPC]         = rtcb->xcp.syscall[index].sysreturn;
#ifdef CONFIG_BUILD_PROTECTED
          regs[REG_INT_CTX]      = rtcb->xcp.syscall[index].int_ctx;
#endif

          /* The return value must be in A0-A1.
           * dispatch_syscall() temporarily moved the value for R0 into A2.
           */

          regs[REG_A0]         = regs[REG_A2];

          /* Save the new SYSCALL nesting level */

          rtcb->xcp.nsyscalls  = index;

          /* Handle any signal actions that were deferred while processing
           * the system call.
           */

          rtcb->flags          &= ~TCB_FLAG_SYSCALL;
          (void)nxsig_unmask_pendingsignal();
        }
        break;
#endif

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

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_task_start:
        {
          /* Set up to return to the user-space task start-up function in
           * unprivileged mode.
           */

          regs[REG_EPC]      = (uintptr_t)USERSPACE->task_startup & ~1;

          regs[REG_A0]       = regs[REG_A1]; /* Task entry */
          regs[REG_A1]       = regs[REG_A2]; /* argc */
          regs[REG_A2]       = regs[REG_A3]; /* argv */

          regs[REG_INT_CTX] &= ~MSTATUS_MPPM; /* User mode */
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
       *   R0 = SYS_pthread_start
       *   R1 = entrypt
       *   R2 = arg
       */

#if !defined(CONFIG_BUILD_FLAT) && !defined(CONFIG_DISABLE_PTHREAD)
      case SYS_pthread_start:
        {
          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode.
           */

          regs[REG_EPC]      = (uintptr_t)regs[REG_A1] & ~1;  /* startup */

          /* Change the parameter ordering to match the expectation of the
           * user space pthread_startup:
           */

          regs[REG_A0]       = regs[REG_A2];  /* pthread entry */
          regs[REG_A1]       = regs[REG_A3];  /* arg */
          regs[REG_INT_CTX] &= ~MSTATUS_MPPM; /* User mode */
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
       *   R0 = SYS_signal_handler
       *   R1 = sighand
       *   R2 = signo
       *   R3 = info
       *   R4 = ucontext
       */

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_signal_handler:
        {
          struct tcb_s *rtcb   = nxsched_self();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn  = regs[REG_EPC];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

          regs[REG_EPC]      = (uintptr_t)USERSPACE->signal_handler & ~1;
          regs[REG_INT_CTX] &= ~MSTATUS_MPPM; /* User mode */

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_A0]       = regs[REG_A1]; /* sighand */
          regs[REG_A1]       = regs[REG_A2]; /* signal */
          regs[REG_A2]       = regs[REG_A3]; /* info */
          regs[REG_A3]       = regs[REG_A4]; /* ucontext */
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

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_signal_handler_return:
        {
          struct tcb_s *rtcb   = nxsched_self();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);
          regs[REG_EPC]        = rtcb->xcp.sigreturn & ~1;
          regs[REG_INT_CTX]   |= MSTATUS_MPPM; /* Machine mode */

          rtcb->xcp.sigreturn  = 0;
        }
        break;
#endif

      /* This is not an architecture-specify system call.  If NuttX is built
       * as a standalone kernel with a system call interface, then all of the
       * additional system calls must be handled as in the default case.
       */

      default:
        {
#ifdef CONFIG_LIB_SYSCALL
          struct tcb_s *rtcb = nxsched_self();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(CURRENT_REGS[REG_A0] < SYS_maxsyscall);

          /* Make sure that we got here that there is a no saved syscall
           * return address.  We cannot yet handle nested system calls.
           */

          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to dispatch_syscall in privileged mode. */

          rtcb->xcp.syscall[index].sysreturn  = regs[REG_EPC];
#ifdef CONFIG_BUILD_PROTECTED
          rtcb->xcp.syscall[index].int_ctx     = regs[REG_INT_CTX];
#endif

          rtcb->xcp.nsyscalls  = index + 1;

          regs[REG_EPC]        = (uintptr_t)dispatch_syscall & ~1;

#ifdef CONFIG_BUILD_PROTECTED
          regs[REG_INT_CTX]   |= MSTATUS_MPPM; /* Machine mode */
#endif

          /* Offset A0 to account for the reserved values */

          regs[REG_A0]        -= CONFIG_SYS_RESERVED;

          /* Indicate that we are in a syscall handler. */

          rtcb->flags         |= TCB_FLAG_SYSCALL;
#else
          svcerr("ERROR: Bad SYS call: %" PRIdPTR "\n", regs[REG_A0]);
#endif
        }
        break;
    }

  /* Report what happened.  That might difficult in the case of a context
   * switch
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  if (regs != CURRENT_REGS)
    {
      svcinfo("SWInt Return: Context switch!\n");
      riscv_registerdump((const uintptr_t *)CURRENT_REGS);
    }
  else
    {
      svcinfo("SWInt Return: %d\n", regs[REG_A0]);
    }
#endif

  return OK;
}
