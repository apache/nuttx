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

#include "signal/signal.h"
#include "riscv_internal.h"
#include "addrenv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
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
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL
static void dispatch_syscall(void) naked_function;
static void dispatch_syscall(void)
{
  asm volatile
    (
     "addi sp, sp, -" STACK_FRAME_SIZE "\n" /* Create a stack frame to hold ra */
     REGSTORE " ra, 0(sp)\n"                /* Save ra in the stack frame */
     "la   t0, g_stublookup\n"              /* t0=The base of the stub lookup table */
#ifdef CONFIG_ARCH_RV32
     "slli a0, a0, 2\n"                     /* a0=Offset for the stub lookup table */
#else
     "slli a0, a0, 3\n"                     /* a0=Offset for the stub lookup table */
#endif
     "add  t0, t0, a0\n"                    /* t0=The address in the table */
     REGLOAD " t0, 0(t0)\n"                 /* t0=The address of the stub for this syscall */
     "jalr ra, t0\n"                        /* Call the stub (modifies ra) */
     REGLOAD " ra, 0(sp)\n"                 /* Restore ra */
     "addi sp, sp, " STACK_FRAME_SIZE "\n"  /* Destroy the stack frame */
     "mv   a2, a0\n"                        /* a2=Save return value in a0 */
     "li   a0, 3\n"                         /* a0=SYS_syscall_return (3) */
#ifdef CONFIG_ARCH_USE_S_MODE
     "j    sys_call2"                       /* Return from the syscall */
#else
     "ecall"                                /* Return from the syscall */
#endif
  );
}
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
  up_dump_register(regs);
#endif

  /* Handle the SWInt according to the command in $a0 */

  switch (regs[REG_A0])
    {
      /* A0=SYS_save_context:  This is a save context command:
       *
       *   int up_saveusercontext(void *saveregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_save_context
       *   A1 = saveregs
       *
       * In this case, we simply need to copy the current registers to the
       * save register space references in the saved A1 and return.
       */

      case SYS_save_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0);
          riscv_copystate((uintptr_t *)regs[REG_A1], regs);
        }
        break;

      /* A0=SYS_restore_context: This a restore context command:
       *
       * void
       * riscv_fullcontextrestore(uintptr_t *restoreregs) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_restore_context
       *   A1 = restoreregs
       *
       * In this case, we simply need to set CURRENT_REGS to restore register
       * area referenced in the saved A1. context == CURRENT_REGS is the
       * normal exception return.  By setting CURRENT_REGS = context[A1], we
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
       * void
       * riscv_switchcontext(uintptr_t *saveregs, uintptr_t *restoreregs);
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
          *(uintptr_t **)regs[REG_A1] = (uintptr_t *)regs;
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
#ifndef CONFIG_BUILD_FLAT
          regs[REG_INT_CTX]     = rtcb->xcp.syscall[index].int_ctx;
#endif

          /* The return value must be in A0-A1.
           * dispatch_syscall() temporarily moved the value for R0 into A2.
           */

          regs[REG_A0]         = regs[REG_A2];

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* If this is the outermost SYSCALL and if there is a saved user
           * stack pointer, then restore the user stack pointer on this
           * final return to user code.
           */

          if (index == 0 && rtcb->xcp.ustkptr != NULL)
            {
              regs[REG_SP]      = (uintptr_t)rtcb->xcp.ustkptr;
              rtcb->xcp.ustkptr = NULL;
            }
#endif

          /* Save the new SYSCALL nesting level */

          rtcb->xcp.nsyscalls  = index;

          /* Handle any signal actions that were deferred while processing
           * the system call.
           */

          rtcb->flags          &= ~TCB_FLAG_SYSCALL;
          nxsig_unmask_pendingsignal();
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

#ifndef CONFIG_BUILD_FLAT
      case SYS_task_start:
        {
          /* Set up to return to the user-space task start-up function in
           * unprivileged mode.
           */

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
          struct tcb_s *rtcb   = nxsched_self();

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

              DEBUGASSERT(rtcb->xcp.kstkptr == NULL);

              /* Copy "info" into user stack */

              if (rtcb->xcp.sigdeliver)
                {
                  usp = rtcb->xcp.saved_regs[REG_SP];
                }
              else
                {
                  usp = rtcb->xcp.regs[REG_SP];
                }

              /* Create a frame for info and copy the kernel info */

              usp = usp - sizeof(siginfo_t);
              memcpy((void *)usp, (void *)regs[REG_A2], sizeof(siginfo_t));

              /* Now set the updated SP and user copy of "info" to A2 */

              rtcb->xcp.kstkptr = (uintptr_t *)regs[REG_SP];
              regs[REG_SP]      = usp;
              regs[REG_A2]      = usp;
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
          struct tcb_s *rtcb   = nxsched_self();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);
          regs[REG_EPC]        = rtcb->xcp.sigreturn;
          regs[REG_INT_CTX]   |= STATUS_PPP; /* Privileged mode */

          rtcb->xcp.sigreturn  = 0;

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* We must enter here be using the user stack.  We need to switch
           * to back to the kernel user stack before returning to the kernel
           * mode signal trampoline.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              DEBUGASSERT(rtcb->xcp.kstkptr != NULL);

              regs[REG_SP]      = (uintptr_t)rtcb->xcp.kstkptr;
              rtcb->xcp.kstkptr = NULL;
            }
#endif
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
#ifndef CONFIG_BUILD_FLAT
          rtcb->xcp.syscall[index].int_ctx    = regs[REG_INT_CTX];
#endif

          rtcb->xcp.nsyscalls  = index + 1;

          regs[REG_EPC]        = (uintptr_t)dispatch_syscall;

#ifndef CONFIG_BUILD_FLAT
          regs[REG_INT_CTX]   |= STATUS_PPP; /* Privileged mode */
#endif

          /* Offset A0 to account for the reserved values */

          regs[REG_A0]        -= CONFIG_SYS_RESERVED;

          /* Indicate that we are in a syscall handler. */

          rtcb->flags         |= TCB_FLAG_SYSCALL;
#else
          svcerr("ERROR: Bad SYS call: %" PRIdPTR "\n", regs[REG_A0]);
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* If this is the first SYSCALL and if there is an allocated
           * kernel stack, then switch to the kernel stack.
           */

          if (index == 0 && rtcb->xcp.kstack != NULL)
            {
              rtcb->xcp.ustkptr = (uintptr_t *)regs[REG_SP];
              if (rtcb->xcp.kstkptr != NULL)
                {
                  regs[REG_SP]  = (uintptr_t)rtcb->xcp.kstkptr;
                }
              else
                {
                  regs[REG_SP]  = (uintptr_t)rtcb->xcp.kstack +
                                  ARCH_KERNEL_STACKSIZE;
                }
            }
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
      up_dump_register(CURRENT_REGS);
    }
  else
    {
      svcinfo("SWInt Return: %" PRIxPTR "\n", regs[REG_A0]);
    }
#endif

  return OK;
}
