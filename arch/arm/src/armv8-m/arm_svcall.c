/****************************************************************************
 * arch/arm/src/armv8-m/arm_svcall.c
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
#include <debug.h>
#include <syscall.h>

#include <arch/irq.h>
#include <nuttx/sched.h>
#include <nuttx/userspace.h>

#include "signal/signal.h"
#include "exc_return.h"
#include "arm_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
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
 *
 *   The values of R4-R5 may be preserved in the proxy called by the user
 *   code if they are used (but otherwise will not be).
 *
 *   Register usage:
 *
 *     R0 - Need not be preserved.
 *     R1-R3 - Need to be preserved until the stub is called.  The values of
 *       R0 and R1 returned by the stub must be preserved.
 *     R4-R11 must be preserved to support the expectations of the user-space
 *       callee.  R4-R6 may have been preserved by the proxy, but don't know
 *       for sure.
 *     R12 - Need not be preserved
 *     R13 - (stack pointer)
 *     R14 - Need not be preserved
 *     R15 - (PC)
 *
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL
static void dispatch_syscall(void) naked_function;
static void dispatch_syscall(void)
{
  __asm__ __volatile__
    (
      /* Create a stack frame to hold 3 parameters + LR and SP adjustment
       * value.  Also, Ensure 8 bytes alignment. We use IP as a scratch.
       *
       * NOTE: new_SP = (orig_SP - 20) & ~7
       *              = orig_SP - 20 - ((orig_SP - 20) & ~7)
       */

      " mov ip, sp\n"                /* Calculate (orig_SP - new_SP) */
      " sub ip, ip, #20\n"
      " and ip, ip, #7\n"
      " add ip, ip, #20\n"
      " sub sp, sp, ip\n"
      " str r4, [sp, #0]\n"          /* Move parameter 4 (if any) into position */
      " str r5, [sp, #4]\n"          /* Move parameter 5 (if any) into position */
      " str r6, [sp, #8]\n"          /* Move parameter 6 (if any) into position */
      " str lr, [sp, #12]\n"         /* Save lr in the stack frame */
      " str ip, [sp, #16]\n"         /* Save (orig_SP - new_SP) value */
      " ldr ip, =g_stublookup\n"     /* R12=The base of the stub lookup table */
      " ldr ip, [ip, r0, lsl #2]\n"  /* R12=The address of the stub for this syscall */
      " blx ip\n"                    /* Call the stub (modifies lr) */
      " ldr lr, [sp, #12]\n"         /* Restore lr */
      " ldr r2, [sp, #16]\n"         /* Restore (orig_SP - new_SP) value */
      " add sp, sp, r2\n"            /* Restore SP */
      " mov r2, r0\n"                /* R2=Save return value in R2 */
      " mov r0, %0\n"                /* R0=SYS_syscall_return */
      " svc %1\n"::"i"(SYS_syscall_return),
                   "i"(SYS_syscall)  /* Return from the SYSCALL */
    );
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

int arm_svcall(int irq, void *context, void *arg)
{
  uint32_t *regs = (uint32_t *)context;
  uint32_t cmd;

  DEBUGASSERT(regs && regs == CURRENT_REGS);
  cmd = regs[REG_R0];

  /* The SVCall software interrupt is called with R0 = system call command
   * and R1..R7 =  variable number of arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
# ifndef CONFIG_DEBUG_SVCALL
  if (cmd > SYS_switch_context)
# endif
    {
      svcinfo("SVCALL Entry: regs: %p cmd: %d\n", regs, cmd);
      svcinfo("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_R0],  regs[REG_R1],  regs[REG_R2],  regs[REG_R3],
              regs[REG_R4],  regs[REG_R5],  regs[REG_R6],  regs[REG_R7]);
      svcinfo("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
              regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
# ifdef REG_EXC_RETURN
      svcinfo(" PSR: %08x EXC_RETURN: %08x\n",
              regs[REG_XPSR], regs[REG_EXC_RETURN]);
# else
      svcinfo(" PSR: %08x\n", regs[REG_XPSR]);
# endif
    }
#endif

  /* Handle the SVCall according to the command in R0 */

  switch (cmd)
    {
      /* R0=SYS_save_context:  This is a save context command:
       *
       *   int up_saveusercontext(void *saveregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_save_context
       *   R1 = saveregs
       *
       * In this case, we simply need to copy the current registers to the
       * save register space references in the saved R1 and return.
       */

      case SYS_save_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0);
          memcpy((uint32_t *)regs[REG_R1], regs, XCPTCONTEXT_SIZE);
        }
        break;

      /* R0=SYS_restore_context:  This a restore context command:
       *
       *   void arm_fullcontextrestore(uint32_t *restoreregs)
       *          noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_restore_context
       *   R1 = restoreregs
       *
       * In this case, we simply need to set CURRENT_REGS to restore
       * register area referenced in the saved R1. context == CURRENT_REGS
       * is the normal exception return.  By setting CURRENT_REGS =
       * context[R1], we force the return to the saved context referenced
       * in R1.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0);
          CURRENT_REGS = (uint32_t *)regs[REG_R1];
        }
        break;

      /* R0=SYS_switch_context:  This a switch context command:
       *
       *   void arm_switchcontext(uint32_t **saveregs,
       *                          uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_switch_context
       *   R1 = saveregs
       *   R2 = restoreregs
       *
       * In this case, we do both: We save the context registers to the save
       * register area reference by the saved contents of R1 and then set
       * CURRENT_REGS to the save register area referenced by the saved
       * contents of R2.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0 && regs[REG_R2] != 0);
          *(uint32_t **)regs[REG_R1] = regs;
          CURRENT_REGS = (uint32_t *)regs[REG_R2];
        }
        break;

      /* R0=SYS_syscall_return:  This a syscall return command:
       *
       *   void arm_syscall_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_syscall_return
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

          regs[REG_PC]         = rtcb->xcp.syscall[index].sysreturn;
          regs[REG_EXC_RETURN] = rtcb->xcp.syscall[index].excreturn;
          rtcb->xcp.nsyscalls  = index;

          /* The return value must be in R0-R1.  dispatch_syscall()
           * temporarily moved the value for R0 into R2.
           */

          regs[REG_R0]         = regs[REG_R2];

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
       *   void up_task_start(main_t taskentry, int argc, char *argv[])
       *          noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_task_start
       *   R1 = taskentry
       *   R2 = argc
       *   R3 = argv
       */

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_task_start:
        {
          /* Set up to return to the user-space task start-up function in
           * unprivileged mode.
           */

          regs[REG_PC]         = (uint32_t)USERSPACE->task_startup & ~1;
          regs[REG_EXC_RETURN] = EXC_RETURN_UNPRIVTHR;

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s task_startup:
           */

          regs[REG_R0]         = regs[REG_R1]; /* Task entry */
          regs[REG_R1]         = regs[REG_R2]; /* argc */
          regs[REG_R2]         = regs[REG_R3]; /* argv */
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
       *   R1 = startup (trampoline)
       *   R2 = entrypt
       *   R3 = arg
       */

#if !defined(CONFIG_BUILD_FLAT) && !defined(CONFIG_DISABLE_PTHREAD)
      case SYS_pthread_start:
        {
          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode.
           */

          regs[REG_PC]         = (uint32_t)regs[REG_R1] & ~1;  /* startup */
          regs[REG_EXC_RETURN] = EXC_RETURN_UNPRIVTHR;

          /* Change the parameter ordering to match the expectation of the
           * user space pthread_startup:
           */

          regs[REG_R0]         = regs[REG_R2]; /* pthread entry */
          regs[REG_R1]         = regs[REG_R3]; /* arg */
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
          rtcb->xcp.sigreturn  = regs[REG_PC];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

          regs[REG_PC]         = (uint32_t)USERSPACE->signal_handler & ~1;
          regs[REG_EXC_RETURN] = EXC_RETURN_UNPRIVTHR;

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_R0]         = regs[REG_R1]; /* sighand */
          regs[REG_R1]         = regs[REG_R2]; /* signal */
          regs[REG_R2]         = regs[REG_R3]; /* info */
          regs[REG_R3]         = regs[REG_R4]; /* ucontext */
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

          regs[REG_PC]         = rtcb->xcp.sigreturn & ~1;
          regs[REG_EXC_RETURN] = EXC_RETURN_PRIVTHR;
          rtcb->xcp.sigreturn  = 0;
        }
        break;
#endif

      /* This is not an architecture-specific system call.  If NuttX is built
       * as a standalone kernel with a system call interface, then all of the
       * additional system calls must be handled as in the default case.
       */

      default:
        {
#ifdef CONFIG_LIB_SYSCALL
          struct tcb_s *rtcb = nxsched_self();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(cmd >= CONFIG_SYS_RESERVED && cmd < SYS_maxsyscall);

          /* Make sure that there is a no saved syscall return address.  We
           * cannot yet handle nested system calls.
           */

          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to dispatch_syscall in privileged mode. */

          rtcb->xcp.syscall[index].sysreturn  = regs[REG_PC];
          rtcb->xcp.syscall[index].excreturn  = regs[REG_EXC_RETURN];
          rtcb->xcp.nsyscalls  = index + 1;

          regs[REG_PC]         = (uint32_t)dispatch_syscall & ~1;
          regs[REG_EXC_RETURN] = EXC_RETURN_PRIVTHR;

          /* Offset R0 to account for the reserved values */

          regs[REG_R0]        -= CONFIG_SYS_RESERVED;

          /* Indicate that we are in a syscall handler. */

          rtcb->flags         |= TCB_FLAG_SYSCALL;
#else
          svcerr("ERROR: Bad SYS call: %d\n", (int)regs[REG_R0]);
#endif
        }
        break;
    }

  /* Report what happened.  That might difficult in the case of a context
   * switch.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
# ifndef CONFIG_DEBUG_SVCALL
  if (cmd > SYS_switch_context)
# else
  if (regs != CURRENT_REGS)
# endif
    {
      svcinfo("SVCall Return:\n");
      svcinfo("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              CURRENT_REGS[REG_R0],  CURRENT_REGS[REG_R1],
              CURRENT_REGS[REG_R2],  CURRENT_REGS[REG_R3],
              CURRENT_REGS[REG_R4],  CURRENT_REGS[REG_R5],
              CURRENT_REGS[REG_R6],  CURRENT_REGS[REG_R7]);
      svcinfo("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              CURRENT_REGS[REG_R8],  CURRENT_REGS[REG_R9],
              CURRENT_REGS[REG_R10], CURRENT_REGS[REG_R11],
              CURRENT_REGS[REG_R12], CURRENT_REGS[REG_R13],
              CURRENT_REGS[REG_R14], CURRENT_REGS[REG_R15]);
# ifdef REG_EXC_RETURN
      svcinfo(" PSR: %08x EXC_RETURN: %08x\n",
              CURRENT_REGS[REG_XPSR], CURRENT_REGS[REG_EXC_RETURN]);
# else
      svcinfo(" PSR: %08x\n", CURRENT_REGS[REG_XPSR]);
# endif
    }
# ifdef CONFIG_DEBUG_SVCALL
  else
    {
      svcinfo("SVCall Return: %d\n", regs[REG_R0]);
    }
# endif
#endif

  return OK;
}
