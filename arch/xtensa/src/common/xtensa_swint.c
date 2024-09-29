/****************************************************************************
 * arch/xtensa/src/common/xtensa_swint.c
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

#include <assert.h>
#include <debug.h>
#include <stdint.h>

#include <arch/xtensa/xtensa_specregs.h>
#include <nuttx/arch.h>
#include <sys/syscall.h>

#include "sched/sched.h"
#include "chip.h"
#include "signal/signal.h"
#include "sched/sched.h"
#include "xtensa.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_swint
 *
 * Description:
 *   This is software interrupt exception handler that performs context
 *   switching and manages system calls
 *
 ****************************************************************************/

int xtensa_swint(int irq, void *context, void *arg)
{
  uint32_t *regs = (uint32_t *)context;
  struct tcb_s *tcb = this_task();
  uint32_t cmd;

  DEBUGASSERT(regs != NULL);

  cmd = regs[REG_A2];
  tcb->xcp.regs = regs;

  /* The syscall software interrupt is called with A2 = system call command
   * and A3..A9 = variable number of arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  svcinfo("SYSCALL Entry: regs: %p cmd: %" PRIu32 "\n", regs, cmd);
  up_dump_register(regs);
#endif

  /* Handle the syscall according to the command in A2 */

  switch (cmd)
    {
      /* A2=SYS_save_context:  This is a save context command:
       *
       * int up_saveusercontext(uint32_t *saveregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_save_context
       *   A3 = saveregs
       *
       * In this case, we simply need to copy the current registers to the
       * save register space references in the saved A3 and return.
       */

      case SYS_save_context:
        {
          DEBUGASSERT(regs[REG_A3] != 0);
          memcpy((uint32_t *)regs[REG_A3], regs, XCPTCONTEXT_SIZE);
        }
        break;

      /* A2=SYS_restore_context:  This is a restore context command:
       *
       * void xtensa_fullcontextrestore(uint32_t *restoreregs)
       *      noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_restore_context
       *   A3 = restoreregs
       *
       * In this case, we simply need to set current_regs to restore
       * register area referenced in the saved A3. context == current_regs
       * is the normal exception return.  By setting current_regs =
       * context[A3], we force the return to the saved context referenced
       * in A3.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_A3] != 0);
          tcb->xcp.regs = (uint32_t *)regs[REG_A3];
        }
        break;

      /* A2=SYS_switch_context:  This is a switch context command:
       *
       * void xtensa_switchcontext
       *      (uint32_t *saveregs, uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_switch_context
       *   A3 = saveregs
       *   A4 = restoreregs
       *
       * In this case, we do both: We save the context registers to the save
       * register area reference by the saved contents of A3 and then set
       * current_regs to the save register area referenced by the saved
       * contents of A4.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_A3] != 0 && regs[REG_A4] != 0);
          *(uint32_t **)regs[REG_A3] = regs;
          tcb->xcp.regs = (uint32_t *)regs[REG_A4];
        }
        break;

      /* A2=SYS_syscall_return: This is a syscall return command:
       *
       *   void xtensa_syscall_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_syscall_return
       *
       * We need to restore the saved return address and return in
       * unprivileged thread mode.
       */

#ifdef CONFIG_LIB_SYSCALL
      case SYS_syscall_return:
        {
          struct tcb_s *rtcb = this_task();
          int index = (int)rtcb->xcp.nsyscalls - 1;

          /* Make sure that there is a saved syscall return address. */

          DEBUGASSERT(index >= 0);

          /* Setup to return to the saved syscall return address in
           * the original mode.
           */

          regs[REG_PC]        = rtcb->xcp.syscall[index].sysreturn;
#ifndef CONFIG_BUILD_FLAT
          xtensa_restoreprivilege(regs, rtcb->xcp.syscall[index].int_ctx);
#endif

          /* The return value must be in A2-A5.
           * xtensa_dispatch_syscall() temporarily moved the value into A3.
           */

          regs[REG_A2]        = regs[REG_A3];

          /* Save the new syscall nesting level */

          rtcb->xcp.nsyscalls = index;

          /* Handle any signal actions that were deferred while processing
           * the system call.
           */

          rtcb->flags         &= ~TCB_FLAG_SYSCALL;
          nxsig_unmask_pendingsignal();
        }
        break;
#endif

      /* A2=SYS_task_start:  This a user task start
       *
       *   void up_task_start(main_t taskentry, int argc,
       *                      char *argv[]) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_task_start
       *   A3 = taskentry
       *   A4 = argc
       *   A5 = argv
       */

#ifndef CONFIG_BUILD_FLAT
      case SYS_task_start:
        {
          /* Set up to return to the user-space task start-up function in
           * unprivileged mode.
           */

#ifdef CONFIG_BUILD_PROTECTED
          /* Use the nxtask_startup trampoline function */

          regs[REG_PC] = (uintptr_t)USERSPACE->task_startup;
          regs[REG_A6] = regs[REG_A3]; /* Task entry */
          regs[REG_A7] = regs[REG_A4]; /* argc */
          regs[REG_A8] = regs[REG_A5]; /* argv */
#else
          /* Start the user task directly */

          regs[REG_PC] = (uintptr_t)regs[REG_A3];
          regs[REG_A6] = regs[REG_A4]; /* argc */
          regs[REG_A7] = regs[REG_A5]; /* argv */
#endif

          /* Execute the task in User mode */

          xtensa_lowerprivilege(regs);        /* User mode */

          /* User task rotates window, so pretend task was 'call4'd */

          regs[REG_PS] = PS_UM | PS_WOE | PS_CALLINC(1);
        }
        break;
#endif

      /* A2=SYS_pthread_start:  This a user pthread start
       *
       *   void up_pthread_start(pthread_startroutine_t entrypt,
       *                         pthread_addr_t arg) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_pthread_start
       *   A3 = startup
       *   A4 = entrypt
       *   A5 = arg
       */

#if !defined(CONFIG_BUILD_FLAT) && !defined(CONFIG_DISABLE_PTHREAD)
      case SYS_pthread_start:
        {
          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode.
           */

          regs[REG_PC] = (uintptr_t)regs[REG_A3];  /* startup */

          /* Change the parameter ordering to match the expectation of the
           * user space pthread_startup:
           */

          regs[REG_A6] = regs[REG_A4];  /* pthread entry */
          regs[REG_A7] = regs[REG_A5];  /* arg */

          /* Execute the pthread in User mode */

          xtensa_lowerprivilege(regs);        /* User mode */

          /* Startup task rotates window, so pretend task was 'call4'd */

          regs[REG_PS] = PS_UM | PS_WOE | PS_CALLINC(1);
        }
        break;
#endif

      /* A2=SYS_signal_handler:  This a user signal handler callback
       *
       * void signal_handler(_sa_sigaction_t sighand, int signo,
       *                     siginfo_t *info, void *ucontext);
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_signal_handler
       *   A3 = sighand
       *   A4 = signo
       *   A5 = info
       *   A6 = ucontext
       */

#ifndef CONFIG_BUILD_FLAT
      case SYS_signal_handler:
        {
          struct tcb_s *rtcb  = this_task();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn = regs[REG_PC];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

          regs[REG_PC]        = (uintptr_t)USERSPACE->signal_handler;

          xtensa_lowerprivilege(regs);        /* User mode */

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_A2]        = regs[REG_A3]; /* sighand */
          regs[REG_A3]        = regs[REG_A4]; /* signal */
          regs[REG_A4]        = regs[REG_A5]; /* info */
          regs[REG_A5]        = regs[REG_A6]; /* ucontext */
        }
        break;
#endif

      /* A2=SYS_signal_handler_return:  This a user signal handler callback
       *
       *   void signal_handler_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_signal_handler_return
       */

#ifndef CONFIG_BUILD_FLAT
      case SYS_signal_handler_return:
        {
          struct tcb_s *rtcb  = this_task();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);
          regs[REG_PC] = rtcb->xcp.sigreturn;

          xtensa_raiseprivilege(regs);        /* Privileged mode */

          rtcb->xcp.sigreturn = 0;
        }
        break;
#endif

      /* This is not an architecture-specific system call. If NuttX is built
       * as a standalone kernel with a system call interface, then all of the
       * additional system calls must be handled as in the default case.
       */

      default:
        {
#ifdef CONFIG_LIB_SYSCALL
          struct tcb_s *rtcb = this_task();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the syscall number is within range */

          DEBUGASSERT(cmd < SYS_maxsyscall);

          /* Make sure that we got here that there is a no saved syscall
           * return address.  We cannot yet handle nested system calls.
           */

          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to xtensa_dispatch_syscall in privileged mode. */

          rtcb->xcp.syscall[index].sysreturn = regs[REG_PC];
#ifndef CONFIG_BUILD_FLAT
          xtensa_saveprivilege(regs, rtcb->xcp.syscall[index].int_ctx);
#endif

          rtcb->xcp.nsyscalls = index + 1;

          regs[REG_PC]        = (uintptr_t)xtensa_dispatch_syscall;

#ifndef CONFIG_BUILD_FLAT
          xtensa_raiseprivilege(regs);        /* Privileged mode */
#endif

          /* Offset A2 to account for the reserved values */

          regs[REG_A2]        -= CONFIG_SYS_RESERVED;

          /* Indicate that we are in a syscall handler. */

          rtcb->flags         |= TCB_FLAG_SYSCALL;
#else
          svcerr("ERROR: Bad SYSCALL: %" PRIu32 "\n", cmd);
#endif
        }
        break;

      /* A2=SYS_flush_context:  This flush windows to the stack:
       *
       * int xtensa_flushcontext(void);
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_flush_context
       *
       * In this case, we simply need to do nothing.
       * As flush the register windows to the stack has be done by
       * interrupt enter handler.
       */

      case SYS_flush_context:

        break;
    }

  if ((tcb->xcp.regs[REG_PS] & PS_EXCM_MASK) != 0)
    {
      tcb->xcp.regs[REG_PS] &= ~PS_EXCM_MASK;
    }

  /* Report what happened.  That might difficult in the case of a context
   * switch.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  if (regs != tcb->xcp.regs)
    {
      svcinfo("SYSCALL Return: Context switch!\n");
      up_dump_register(tcb->xcp.regs);
    }
  else
    {
      svcinfo("SYSCALL Return: %" PRIu32 "\n", cmd);
    }
#endif

  if (regs != tcb->xcp.regs)
    {
      restore_critical_section(this_task(), this_cpu());
    }

  return OK;
}
