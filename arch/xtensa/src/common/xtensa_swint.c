/****************************************************************************
 * arch/xtensa/src/common/xtensa_swint.c
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

#include <assert.h>
#include <nuttx/debug.h>
#include <stdint.h>

#include <arch/xtensa/xtensa_specregs.h>
#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <sys/syscall.h>

#include "sched/sched.h"
#include "chip.h"
#include "signal/signal.h"
#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_KERNEL_STACK
/* A stack pointer is 16-byte aligned, and the windowed ABI reserves the
 * 16 bytes below one as the base save area of the frame that owns it.
 */

#  define SIGTRAMP_STACK_ALIGN  16
#  define SIGTRAMP_SAVE_AREA    16
#endif

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

      case SYS_restore_context:
      case SYS_switch_context:
        {
#ifdef CONFIG_ARCH_ADDRENV
          /* Close down the outgoing task's address environment and
           * instantiate the incoming one.  up_switch_context() is a
           * SYS_switch_context call on this architecture, so this is the
           * path every *voluntary* context switch takes -- without it a task
           * resumed here keeps running against whatever address environment
           * happened to be resident, which on the ESP32-S3 means the
           * cache-MMU windows still point at another process's pages.
           *
           * addrenv_switch() may change this_task(), because dropping an
           * address environment can post to the high-priority work queue, so
           * re-read the TCB afterwards -- as arm_syscall.c does.
           */

          addrenv_switch(tcb);
          tcb = this_task();
#endif

          restore_critical_section(tcb, this_cpu());
#ifdef CONFIG_DEBUG_SYSCALL_INFO
          svcinfo("SYSCALL Return: Context switch!\n");
          up_dump_register(tcb->xcp.regs);
#endif
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

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* Leaving the outermost system call: hand the thread back its own
           * stack, which it has not touched while the kernel borrowed its
           * context.
           */

          if (index == 0 && rtcb->xcp.ustkptr != NULL)
            {
              regs[REG_A1]      = (uintptr_t)rtcb->xcp.ustkptr;
              rtcb->xcp.ustkptr = NULL;
            }
#endif

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

#if !defined(CONFIG_BUILD_FLAT) && defined(CONFIG_ENABLE_ALL_SIGNALS)
      case SYS_signal_handler:
        {
          struct tcb_s *rtcb  = this_task();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn = regs[REG_PC];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

#if defined(CONFIG_BUILD_PROTECTED)
          regs[REG_PC]        = (uintptr_t)USERSPACE->signal_handler;
#else
          regs[REG_PC]        = (uintptr_t)ARCH_DATA_RESERVE->ar_sigtramp;
#endif

          xtensa_lowerprivilege(regs);        /* User mode */

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_A2]        = regs[REG_A3]; /* sighand */
          regs[REG_A3]        = regs[REG_A4]; /* signal */
          regs[REG_A4]        = regs[REG_A5]; /* info */
          regs[REG_A5]        = regs[REG_A6]; /* ucontext */

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* The handler runs in user mode, so it has to run on the user
           * stack.  Signal dispatch always reaches here on the thread's
           * kernel stack -- up_schedule_sigaction() builds the dispatch
           * context below the interrupted one -- so put that stack pointer
           * aside and hand the thread its own stack back for the duration.
           *
           * Having a kernel stack at all is what says this is a user
           * process.  Testing xcp.ustkptr instead would be wrong: that
           * holds the user stack pointer only while a system call is in
           * progress, so a signal caught in user code would leave the
           * handler running on the kernel stack.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              uintptr_t usp;

              rtcb->xcp.kstkptr = (uint32_t *)regs[REG_A1];

              /* The thread's own stack pointer is the one the system call
               * saved if it was in one, and otherwise the one it was
               * interrupted with, which up_schedule_sigaction() kept.
               */

              usp = rtcb->xcp.ustkptr != NULL ?
                    (uintptr_t)rtcb->xcp.ustkptr :
                    (uintptr_t)rtcb->xcp.saved_regs[REG_A1];

              /* The siginfo passed in lives on the kernel stack, which the
               * handler must not reach -- and cannot, once the permission
               * control is programmed.  Copy it onto the user stack and
               * hand the handler that copy.
               *
               * Skip the base save area the windowed ABI keeps in the
               * 16 bytes below a stack pointer: it belongs to the frame
               * that was interrupted.
               */

              usp = (usp - SIGTRAMP_SAVE_AREA - sizeof(siginfo_t)) &
                    ~(SIGTRAMP_STACK_ALIGN - 1);

              memcpy((void *)usp, (void *)regs[REG_A4], sizeof(siginfo_t));

              regs[REG_A4]      = usp;            /* info */
              regs[REG_A1]      = usp;
            }
#endif
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

#if !defined(CONFIG_BUILD_FLAT) && defined(CONFIG_ENABLE_ALL_SIGNALS)
      case SYS_signal_handler_return:
        {
          struct tcb_s *rtcb  = this_task();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);
          regs[REG_PC] = rtcb->xcp.sigreturn;

          xtensa_raiseprivilege(regs);        /* Privileged mode */

          rtcb->xcp.sigreturn = 0;

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* The handler is done: return to the kernel stack the signal
           * dispatch was running on.
           */

          if (rtcb->xcp.kstack != NULL)
            {
              DEBUGASSERT(rtcb->xcp.kstkptr != NULL);

              regs[REG_A1]      = (uintptr_t)rtcb->xcp.kstkptr;
              rtcb->xcp.kstkptr = NULL;
            }
#endif
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

          /* Remember where the caller's registers are.  The cloning
           * primitives need them:  a system call body runs as C code after
           * this exception has returned, and xcp.regs is NULL by then.  Only
           * the outermost call is of interest, since that is the one the
           * caller made.  See up_fork().
           */

          if (index == 0)
            {
              rtcb->xcp.sregs = regs;
            }

          rtcb->xcp.nsyscalls = index + 1;

          regs[REG_PC]        = (uintptr_t)xtensa_dispatch_syscall;

#ifndef CONFIG_BUILD_FLAT
          xtensa_raiseprivilege(regs);        /* Privileged mode */
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
          /* The system call itself runs in this task's own context, so
           * without help it would run the kernel on the *user* stack.  That
           * cannot be allowed in a kernel build: the user stack lives in a
           * cache-MMU window, and any system call that selects a different
           * address environment -- exec() loading a program, for one --
           * reprograms that window and the kernel's stack disappears from
           * under it, taking the frames it is standing on.
           *
           * So the outermost system call moves to the thread's kernel
           * stack, which lives in kernel memory and is unaffected by
           * address environment changes.  Nested calls are already on it.
           */

          if (index == 0 && rtcb->xcp.ktopstk != NULL)
            {
              rtcb->xcp.ustkptr = (uint32_t *)regs[REG_A1];

              /* Start at the top of the kernel stack -- unless a signal
               * handler is running, in which case the kernel stack is in
               * use down to the point the dispatch left it at, and this
               * call has to continue below that.  Restarting at the top
               * would overwrite both the suspended signal dispatch and the
               * context it saved to resume the thread with, which sits in
               * the topmost frame.
               */

              regs[REG_A1]      = rtcb->xcp.kstkptr != NULL ?
                                  (uintptr_t)rtcb->xcp.kstkptr :
                                  (uintptr_t)rtcb->xcp.ktopstk;
            }
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

  return OK;
}
