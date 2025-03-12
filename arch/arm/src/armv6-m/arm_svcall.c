/****************************************************************************
 * arch/arm/src/armv6-m/arm_svcall.c
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

#include <arch/irq.h>
#include <nuttx/macro.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "signal/signal.h"
#include "exc_return.h"
#include "arm_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  struct tcb_s *tcb;
  uint32_t cmd;

  cmd = regs[REG_R0];

  /* The SVCall software interrupt is called with R0 = system call command
   * and R1..R7 =  variable number of arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
#  ifndef CONFIG_DEBUG_SVCALL
  if (cmd > SYS_switch_context)
#  endif
    {
      svcinfo("SVCALL Entry: regs: %p cmd: %d\n", regs, cmd);
      svcinfo("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_R0],  regs[REG_R1],  regs[REG_R2],  regs[REG_R3],
              regs[REG_R4],  regs[REG_R5],  regs[REG_R6],  regs[REG_R7]);
      svcinfo("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
              regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
      svcinfo(" PSR: %08x PRIMASK: %08x EXC_RETURN: %08x\n",
              regs[REG_XPSR], regs[REG_PRIMASK], regs[REG_EXC_RETURN]);
    }
#endif

  /* Handle the SVCall according to the command in R0 */

  switch (cmd)
    {
      case SYS_restore_context:
      case SYS_switch_context:
        {
          tcb = this_task();
          restore_critical_section(tcb, this_cpu());

#ifdef CONFIG_DEBUG_SYSCALL_INFO
          regs = tcb->xcp.regs;
#endif
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
          struct tcb_s *rtcb = this_task();
          int index = (int)rtcb->xcp.nsyscalls - 1;

          /* Make sure that there is a saved syscall return address. */

          DEBUGASSERT(index >= 0);

          /* Setup to return to the saved syscall return address in
           * the original mode.
           */

          regs[REG_PC]         = rtcb->xcp.syscall[index].sysreturn;
          regs[REG_EXC_RETURN] = rtcb->xcp.syscall[index].excreturn;
          regs[REG_CONTROL]    = rtcb->xcp.syscall[index].ctrlreturn;
          rtcb->xcp.nsyscalls  = index;

          /* The return value must be in R0-R1.  arm_dispatch_syscall()
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
       *     noreturn_function;
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

          regs[REG_PC]         = (uint32_t)USERSPACE->task_startup;
          regs[REG_EXC_RETURN] = EXC_RETURN_THREAD;

          /* Return unprivileged mode */

          regs[REG_CONTROL]    = getcontrol() | CONTROL_NPRIV;

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
       *   R1 = entrypt
       *   R2 = arg
       */

#if !defined(CONFIG_BUILD_FLAT) && !defined(CONFIG_DISABLE_PTHREAD)
      case SYS_pthread_start:
        {
          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode.
           */

          regs[REG_PC]         = (uint32_t)regs[REG_R1]; /* startup */
          regs[REG_EXC_RETURN] = EXC_RETURN_THREAD;

          /* Return unprivileged mode */

          regs[REG_CONTROL]    = getcontrol() | CONTROL_NPRIV;

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
          struct tcb_s *rtcb   = this_task();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn  = regs[REG_PC];

          /* Set up to return to the user-space trampoline function in
           * unprivileged mode.
           */

          regs[REG_PC]         = (uint32_t)USERSPACE->signal_handler;
          regs[REG_EXC_RETURN] = EXC_RETURN_THREAD;

          /* Return unprivileged mode */

          regs[REG_CONTROL]    = getcontrol() | CONTROL_NPRIV;

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
          struct tcb_s *rtcb   = this_task();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);

          regs[REG_PC]         = rtcb->xcp.sigreturn;
          regs[REG_EXC_RETURN] = EXC_RETURN_THREAD;

          /* Return privileged mode */

          regs[REG_CONTROL]    = getcontrol() & ~CONTROL_NPRIV;
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
          struct tcb_s *rtcb = this_task();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(cmd >= CONFIG_SYS_RESERVED && cmd < SYS_maxsyscall);

          /* Make sure that there is a no saved syscall return address.  We
           * cannot yet handle nested system calls.
           */

          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Use ip to create a debug frame.
           * we can use gdb backtrace from syscall to user space.
           */

          regs[REG_IP] = regs[REG_PC];

          /* Setup to return to arm_dispatch_syscall in privileged mode. */

          rtcb->xcp.syscall[index].sysreturn  = regs[REG_PC];
          rtcb->xcp.syscall[index].excreturn  = regs[REG_EXC_RETURN];
          rtcb->xcp.syscall[index].ctrlreturn = regs[REG_CONTROL];
          rtcb->xcp.nsyscalls  = index + 1;

          regs[REG_PC]         = (uint32_t)arm_dispatch_syscall;
          regs[REG_EXC_RETURN] = EXC_RETURN_THREAD;

          /* Return privileged mode */

          regs[REG_CONTROL]    = getcontrol() & ~CONTROL_NPRIV;

          /* Offset R0 to account for the reserved values */

          regs[REG_R0]        -= CONFIG_SYS_RESERVED;

          /* Indicate that we are in a syscall handler. */

          rtcb->flags         |= TCB_FLAG_SYSCALL;
#else
          svcerr("ERROR: Bad SYS call: %" PRId32 "\n", regs[REG_R0]);
#endif
        }
        break;
    }

  /* Report what happened.  That might difficult in the case of a context
   * switch.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
#  ifndef CONFIG_DEBUG_SVCALL
  if (cmd > SYS_switch_context)
#  endif
    {
      svcinfo("SVCall Return:\n");
      svcinfo("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_R0],  regs[REG_R1], regs[REG_R2],  regs[REG_R3],
              regs[REG_R4],  regs[REG_R5], regs[REG_R6],  regs[REG_R7]);
      svcinfo("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_R8],  regs[REG_R9], regs[REG_R10], regs[REG_R11],
              regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
      svcinfo(" PSR: %08x EXC_RETURN: %08x CONTROL: %08x\n",
              regs[REG_XPSR], regs[REG_EXC_RETURN], regs[REG_CONTROL]);
    }
#endif

  UNUSED(tcb);
  return OK;
}
