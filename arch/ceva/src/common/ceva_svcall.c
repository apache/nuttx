/****************************************************************************
 * arch/ceva/src/common/ceva_svcall.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>
#include <syscall.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/userspace.h>

#include "ceva_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ceva_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

int ceva_svcall(int irq, void *context, void *arg)
{
  uint32_t *regs = (uint32_t *)context;
  uint32_t cmd;

  DEBUGASSERT(regs && regs == CURRENT_REGS);
  cmd = regs[REG_A0];

  /* The SVCall software interrupt is called with A0 = system call command
   * and A1..A6 =  variable number of arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
# ifndef CONFIG_DEBUG_SVCALL
  if (cmd > SYS_switch_context)
# endif
    {
      svcinfo("SVCALL Entry: regs: %p cmd: %d\n", regs, cmd);
      svcinfo("A0: %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_A0], regs[REG_A1], regs[REG_A2], regs[REG_A3],
              regs[REG_A4], regs[REG_A5], regs[REG_A6]);
      svcinfo("FP: %08x LR: %08x PC: %08x IRQ: %08x OM: %08x\n",
              regs[REG_FP], regs[REG_LR], regs[REG_PC], regs[REG_IRQ],
# ifdef REG_OM
              regs[REG_OM]
#else
              0x00000000
#endif
             );
    }
#endif

  /* Handle the SVCall according to the command in A0 */

  switch (cmd)
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
       * In this case, we simply need to copy the current regsters to the
       * save register space references in the saved A1 and return.
       */

      case SYS_save_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0);
          memcpy((uint32_t *)regs[REG_A1], regs, XCPTCONTEXT_SIZE);
        }
        break;

      /* A0=SYS_restore_context:  This a restore context command:
       *
       *   void ceva_fullcontextrestore(uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_restore_context
       *   A1 = restoreregs
       *
       * In this case, we simply need to set CURRENT_REGS to restore register
       * area referenced in the saved A1. context == CURRENT_REGS is the
       * noraml exception return.  By setting CURRENT_REGS = context[A1],
       * we force the return to the saved context referenced in A1.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0);
          CURRENT_REGS = (uint32_t *)regs[REG_A1];
        }
        break;

      /* A0=SYS_switch_context:  This a switch context command:
       *
       *   void ceva_switchcontext(uint32_t **saveregs,
       *                           uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_switch_context
       *   A1 = saveregs
       *   A2 = restoreregs
       *
       * In this case, we do both: We save the context registers to the save
       * register area reference by the saved contents of A1 and then set
       * CURRENT_REGS to to the save register area referenced by the saved
       * contents of A2.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_A1] != 0 && regs[REG_A2] != 0);
          *(uint32_t **)regs[REG_A1] = regs;
          CURRENT_REGS = (uint32_t *)regs[REG_A2];
        }
        break;

      /* A0=SYS_syscall_return:  This a syscall return command:
       *
       *   void up_syscall_return(void);
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
          struct tcb_s *rtcb = sched_self();
          int index = rtcb->xcp.nsyscalls - 1;

          /* Make sure that there is a saved syscall return address. */

          DEBUGASSERT(index >= 0);

          /* Setup to return to the saved syscall return address in
           * the original mode.
           */

          regs[REG_PC] = rtcb->xcp.syscall[index].saved_pc;
#ifdef REG_OM
          regs[REG_OM] = rtcb->xcp.syscall[index].saved_om;
#endif
          rtcb->xcp.nsyscalls = index;

          /* The return value must be in A0-A1.  ceva_svcall_handler()
           * temporarily moved the value for A0 into A2.
           */

          regs[REG_A0] = regs[REG_A2];
        }
        break;
#endif

      /* A0=SYS_task_start:  This a user task start
       *
       *   void up_task_start(main_t taskentry, int argc, char *argv[]);
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

          regs[REG_PC]  = (uint32_t)USERSPACE->task_startup;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_USER;
#endif

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s task_startup:
           */

          regs[REG_A0]  = regs[REG_A1]; /* Task entry */
          regs[REG_A1]  = regs[REG_A2]; /* argc */
          regs[REG_A2]  = regs[REG_A3]; /* argv */
        }
        break;
#endif

      /* A0=SYS_pthread_start:  This a user pthread start
       *
       *   void up_pthread_start(pthread_startroutine_t entrypt,
                                 pthread_addr_t arg);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_pthread_start
       *   A1 = entrypt
       *   A2 = arg
       */

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_DISABLE_PTHREAD)
      case SYS_pthread_start:
        {
          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode.
           */

          regs[REG_PC]  = (uint32_t)USERSPACE->pthread_startup;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_USER;
#endif

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s pthread_startup:
           */

          regs[REG_A0]  = regs[REG_A1]; /* pthread entry */
          regs[REG_A1]  = regs[REG_A2]; /* arg */
        }
        break;
#endif

      /* A0=SYS_signal_handler:  This a user signal handler callback
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

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_DISABLE_SIGNALS)
      case SYS_signal_handler:
        {
          struct tcb_s *rtcb = sched_self();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn = regs[REG_PC];

          /* Set up to return to the user-space signal handler function in
           * unprivileged mode.
           */

          regs[REG_PC]  = (uint32_t)USERSPACE->signal_handler;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_USER;
#endif

          /* Change the parameter ordering to match the expectation of struct
           * userpace_s signal_handler.
           */

          regs[REG_A0]  = regs[REG_A1]; /* sighand */
          regs[REG_A1]  = regs[REG_A2]; /* signal */
          regs[REG_A2]  = regs[REG_A3]; /* info */
          regs[REG_A3]  = regs[REG_A4]; /* ucontext */
        }
        break;
#endif

      /* A0=SYS_signal_handler_return:  This a user signal handler callback
       *
       *   void signal_handler_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_signal_handler_return
       */

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_DISABLE_SIGNALS)
      case SYS_signal_handler_return:
        {
          struct tcb_s *rtcb = sched_self();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);

          regs[REG_PC]  = rtcb->xcp.sigreturn;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_KERNEL;
#endif
          rtcb->xcp.sigreturn = 0;
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
          struct tcb_s *rtcb = sched_self();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(cmd >= CONFIG_SYS_RESERVED && cmd < SYS_maxsyscall);
          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to ceva_svcall_handler in privileged mode. */

          rtcb->xcp.syscall[index].saved_pc = regs[REG_PC];
#ifdef REG_OM
          rtcb->xcp.syscall[index].saved_om = regs[REG_OM];
#endif
          rtcb->xcp.nsyscalls = index + 1;

          regs[REG_PC]  = (uint32_t)ceva_svcall_handler;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_KERNEL;
#endif

          /* Offset A0 to account for the reserved values */

          regs[REG_A0] -= CONFIG_SYS_RESERVED;
#else
          svcerr("ERROR: Bad SYS call: %d\n", regs[REG_A0]);
#endif
        }
        break;
    }

  /* Report what happened.
   * That might be different in the case of a context switch
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
# ifndef CONFIG_DEBUG_SVCALL
  if (cmd > SYS_switch_context)
# else
  if (regs != CURRENT_REGS)
# endif
    {
      svcinfo("SVCall Return:\n");
      svcinfo("A0: %08x %08x %08x %08x %08x %08x %08x\n",
              CURRENT_REGS[REG_A0], CURRENT_REGS[REG_A1],
              CURRENT_REGS[REG_A2], CURRENT_REGS[REG_A3],
              CURRENT_REGS[REG_A4], CURRENT_REGS[REG_A5],
              CURRENT_REGS[REG_A6]);
      svcinfo("FP: %08x LR: %08x PC: %08x IRQ: %08x OM: %08x\n",
              CURRENT_REGS[REG_FP], CURRENT_REGS[REG_LR],
              CURRENT_REGS[REG_PC], CURRENT_REGS[REG_IRQ],
# ifdef REG_OM
              CURRENT_REGS[REG_OM]
#else
              0x00000000
#endif
              );
    }
# ifdef CONFIG_DEBUG_SVCALL
  else
    {
      svcinfo("SVCall Return: %d\n", regs[REG_A0]);
    }
# endif
#endif

  return OK;
}
