/****************************************************************************
 * arch/sparc/src/sparc_v8/up_swint1.c
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
#include <syscall.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/sched.h>

#include <arch/irq.h>

#include "up_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SYSCALL_INFO
static void up_registerdump(const uint32_t *regs)
{
}
#else
#  define up_registerdump(regs)
#endif

/****************************************************************************
 * Name: dispatch_syscall
 *
 * Description:
 *   Call the stub function corresponding to the system call.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
static void dispatch_syscall(void) naked_function;
static void dispatch_syscall(void)
{
#  error "Missing logic"
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_swint1
 *
 * Description:
 *   This is software interrupt 8 exception handler that performs context
 *   switching and manages system calls
 *
 ****************************************************************************/

int up_swint1(int irq, FAR void *context, FAR void *arg)
{
  uint32_t *regs = (uint32_t *)context;

  DEBUGASSERT(regs && regs == g_current_regs);

  /* Software interrupt 0 is invoked with REG_A0 (REG_R4) = system call
   * command and REG_A1-3 and REG_T0-2 (REG_R5-10) = variable number of
   * arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  svcinfo("Entry: regs: %p cmd: %d\n", regs, regs[REG_I0]);
  up_registerdump(regs);
#endif

  /* Handle the SWInt according to the command in $4 */

  switch (regs[REG_I0])
    {
      /* A0=SYS_restore_context: This a restore context command:
       *
       * void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_restore_context
       *   A1 = restoreregs
       *
       * In this case, we simply need to set g_current_regs to restore
       * register area referenced in the saved R1. context == g_current
       * regs is the normal exception return.  By setting g_current
       * regs = context[R1], we force the return to the saved context
       * referenced in $a1.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_I1] != 0);
          g_current_regs = (uint32_t *)regs[REG_I1];
        }
        break;

      /* A0=SYS_switch_context: This a switch context command:
       *
       *   void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A0 = SYS_switch_context
       *   A1 = saveregs
       *   A2 = restoreregs
       *
       * In this case, we save the context registers to the save register
       * area referenced by the saved contents of R5 and then set
       * g_current_regs to the save register area referenced by the saved
       * contents of R6.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_I1] != 0 && regs[REG_I2] != 0);
          trap_flush_task((uint32_t *)regs[REG_I1], regs);

          /* task_flush_trap(regs,(uint32_t *)regs[REG_I2]); */

          g_current_regs = (uint32_t *)regs[REG_I2];
        }
        break;

      /* R0=SYS_syscall_return: This a switch context command:
       *
       *   void up_sycall_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_syscall_return
       *
       * We need to restore the saved return address and return in
       * unprivileged thread mode.
       */

#ifdef CONFIG_BUILD_KERNEL
      case SYS_syscall_return:
        {
          struct tcb_s *rtcb = sched_self();
          int index = (int)rtcb->xcp.nsyscalls - 1;

          /* Make sure that there is a saved syscall return address. */

          DEBUGASSERT(index >= 0);

          /* Setup to return to the saved syscall return address in
           * the original mode.
           */

          g_current_regs[REG_I7] = rtcb->xcp.syscall[index].sysreturn;
#error "Missing logic -- need to restore the original mode"
          rtcb->xcp.nsyscalls   = index;
        }
        break;
#endif

      default:
        {
#ifdef CONFIG_BUILD_KERNEL
          FAR struct tcb_s *rtcb = sched_self();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(g_current_regs[REG_I1] < SYS_maxsyscall);

          /* Make sure that we got here that there is a no saved syscall
           * return address.  We cannot yet handle nested system calls.
           */

          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to dispatch_syscall in privileged mode. */

          rtcb->xcpsyscall[index].sysreturn = regs[REG_I7];
#error "Missing logic -- Need to save mode"
          rtcb->xcp.nsyscalls  = index + 1;

          regs[REG_I7] = (uint32_t)dispatch_syscall;
#error "Missing logic -- Need to set privileged mode"

          /* Offset R0 to account for the reserved values */

          /* g_current_regs[REG_R0] -= CONFIG_SYS_RESERVED; *//*zouboan*/
#else
          svcerr("ERROR: Bad SYS call: %d\n", regs[REG_I1]);
#endif
        }
        break;
    }

  /* Report what happened. That might difficult in the case of a context
   * switch
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  if (regs != g_current_regs)
    {
      svcinfo("SWInt Return: Context switch!\n");
      up_registerdump((const uint32_t *)g_current_regs);
    }
  else
    {
      svcinfo("SWInt Return: %d\n", regs[REG_I0]);
    }
#endif

  return OK;
}
