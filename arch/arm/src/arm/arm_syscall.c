/****************************************************************************
 * arch/arm/src/arm/arm_syscall.c
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
#include <assert.h>
#include <debug.h>
#include <syscall.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_syscall
 *
 * Description:
 *   SWI interrupts will vector here with insn=the SWI instruction and
 *   xcp=the interrupt context
 *
 *   The handler may get the SWI number be de-referencing
 *   the return address saved in the xcp and decoding
 *   the SWI instruction
 *
 ****************************************************************************/

uint32_t *arm_syscall(uint32_t *regs)
{
  uint32_t cmd;

  /* Nested interrupts are not supported */

  DEBUGASSERT(CURRENT_REGS == NULL);

  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   */

  CURRENT_REGS = regs;

  /* The SYSCALL command is in R0 on entry.  Parameters follow in R1..R7 */

  cmd = regs[REG_R0];

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

      /* R0=SYS_restore_context:  Restore task context
       *
       * void arm_fullcontextrestore(uint32_t *restoreregs)
       *   noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_restore_context
       *   R1 = restoreregs
       */

      case SYS_restore_context:
        {
          /* Replace 'regs' with the pointer to the register set in
           * regs[REG_R1].  On return from the system call, that register
           * set will determine the restored context.
           */

          CURRENT_REGS = (uint32_t *)regs[REG_R1];
          DEBUGASSERT(CURRENT_REGS);
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
       * regs to the save register area referenced by the saved
       * contents of R2.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0 && regs[REG_R2] != 0);
          *(uint32_t **)regs[REG_R1] = regs;
          CURRENT_REGS = (uint32_t *)regs[REG_R2];
        }
        break;

      default:
        {
          svcerr("ERROR: Bad SYS call: 0x%" PRIx32 "\n", regs[REG_R0]);
          _alert("Syscall from 0x%" PRIx32 "\n", regs[REG_PC]);
          PANIC();
        }
        break;
    }

#ifdef CONFIG_ARCH_ADDRENV
  /* Check for a context switch.  If a context switch occurred, then
   * CURRENT_REGS will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then establish the correct
   * address environment before returning from the interrupt.
   */

  if (regs != CURRENT_REGS)
    {
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      addrenv_switch(NULL);
    }
#endif

  /* Restore the cpu lock */

  if (regs != CURRENT_REGS)
    {
      /* Record the new "running" task.  g_running_tasks[] is only used by
       * assertion logic for reporting crashes.
       */

      g_running_tasks[this_cpu()] = this_task();

      restore_critical_section();
      regs = (uint32_t *)CURRENT_REGS;
    }

  /* Set CURRENT_REGS to NULL to indicate that we are no longer in an
   * interrupt handler.
   */

  CURRENT_REGS = NULL;

  /* Return the last value of curent_regs.  This supports context switches
   * on return from the exception.  That capability is only used with the
   * SYS_context_switch system call.
   */

  return regs;
}
