/****************************************************************************
 * arch/ceva/src/common/ceva_doirq.c
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

#include <nuttx/arch.h>

#include "sched/sched.h"
#include "ceva_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the
 * [get/set]_current_regs for portability.
 */

uint32_t *volatile g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *ceva_doirq(int irq, uint32_t *regs)
{
  /* Is it the outermost interrupt? */

  if (up_current_regs() != NULL)
    {
      /* No, simply deliver the IRQ because only the outermost nested
       * interrupt can result in a context switch.
       */

      irq_dispatch(irq, regs);
    }
  else
    {
      struct tcb_s **running_task = &g_running_tasks[this_cpu()];

      if (*running_task != NULL)
        {
          (*running_task)->xcp.regs = regs;
        }

      /* Current regs non-zero indicates that we are processing an interrupt;
       * current_regs is also used to manage interrupt level context
       * switches.
       */

      up_set_current_regs(regs);

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* If a context switch occurred while processing the interrupt then
       * current_regs may have change value.  If we return any value
       * different from the input regs, then the lower level will know that
       * a context switch occurred during interrupt processing.
       */

      if (regs != up_current_regs())
        {
          /* Update scheduler parameters */

          nxsched_suspend_scheduler(*running_task);
          nxsched_resume_scheduler(this_task());

          /* Record the new "running" task when context switch occurred.
           * g_running_tasks[] is only used by assertion logic for reporting
           * crashes.
           */

          g_running_tasks[this_cpu()] = this_task();
          regs = up_current_regs();
        }

      /* Restore the previous value of current_regs.  NULL would indicate
       * that we are no longer in an interrupt handler.
       * It will be non-NULL if we are returning from a nested interrupt.
       */

      up_set_current_regs(NULL);

      if (regs != (uint32_t *)regs[REG_SP])
        {
          /* We are returning with a pending context switch. This case is
           * different because in this case, the register save structure
           * does not lie on the stack but, rather within other storage.
           * We'll have to copy some values to the stack.
           */

          memcpy((uint32_t *)regs[REG_SP], regs, XCPTCONTEXT_SIZE);
          regs = (uint32_t *)regs[REG_SP];
        }
    }

  return regs;
}
