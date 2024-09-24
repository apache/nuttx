/****************************************************************************
 * arch/sim/src/sim/sim_doirq.c
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

#include <stdbool.h>
#include <nuttx/arch.h>
#include <sched/sched.h>
#include <nuttx/init.h>

#include "sim_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void  sim_unlock(void)
{
  /* wait until cpu0 in idle() */

  while (!OSINIT_IDLELOOP());

  sched_unlock();
}

/****************************************************************************
 * Name: sim_doirq
 ****************************************************************************/

void *sim_doirq(int irq, void *context)
{
  /* Allocate temporary context on the stack */

  xcpt_reg_t tmp[XCPTCONTEXT_REGS];
  void *regs = (void *)tmp;
  int ret;

  /* current_regs non-zero indicates that we are processing an interrupt.
   * current_regs is also used to manage interrupt level context switches.
   */

  sim_saveusercontext(regs, ret);
  if (ret == 0)
    {
      struct tcb_s **running_task = &g_running_tasks[this_cpu()];

      if (*running_task != NULL)
        {
          sim_copyfullstate((*running_task)->xcp.regs, regs);
        }

      up_set_current_regs(regs);

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* If a context switch occurred while processing the interrupt then
       * current_regs may have change value.  If we return any value
       * different from the input regs, then the lower level will know that
       * context switch occurred during interrupt processing.
       */

      if (regs != up_current_regs())
        {
          /* Record the new "running" task when context switch occurred.
           * g_running_tasks[] is only used by assertion logic for reporting
           * crashes.
           */

          g_running_tasks[this_cpu()] = this_task();
        }

      regs = up_current_regs();

      /* Restore the previous value of current_regs.  NULL would indicate
       * that we are no longer in an interrupt handler.  It will be non-NULL
       * if we are returning from a nested interrupt.
       */

      up_set_current_regs(NULL);

      /* Then switch contexts */

      sim_fullcontextrestore(regs);
    }

  return NULL;
}
