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

#include "sim_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_doirq
 ****************************************************************************/

void *sim_doirq(int irq, void *context)
{
  /* Allocate temporary context on the stack */

  xcpt_reg_t tmp[XCPTCONTEXT_REGS];
  void *regs = (void *)tmp;

  /* CURRENT_REGS non-zero indicates that we are processing an interrupt.
   * CURRENT_REGS is also used to manage interrupt level context switches.
   */

#ifdef CONFIG_SMP
  if (setjmp(regs) == 0)
    {
#endif

      CURRENT_REGS = regs;

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      if (regs != CURRENT_REGS)
        {
          /* Restore the cpu lock */

          restore_critical_section();

          /* If a context switch occurred while processing the interrupt then
           * CURRENT_REGS may have change value.  If we return any value
           * different from the input regs, then the lower level will know
           * that context switch occurred during interrupt processing.
           */

          regs = (void *)CURRENT_REGS;
        }

      /* Restore the previous value of CURRENT_REGS.  NULL would indicate
       * that we are no longer in an interrupt handler.  It will be non-NULL
       * if we are returning from a nested interrupt.
       */

      CURRENT_REGS = NULL;

#ifdef CONFIG_SMP
      /* Handle signal */

      sim_sigdeliver();

      /* Then switch contexts */

      longjmp(regs, 1);
    }
#endif

  return regs;
}
