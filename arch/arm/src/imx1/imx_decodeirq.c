/****************************************************************************
 * arch/arm/src/imx1/imx_decodeirq.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/addrenv.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  CURRENT_REGS = regs;
  err("ERROR: Unexpected IRQ\n");
  PANIC();
  return NULL;
#else
  uint32_t regval;
  int irq;

  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   *
   * Nested interrupts are not supported.
   */

  DEBUGASSERT(CURRENT_REGS == NULL);
  CURRENT_REGS = regs;

  /* Loop while there are pending interrupts to be processed */

  do
    {
      /* Decode the interrupt.  First, fetch the NIVECSR register. */

      regval = getreg32(IMX_AITC_NIVECSR);

      /* The MS 16 bits of the NIVECSR register contains vector index for the
       * highest pending normal interrupt.
       */

      irq = regval >> AITC_NIVECSR_NIVECTOR_SHIFT;

      /* If irq < 64, then this is the IRQ.
       * If there is no pending interrupt, then irq will be >= 64
       * (it will be 0xffff for illegal source).
       */

      if (irq < NR_IRQS)
        {
          /* Deliver the IRQ */

          irq_dispatch(irq, regs);

#ifdef CONFIG_ARCH_ADDRENV
          /* Check for a context switch.  If a context switch occurred, then
           * CURRENT_REGS will have a different value than it did on entry.
           * If an interrupt level context switch has occurred, then
           * establish the correct address environment before returning
           * from the interrupt.
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
        }
    }
  while (irq < NR_IRQS);

  /* Set CURRENT_REGS to NULL to indicate that we are no longer in
   * an interrupt handler.
   */

  CURRENT_REGS = NULL;
  return NULL;  /* Return not used in this architecture */
#endif
}
