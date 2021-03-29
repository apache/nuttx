/****************************************************************************
 * arch/z80/src/z8/z8_saveirqcontext.c
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
#include <arch/irq.h>

#include "chip/switch.h"
#include "z80_internal.h"

/****************************************************************************
 * Pre-processor Definitions
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

/****************************************************************************
 * Name: z8_saveirqcontext
 *
 * Description:
 *   In order to provide faster interrupt handling, the interrupt logic does
 *   "lazy" context saving as described below:
 *
 *   (1) At the time of the interrupt, minimum information is saved and the
 *       register pointer is changed so that the interrupt logic does not
 *       alter the state of the interrupted task's registers.
 *   (2) If no context switch occurs during the interrupt processing, then
 *       the return from interrupt is also simple.
 *   (3) If a context switch occurs during interrupt processing, then
 *       (a) The full context of the interrupt task is saved, and
 *       (b) A full context switch is performed when the interrupt exits
 *           (see z8_vector.S).
 *
 * This function implements the full-context switch of bullet 3a.
 *
 ****************************************************************************/

void z8_saveirqcontext(FAR chipreg_t *regs)
{
  /* If we have already saved the interrupted task's registers in the TCB,
   * then we do not need to do anything.
   */

  if (g_z8irqstate.state == Z8_IRQSTATE_ENTRY)
    {
      /* Calculate the source address based on the saved RP value */

      uint16_t       rp  = g_z8irqstate.regs[Z8_IRQSAVE_RPFLAGS] >> 8;
      FAR chipreg_t *src = (FAR uint16_t *)(rp & 0xf0);
      FAR chipreg_t *dest = &regs[XCPT_RR0];

      /* Copy the interrupted tasks register into the TCB register save
       * area.
       */

      int i;
      for (i = 0; i < XCPTCONTEXT_REGS; i++)
        {
          *dest++ = *src++;
        }

      /* Since the task was interrupted, we know that interrupts were
       * enabled
       */

      regs[XCPT_IRQCTL] = 0x0080; /* IRQE bit will enable interrupts */

      /* The g_z8irqstate.regs pointer is the value of the stack pointer at
       * the time that z80_doirq() was called.  Therefore, we can calculate
       * the correct value for the stack pointer on return from interrupt:
       */

      regs[XCPT_SP] = ((chipreg_t)g_z8irqstate.regs) + Z8_IRQSAVE_SIZE;

      /* Copy the PC, RP, and FLAGS information from the lazy save to the TCB
       * register save area.
       */

      regs[XCPT_RPFLAGS] = g_z8irqstate.regs[Z8_IRQSAVE_RPFLAGS];
      regs[XCPT_PC]      = g_z8irqstate.regs[Z8_IRQSAVE_PC];

      /* Now update the IRQ save area so that we will know that we have
       * already done this.
       */

      g_z8irqstate.state = Z8_IRQSTATE_SAVED;
      g_z8irqstate.regs  = regs;
    }
}
