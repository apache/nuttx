/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_intdecode.c
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

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "esp32s2_cpuint.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_intclear
 ****************************************************************************/

static inline void xtensa_intclear(uint32_t mask)
{
  __asm__ __volatile__
  (
    "wsr %0, INTCLEAR\n"
    : "=r"(mask) : :
  );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_int_decode
 *
 * Description:
 *   Determine the peripheral that generated the interrupt and dispatch
 *   handling to the registered interrupt handler via xtensa_irq_dispatch().
 *
 * Input Parameters:
 *   cpuints - Set of pending interrupts valid for this level
 *   regs    - Saves processor state on the stack
 *
 * Returned Value:
 *   Normally the same value as regs is returned.  But, in the event of an
 *   interrupt level context switch, the returned value will, instead point
 *   to the saved processor state in the TCB of the newly started task.
 *
 ****************************************************************************/

uint32_t *xtensa_int_decode(uint32_t cpuints, uint32_t *regs)
{
  uint8_t *intmap;
  uint32_t mask;
  int bit;

  intmap = g_cpu0_intmap;

  /* Skip over zero bits, eight at a time */

  for (bit = 0, mask = 0xff;
       bit < ESP32S2_NCPUINTS && (cpuints & mask) == 0;
       bit += 8, mask <<= 8);

  /* Process each pending CPU interrupt */

  for (; bit < ESP32S2_NCPUINTS && cpuints != 0; bit++)
    {
      mask = (1 << bit);
      if ((cpuints & mask) != 0)
        {
          /* Extract the IRQ number from the mapping table */

          uint8_t irq = intmap[bit];
          DEBUGASSERT(irq != CPUINT_UNASSIGNED);

          /* Clear software or edge-triggered interrupt */

           xtensa_intclear(mask);

          /* Dispatch the CPU interrupt.
           *
           * NOTE that regs may be altered in the case of an interrupt
           * level context switch.
           */

          regs = xtensa_irq_dispatch((int)irq, regs);

          /* Clear the bit in the pending interrupt so that perhaps
           * we can exit the look early.
           */

          cpuints &= ~mask;
        }
    }

  return regs;
}
