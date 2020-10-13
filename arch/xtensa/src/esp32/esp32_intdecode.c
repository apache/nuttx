/****************************************************************************
 * arch/xtensa/src/esp32/esp32_intdecode.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "esp32_cpuint.h"

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
#ifdef CONFIG_SMP
  int cpu;
#endif

#ifdef CONFIG_SMP
  /* Select PRO or APP CPU interrupt mapping table */

  cpu = up_cpu_index();
  if (cpu != 0)
    {
      intmap = g_cpu1_intmap;
    }
  else
#endif
    {
      intmap = g_cpu0_intmap;
    }

  /* Skip over zero bits, eight at a time */

  for (bit = 0, mask = 0xff;
       bit < ESP32_NCPUINTS && (cpuints & mask) == 0;
       bit += 8, mask <<= 8);

  /* Process each pending CPU interrupt */

  for (; bit < ESP32_NCPUINTS && cpuints != 0; bit++)
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
