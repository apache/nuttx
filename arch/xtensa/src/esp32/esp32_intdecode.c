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

#include <nuttx/sched.h>
#include <arch/chip/irq.h>

#include "chip/esp32_dport.h"
#include "xtensa.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_baseirq[3] =
{
  ESP32_IRQ_SREG0,
  ESP32_IRQ_SREG1,
  ESP32_IRQ_SREG2
};

static const uint8_t g_nirqs[3] =
{
  ESP32_NIRQS_SREG0,
  ESP32_NIRQS_SREG1,
  ESP32_NIRQS_SREG2
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_int_decode
 *
 * Description:
 *   Determine the peripheral that geneated the interrupt and dispatch
 *   handling to the registered interrupt handler via xtensa_irq_dispatch().
 *
 * Input Parameters:
 *   regs - Saves processor state on the stack
 *
 * Returned Value:
 *   Normally the same vale as regs is returned.  But, in the event of an
 *   interrupt level context switch, the returned value will, instead point
 *   to the saved processor state in the TCB of the newly started task.
 *
 ****************************************************************************/

uint32_t *xtensa_int_decode(uint32_t *regs)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t mask;
  int regndx;
  int bit;
  int baseirq;
  int nirqs;

#ifdef CONFIG_SMP
  int cpu;

  /* Select PRO or APP interrupt status registers */

  cpu = up_cpu_index();
  if (cpu == 0)
    {
      regaddr = DPORT_PRO_INTR_STATUS_0_REG;
    }
  else
#endif
    {
      regaddr = DPORT_APP_INTR_STATUS_0_REG;
    }

  /* Process each pending interrupt in each of the three interrupt status
   * registers.
   */

  for (regndx = 0; regndx < 3; regndx++)
    {
      /* Fetch the next register status register */

      regval   = getreg32(regaddr);
      regaddr += sizeof(uint32_t);

      /* Set up the search */

      baseirq = g_baseirq[regndx];
      nirqs   = g_nirqs[regndx];

      /* Decode and dispatch each pending bit in the interrupt status
       * register.
       */

      for (bit = 0; regval != 0 && bit < nirqs; bit++)
        {
          /* Check if this interrupt is pending */

          mask = (1 << bit);
          if ((regval & mask) != 0)
            {
              /* Yes.. Dispatch the interrupt.  Note that regs may be
               * altered in the case of an interrupt level context switch.
               */

              regs = xtensa_irq_dispatch(baseirq + bit, regs);

              /* Clear this bit in the sampled status register so that
               * perhaps we can exit this loop sooner.
               */

              regval &= ~mask;
            }
        }
    }

  return regs;
}
