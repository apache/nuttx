/****************************************************************************
 * arch/xtensa/src/esp32/esp32_intercpu_interrupt.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>>
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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/spinlock.h>
#include <arch/irq.h>

#include "chip/esp32_dport.h"
#include "xtensa.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Single parameter passed with the inter-CPU interrupt */

static volatile uint8_t g_intcode[CONFIG_SMP_NCPUS] SP_SECTION;

/* Spinlock protects parameter array */

static volatile spinlock_t g_intercpu_spin[CONFIG_SMP_NCPUS] SP_SECTION =
{
  SP_UNLOCKED, SP_UNLOCKED
};

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_fromcpu_interrupt
 *
 * Description:
 *   Common logic called to handle the from CPU0/1 interrupts.
 *
 ****************************************************************************/

static int esp32_fromcpu_interrupt(int fromcpu)
{
  uintptr_t regaddr;
  int intcode;
  int tocpu;

  DEBUGASSERT((unsigned)fromcpu < CONFIG_SMP_NCPUS);

  /* Clear the interrupt from the other CPU */

  regaddr = (fromcpu == 0) ? DPORT_CPU_INTR_FROM_CPU_0_REG :
                             DPORT_CPU_INTR_FROM_CPU_1_REG;
  putreg32(0, regaddr);

  /* Get the inter-CPU interrupt code */

  tocpu            = up_cpu_index();
  intcode          = g_intcode[tocpu];
  g_intcode[tocpu] = CPU_INTCODE_NONE;

  spin_unlock(&g_intercpu_spin[tocpu]);

  /* Dispatch the inter-CPU interrupt based on the intcode value */

  switch (intcode)
    {
      case CPU_INTCODE_NONE:
        break;

      case CPU_INTCODE_PAUSE:
        xtensa_pause_handler();
        break;

      default:
        DEBUGPANIC();
        break;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_fromcpu[0,1]_interrupt
 *
 * Description:
 *   Called to handle the from CPU0/1 interrupts.
 *
 ****************************************************************************/

int esp32_fromcpu0_interrupt(int irq, FAR void *context, FAR void *arg)
{
  return esp32_fromcpu_interrupt(0);
}

int esp32_fromcpu1_interrupt(int irq, FAR void *context, FAR void *arg)
{
  return esp32_fromcpu_interrupt(1);
}

/****************************************************************************
 * Name: xtensa_intercpu_interrupt
 *
 * Description:
 *   Called to trigger a CPU interrupt
 *
 ****************************************************************************/

int xtensa_intercpu_interrupt(int tocpu, int intcode)
{
  int fromcpu;

  DEBUGASSERT((unsigned)tocpu < CONFIG_SMP_NCPUS &&
              (unsigned)intcode <= UINT8_MAX);

  /* Make sure that each inter-cpu event is atomic.  The spinlock should
   * only be locked if we just completed sending an interrupt to this
   * CPU but the other CPU has not yet processed it.
   */

  spin_lock(&g_intercpu_spin[tocpu]);

  /* Save the passed parameter.  The previous interrupt code should be
   * CPU_INTCODE_NONE or we have overrun the other CPU.
   */

  DEBUGASSERT(g_intcode[tocpu] == CPU_INTCODE_NONE);
  g_intcode[tocpu] = intcode;

  /* Interrupt the other CPU (tocpu) form this CPU.  NOTE: that this logic
   * fails in numerous ways if fromcpu == tocpu (for example because non-
   * reentrant spinlocks are used).
   */

  fromcpu = up_cpu_index();
  DEBUGASSERT(fromcpu != tocpu);

  if (fromcpu == 0)
    {
      putreg32(DPORT_CPU_INTR_FROM_CPU_0, DPORT_CPU_INTR_FROM_CPU_0_REG);
    }
  else
    {
      putreg32(DPORT_CPU_INTR_FROM_CPU_1, DPORT_CPU_INTR_FROM_CPU_1_REG);
    }

  return OK;
}

#endif /* CONFIG_SMP */
