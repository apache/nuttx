/****************************************************************************
 * arch/xtensa/src/esp32/esp32_cpu_interrupt.c
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

#include <arch/irq.h>

#include "xtensa.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_cpu_interrupt
 *
 * Description:
 *   Called to handle the CPU0/1 interrupts.
 *
 ****************************************************************************/

int esp32_cpu_interrupt(int irq, FAR void *context)
{
  uint32_t *regs = (uint32_t *)context;
  int intcode;

  DEBUGASSERT(regs != NULL);
  intcode = regs[REG_A2];

  /* Dispatch the inter-CPU interrupt based on the intcode value */

  switch (intcode)
    {
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
 * Name: xtensa_cpu_interrupt
 *
 * Description:
 *   Called to trigger a CPU interrupt
 *
 ****************************************************************************/

int xtensa_cpu_interrupt(int cpu, int intcode)
{
#warning Missing logic -- How do we do this?
  return -ENOSYS;
}

#endif /* CONFIG_SMP */
