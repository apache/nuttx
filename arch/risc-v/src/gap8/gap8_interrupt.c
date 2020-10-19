/****************************************************************************
 * arch/risc-v/src/gap8/gap8_interrupt.c
 * GAP8 event system
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
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
 *  GAP8 features a FC controller and a 8-core cluster. IRQ from peripherals
 *  have unique ID, which are dispatched to the FC or cluster by the SOC
 *  event unit, and then by the FC event unit or cluster event unit, and
 *  finally to FC or cluster. Peripherals share the same IRQ entry.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/chip/irq.h>

#include "gap8_udma.h"
#include "gap8_tim.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Function exported to the NuttX kernel */

void up_mdelay(unsigned int time)
{
  while (time--)
    {
      volatile int dummy = 200000;
      while (dummy--)
        {
        }
    }
}

/****************************************************************************
 * Name: up_get_newintctx
 *
 * Description:
 *   Return a value for EPIC. But GAP8 doesn't use EPIC for event control.
 *
 ****************************************************************************/

uint32_t up_get_newintctx(void)
{
  return 0;
}

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Initialize the IRQ on FC.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Deactivate all the soc events */

  SOC_EU->FC_MASK_MSB = 0xffffffff;
  SOC_EU->FC_MASK_LSB = 0xffffffff;

  /* enable soc peripheral interrupt */

  irq_attach(GAP8_IRQ_FC_UDMA, gap8_udma_doirq, NULL);
  up_enable_irq(GAP8_IRQ_FC_UDMA);

  /* Attach system call handler */

  extern int up_swint(int irq, FAR void *context, FAR void *arg);
  irq_attach(GAP8_IRQ_SYSCALL, up_swint, NULL);

  up_irq_enable();
}

/****************************************************************************
 * Name: gap8_dispatch_irq
 *
 * Description:
 *   Called from IRQ vectors. Input vector id. Return SP pointer, modified
 *   or not.
 *
 ****************************************************************************/

void *gap8_dispatch_irq(uint32_t vector, void *current_regs)
{
  /* Clear pending bit and trigger a software event.
   * GAP8 would sleep on sw event 3 on up_idle().
   */

  FCEU->BUFFER_CLEAR = (1 << vector);
  EU_SW_EVNT_TRIG->TRIGGER_SET[3] = 0;

  /* Call nuttx kernel, which may change curr_regs, to perform
   * a context switch
   */

  g_current_regs = current_regs;
  irq_dispatch(vector, current_regs);
  current_regs = (void *)g_current_regs;
  g_current_regs = NULL;

  return current_regs;
}
