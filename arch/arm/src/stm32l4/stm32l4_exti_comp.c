/****************************************************************************
 * arch/arm/src/stm32/stm32l4_exti_comp.c
 *
 *   Copyright (c) 2017 Gregory Nutt. All rights reserved
 *   Copyright (c) 2016 Motorola Mobility, LLC. All rights reserved.
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm_arch.h"
#include "stm32l4_comp.h"
#include "stm32l4_exti.h"
#include "hardware/stm32l4_exti.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct comp_callback_s
{
  xcpt_t callback;
  void  *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to the COMP EXTI lines */

static struct comp_callback_s g_comp_handlers[STM32L4_COMP_NUM];

/* Comparator EXTI lines */

static const uint32_t g_comp_lines[STM32L4_COMP_NUM] =
{
#if defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4X5) || \
    defined(CONFIG_STM32L4_STM32L4X6) || defined(CONFIG_STM32L4_STM32L4XR)
  EXTI1_COMP1,
  EXTI1_COMP2
#else
#  error "Unrecognized STM32L4 chip"
#endif
};

 /****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32l4_exti_comp_isr(int irq, void *context, FAR void *arg)
{
  uint32_t pr;
  uint32_t ln;
  int ret = 0;
  int i;

  /* Examine the state of each comparator line and dispatch interrupts */

  pr = getreg32(STM32L4_EXTI1_PR);
  for (i = 0; i < STM32L4_COMP_NUM; i++)
    {
      ln = g_comp_lines[i];
      if ((pr & ln) != 0)
        {
          /* Clear the pending interrupt */

          putreg32(ln, STM32L4_EXTI1_PR);
          if (g_comp_handlers[i].callback != NULL)
            {
              xcpt_t callback = g_comp_handlers[i].callback;
              void *callback_arg = g_comp_handlers[i].arg;
              ret = callback(irq, context, callback_arg);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_exti_comp
 *
 * Description:
 *   Sets/clears comparator based events and interrupt triggers.
 *
 * Input Parameters:
 *  - cmp: comparator
 *  - rising/falling edge: enables interrupt on rising/falling edge
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *  - arg:    Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int stm32l4_exti_comp(int cmp, bool risingedge, bool fallingedge,
                      bool event, xcpt_t func, void *arg)
{
  irqstate_t flags;
  uint32_t ln = g_comp_lines[cmp];

 /* Perform the following within a critical section so that the handler gets
  * installed correctly before the next interrupt is received.
  */

  flags = enter_critical_section();

  /* Install external interrupt handlers */

  if (func != NULL)
    {
      irq_attach(STM32L4_IRQ_COMP, stm32l4_exti_comp_isr, NULL);
      up_enable_irq(STM32L4_IRQ_COMP);
    }
  else
    {
      up_disable_irq(STM32L4_IRQ_COMP);
    }

  /* Configure rising/falling edges */

  modifyreg32(STM32L4_EXTI1_RTSR, risingedge  ? 0 : ln, risingedge  ? ln : 0);
  modifyreg32(STM32L4_EXTI1_FTSR, fallingedge ? 0 : ln, fallingedge ? ln : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32L4_EXTI1_EMR, event ? 0 : ln, event ? ln : 0);
  modifyreg32(STM32L4_EXTI1_IMR, func  ? 0 : ln, func  ? ln : 0);

  /* Get the previous IRQ handler and save the new IRQ handler. */

  g_comp_handlers[cmp].callback = func;
  g_comp_handlers[cmp].arg      = arg;

  /* Leave the critical section */

  leave_critical_section(flags);
  return OK;
}
