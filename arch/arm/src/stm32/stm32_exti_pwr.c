/****************************************************************************
 * arch/arm/src/stm32/stm32_exti_pwr.c
 *
 *   Copyright (C) 2009, 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_exti_pwr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to the PVD EXTI */

static xcpt_t g_pvd_callback;
static void  *g_callback_arg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_pvd_isr
 *
 * Description:
 *   EXTI PVD interrupt service routine/dispatcher
 *
 ****************************************************************************/

static int stm32_exti_pvd_isr(int irq, void *context, FAR void *arg)
{
  int ret = OK;

  /* Clear the pending EXTI interrupt */

  putreg32(EXTI_PVD_LINE, STM32_EXTI_PR);

  /* And dispatch the interrupt to the handler */

  if (g_pvd_callback != NULL)
    {
      ret = g_pvd_callback(irq, context, g_callback_arg);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_pvd
 *
 * Description:
 *   Sets/clears EXTI PVD interrupt.
 *
 * Input Parameters:
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

int stm32_exti_pvd(bool risingedge, bool fallingedge, bool event,
                   xcpt_t func, void *arg)
{
  /* Get the previous GPIO IRQ handler; Save the new IRQ handler. */

  g_pvd_callback = func;
  g_callback_arg = arg;

  /* Install external interrupt handlers (if not already attached) */

  if (func)
    {
      irq_attach(STM32_IRQ_PVD, stm32_exti_pvd_isr, NULL);
      up_enable_irq(STM32_IRQ_PVD);
    }
  else
    {
      up_disable_irq(STM32_IRQ_PVD);
    }

  /* Configure rising/falling edges */

  modifyreg32(STM32_EXTI_RTSR,
              risingedge ? 0 : EXTI_PVD_LINE,
              risingedge ? EXTI_PVD_LINE : 0);
  modifyreg32(STM32_EXTI_FTSR,
              fallingedge ? 0 : EXTI_PVD_LINE,
              fallingedge ? EXTI_PVD_LINE : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32_EXTI_EMR,
              event ? 0 : EXTI_PVD_LINE,
              event ? EXTI_PVD_LINE : 0);
  modifyreg32(STM32_EXTI_IMR,
              func ? 0 : EXTI_PVD_LINE,
              func ? EXTI_PVD_LINE : 0);

  return OK;
}
