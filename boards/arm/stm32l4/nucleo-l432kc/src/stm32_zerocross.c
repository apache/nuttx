/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_zerocross.c
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sensors/zerocross.h>

#include "stm32l4_gpio.h"
#include "nucleo-l432kc.h"

#ifdef CONFIG_SENSORS_ZEROCROSS

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void zcross_enable(FAR const struct zc_lowerhalf_s *lower,
                          zc_interrupt_t handler, FAR void *arg);

static void zcross_disable(void);
static int  zcross_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Current interrupt handler and argument */

static zc_interrupt_t g_zcrosshandler;
static FAR void *g_zcrossarg;

/* This is the zero cross lower half driver interface */

static struct zc_lowerhalf_s g_zcrosslower =
{
  .zc_enable = zcross_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: zcross_enable
 *
 * Description:
 *   Enable interrupts on the selected zero cross pin.  And empty
 *   set will disable all interrupts.
 *
 ****************************************************************************/

static void zcross_enable(FAR const struct zc_lowerhalf_s *lower,
                          zc_interrupt_t handler, FAR void *arg)
{
  irqstate_t flags;
  bool rising = false;
  bool falling = true;

  /* Start with all interrupts disabled */

  flags = enter_critical_section();
  zcross_disable();

  sninfo("handler: %p arg: %p\n", handler, arg);

  if (handler)
    {
      g_zcrosshandler = handler;
      g_zcrossarg     = arg;
    }

  stm32l4_gpiosetevent(GPIO_ZEROCROSS, rising, falling,
                       true, zcross_interrupt, NULL);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: zcross_disable
 *
 * Description:
 *   Disable zero cross interrupts
 *
 ****************************************************************************/

static void zcross_disable(void)
{
  irqstate_t flags;

  /* Disable zero cross pin interrupt */

  flags = enter_critical_section();

  stm32l4_gpiosetevent(GPIO_ZEROCROSS, false, false, false, NULL, NULL);

  leave_critical_section(flags);

  /* Nullify the handler and argument */

  g_zcrosshandler = NULL;
  g_zcrossarg     = NULL;
}

/****************************************************************************
 * Name: zcross_interrupt
 *
 * Description:
 *   Zero Cross interrupt handler
 *
 ****************************************************************************/

static int zcross_interrupt(int irq, FAR void *context, FAR void *arg)
{
  DEBUGASSERT(g_zcrosshandler != NULL);
  if (g_zcrosshandler)
    {
      g_zcrosshandler(&g_zcrosslower, g_zcrossarg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_zerocross_initialize
 *
 * Description:
 *   Initialize and register the zero cross driver
 *
 ****************************************************************************/

int stm32_zerocross_initialize(void)
{
  /* Configure the GPIO pin as input.    NOTE: This is unnecessary for
   * interrupting pins since it will also be done by stm32l4_gpiosetevent().
   */

  stm32l4_configgpio(GPIO_ZEROCROSS);

  /* Make sure that all interrupts are disabled */

  zcross_disable();

  /* Register the zero cross device as /dev/zc0 */

  return zc_register("/dev/zc0", &g_zcrosslower);
}

#endif /* CONFIG_SENSORS_ZEROCROSS */
