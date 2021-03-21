/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_zerocross.c
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
