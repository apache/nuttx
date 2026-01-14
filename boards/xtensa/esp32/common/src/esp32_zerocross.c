/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_zerocross.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <nuttx/sensors/zerocross.h>

#include "esp32_gpio.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "esp32-wrover-kit.h"
#include "esp32_zerocross.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void zcross_enable(const struct zc_lowerhalf_s *lower,
                          zc_interrupt_t handler, void *arg);

static int zcross_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Current interrupt handler and argument */

static zc_interrupt_t g_zcrosshandler;
static void *g_zcrossarg;

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

static void zcross_enable(const struct zc_lowerhalf_s *lower,
                          zc_interrupt_t handler, void *arg)
{
  irqstate_t flags;
  int irq = ESP32_PIN2IRQ(GPIO_ZERO_CROSS_IRQ);
  int ret;

  flags = enter_critical_section();

  if (handler)
    {
      g_zcrosshandler = handler;
      g_zcrossarg = arg;
    }

  /* Start with all interrupts disabled */

  esp32_gpioirqdisable(irq);

  ret = irq_attach(irq, zcross_interrupt, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: zcross_enable() failed: %d\n", ret);
      leave_critical_section(flags);
    }

  esp32_gpioirqenable(irq, RISING);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: zcross_interrupt
 *
 * Description:
 *   Zero Cross interrupt handler
 *
 ****************************************************************************/

static int zcross_interrupt(int irq, void *context, void *arg)
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
 * Name: esp32_zerocross_initialize
 *
 * Description:
 *   Initialize and register the zero cross driver
 *
 ****************************************************************************/

int board_zerocross_initialize(int devno)
{
  esp32_configgpio(GPIO_ZERO_CROSS_IRQ, INPUT_FUNCTION_3 | PULLUP);

  /* Register the zero cross device as /dev/zc0 */

  return zc_register("/dev/zc0", &g_zcrosslower);
}
