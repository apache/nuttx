/****************************************************************************
 * boards/arm/stm32/hymini-stm32v/src/stm32_ts.c
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
#include <nuttx/spi/spi.h>
#include <nuttx/arch.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "stm32.h"
#include "hymini-stm32v.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_STM32_SPI1)
# error CONFIG_STM32_SPI1 must be defined to use the ADS7843 on this board
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hymini_ts_irq_attach(struct ads7843e_config_s *state,
                                xcpt_t isr);
static void hymini_ts_irq_enable(struct ads7843e_config_s *state,
                                 bool enable);
static void hymini_ts_irq_clear(struct ads7843e_config_s *state);
static bool hymini_ts_busy(struct ads7843e_config_s *state);
static bool hymini_ts_pendown(struct ads7843e_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ads7843e_config_s ts_cfg =
{
  .frequency = CONFIG_ADS7843E_FREQUENCY,

  .attach = hymini_ts_irq_attach,
  .enable = hymini_ts_irq_enable,
  .clear = hymini_ts_irq_clear,
  .busy = hymini_ts_busy,
  .pendown = hymini_ts_pendown
};

static xcpt_t tc_isr;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Attach the ADS7843E interrupt handler to the GPIO interrupt */

static int hymini_ts_irq_attach(struct ads7843e_config_s *state,
                                xcpt_t isr)
{
  iinfo("hymini_ts_irq_attach\n");

  tc_isr = isr;
  stm32_gpiosetevent(GPIO_TS_IRQ, true, true, true, isr, NULL);
  return OK;
}

/* Enable or disable the GPIO interrupt */

static void hymini_ts_irq_enable(struct ads7843e_config_s *state,
                                 bool enable)
{
  iinfo("%d\n", enable);

  stm32_gpiosetevent(GPIO_TS_IRQ, true, true, true,
                     enable ? tc_isr : NULL, NULL);
}

/* Acknowledge/clear any pending GPIO interrupt */

static void hymini_ts_irq_clear(struct ads7843e_config_s *state)
{
  /* FIXME  Nothing to do ? */
}

/* As the busy line is not connected, we just wait a little bit here */

static bool hymini_ts_busy(struct ads7843e_config_s *state)
{
  up_mdelay(50);
  return false;
}

/* Return the state of the pen down GPIO input */

static bool hymini_ts_pendown(struct ads7843e_config_s *state)
{
  bool pin_value = stm32_gpioread(GPIO_TS_IRQ);
  return !pin_value;
}

/****************************************************************************
 * Name: stm32_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_tsc_setup(int minor)
{
  struct spi_dev_s *dev;

  iinfo("minor %d\n", minor);

  dev = stm32_spibus_initialize(1);
  if (!dev)
    {
      ierr("ERROR: Failed to initialize SPI bus\n");
      return -ENODEV;
    }

  /* Configure the PIO interrupt */

  stm32_configgpio(GPIO_TS_IRQ);

  return ads7843e_register(dev, &ts_cfg, minor);
}
