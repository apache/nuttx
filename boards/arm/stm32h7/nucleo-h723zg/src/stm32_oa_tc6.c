/****************************************************************************
 * boards/arm/stm32h7/nucleo-h723zg/src/stm32_oa_tc6.c
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

#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/net/oa_tc6.h>

#include "stm32.h"
#include "nucleo-h723zg.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int board_oa_tc6_attach(FAR const struct oa_tc6_config_s *config,
                               xcpt_t handler,
                               FAR void *arg);

static int board_oa_tc6_enable(FAR const struct oa_tc6_config_s *config,
                               bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oa_tc6_config_s g_stm32_oa_tc6_config =
{
  .id                 = SPIDEV_ETHERNET(0),
  .frequency          = 25000000,
  .chunk_payload_size = 64,
  .rx_cut_through     = true,
  .attach             = board_oa_tc6_attach,
  .enable             = board_oa_tc6_enable,
};

static xcpt_t g_stm32_oa_tc6_isr    = NULL;
static void *g_stm32_oa_tc6_isr_arg = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_oa_tc6_attach
 *
 * Description:
 *   Configure interrupt and attach handler to the interrupt signal
 *   from the MAC-PHY.
 *
 * Input Parameters:
 *   config  - reference to the MAC-PHY configuration
 *   handler - the interrupt handler function
 *   arg     - reference passed to the handler on invocation
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int board_oa_tc6_attach(FAR const struct oa_tc6_config_s *config,
                               xcpt_t handler, FAR void *arg)
{
  uint32_t pin;

  switch (config->id)
    {
      case SPIDEV_ETHERNET(0):
          pin = GPIO_OA_TC6_INT;
          break;
      default:

          /* Unknown id */

          DEBUGPANIC();
          return ERROR;
    }

  /* Configure pin and disable all possible interrupt events */

  stm32_configgpio(pin);
  stm32_gpiosetevent(pin, false, false, false, NULL, NULL);

  g_stm32_oa_tc6_isr     = handler;
  g_stm32_oa_tc6_isr_arg = arg;

  return OK;
}

/****************************************************************************
 * Name: board_oa_tc6_enable
 *
 * Description:
 *   Enable or disable the interrupt on the hardware level.
 *   Enable when the enable argument is true, otherwise disable.
 *
 * Input Parameters:
 *   config - reference to the MAC-PHY configuration
 *   enable - enable / disable control
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int board_oa_tc6_enable(FAR const struct oa_tc6_config_s *config,
                               bool enable)
{
  uint32_t pin;

  switch (config->id)
    {
      case SPIDEV_ETHERNET(0):
          pin = GPIO_OA_TC6_INT;
          break;
      default:

          /* Unknown id */

          DEBUGPANIC();
          return ERROR;
    }

  DEBUGASSERT(g_stm32_oa_tc6_isr != NULL && g_stm32_oa_tc6_isr_arg != NULL);
  if (enable)
    {
      /* Enable interrupt on the falling edge */

      stm32_gpiosetevent(pin, false, true, false,
                         g_stm32_oa_tc6_isr, g_stm32_oa_tc6_isr_arg);
    }
  else
    {
      stm32_gpiosetevent(pin, false, false, false, NULL, NULL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_oa_tc6_initialize
 *
 * Description:
 *   Initialize and register the OA-TC6 10BASE-T1x network driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_oa_tc6_initialize(void)
{
  FAR struct spi_dev_s *spi;
  int ret;

  spi = stm32_spibus_initialize(GPIO_OA_TC6_SPI_PORT);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
             GPIO_OA_TC6_SPI_PORT);
      return -ENODEV;
    }

  /* Bind the SPI port and config to the OA-TC6 driver */

  ret = oa_tc6_initialize(spi, &g_stm32_oa_tc6_config);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SPI port and config to the OA-TC6"
             " network driver: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO,
         "Bound SPI and config to the OA-TC6 network driver\n");

  return OK;
}
