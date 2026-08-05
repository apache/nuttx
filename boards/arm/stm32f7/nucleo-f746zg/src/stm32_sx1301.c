/****************************************************************************
 * boards/arm/stm32f7/nucleo-f746zg/src/stm32_sx1301.c
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

/* Support for a LoRa concentrator shield of the LRWAN_GS_HF1 family (for
 * instance the RisingHF RHF0M301, an SX1301 with two SX1257 front-ends) on
 * SPI4 of the Morpho connector.  See include/board.h for the pin map.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/wireless/lpwan/sx1301.h>

#include <arch/board/board.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "nucleo-f746zg.h"

#ifdef CONFIG_LPWAN_SX1301

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SX1301_SPI_BUS 4

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void stm32_sx1301_reset(FAR const struct sx1301_lower_s *lower,
                               bool assert);
static void stm32_sx1301_band(FAR const struct sx1301_lower_s *lower,
                              int band_mhz);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sx1301_lower_s g_sx1301_lower =
{
  .reset       = stm32_sx1301_reset,
  .band_select = stm32_sx1301_band
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sx1301_reset
 *
 * Description:
 *   Drive the reset line of the concentrator, which is active high.
 *
 ****************************************************************************/

static void stm32_sx1301_reset(FAR const struct sx1301_lower_s *lower,
                               bool assert)
{
  UNUSED(lower);

  stm32_gpiowrite(GPIO_SX1301_RESET, assert);
}

/****************************************************************************
 * Name: stm32_sx1301_band
 *
 * Description:
 *   Select the band of the front-end filter bank of the shield.
 *
 ****************************************************************************/

static void stm32_sx1301_band(FAR const struct sx1301_lower_s *lower,
                              int band_mhz)
{
  UNUSED(lower);

  if (band_mhz >= 900)
    {
      stm32_gpiowrite(GPIO_SX1301_BAND1, false);
      stm32_gpiowrite(GPIO_SX1301_BAND2, true);
    }
  else
    {
      stm32_gpiowrite(GPIO_SX1301_BAND1, true);
      stm32_gpiowrite(GPIO_SX1301_BAND2, false);
    }

  ninfo("Band set to %d MHz\n", band_mhz);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sx1301_initialize
 *
 * Description:
 *   Initialise SPI4 and register the concentrator driver.
 *
 ****************************************************************************/

int stm32_sx1301_initialize(FAR const char *devpath)
{
  FAR struct spi_dev_s *spi;
  int ret;

  /* Control lines first, so that the chip stays in reset until it is
   * started.
   */

  stm32_configgpio(GPIO_SX1301_RESET);
  stm32_configgpio(GPIO_SX1301_BAND1);
  stm32_configgpio(GPIO_SX1301_BAND2);

  stm32_gpiowrite(GPIO_SX1301_RESET, true);

  spi = stm32_spibus_initialize(SX1301_SPI_BUS);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: cannot initialise SPI port %d\n",
             SX1301_SPI_BUS);
      return -ENODEV;
    }

  ret = sx1301_register(devpath, spi, &g_sx1301_lower);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cannot register the concentrator: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "SX1301 concentrator at %s, SPI%d\n", devpath,
         SX1301_SPI_BUS);
  return OK;
}

#endif /* CONFIG_LPWAN_SX1301 */
