/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_sdmmc.c
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

#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_sdmmc.h"
#include "stm32n6570-dk.h"

#ifdef CONFIG_MMCSD_SDIO

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sdio_dev_s *g_sdio_dev;
static bool g_sd_inserted;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ncd_interrupt
 *
 * Description:
 *   Card detect interrupt handler.
 *
 ****************************************************************************/

static int stm32_ncd_interrupt(int irq, void *context, void *arg)
{
  bool present = !stm32_gpioread(GPIO_SDMMC2_NCD);

  if (present != g_sd_inserted)
    {
      g_sd_inserted = present;
      sdio_mediachange(g_sdio_dev, present);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Power the microSD slot, bind SDMMC2 to the MMC/SD driver as
 *   /dev/mmcsd0 and track the card detect switch.
 *
 ****************************************************************************/

int stm32_sdio_initialize(void)
{
  int ret;

  /* Power the card and give the supply time to settle */

  stm32_configgpio(GPIO_SDMMC2_PWR_EN);
  stm32_gpiowrite(GPIO_SDMMC2_PWR_EN, true);
  up_mdelay(10);

  g_sdio_dev = sdio_initialize(SDIO_SLOTNO);
  if (g_sdio_dev == NULL)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n", SDIO_SLOTNO);
      return -ENODEV;
    }

  ret = mmcsd_slotinitialize(SDIO_MINOR, g_sdio_dev);
  if (ret < 0)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  /* Report the current card state and follow the card detect switch */

  stm32_configgpio(GPIO_SDMMC2_NCD);
  g_sd_inserted = !stm32_gpioread(GPIO_SDMMC2_NCD);
  sdio_mediachange(g_sdio_dev, g_sd_inserted);

  ret = stm32_gpiosetevent(GPIO_SDMMC2_NCD, true, true, true,
                           stm32_ncd_interrupt, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to attach card detect interrupt: %d\n", ret);
    }

  return OK;
}

#endif /* CONFIG_MMCSD_SDIO */
