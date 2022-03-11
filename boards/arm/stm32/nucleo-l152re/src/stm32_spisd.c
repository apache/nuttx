/****************************************************************************
 * boards/arm/stm32/nucleo-l152re/src/stm32_spisd.c
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

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_spi.h"
#include "stm32_gpio.h"

#include <arch/board/board.h>
#include "nucleo-l152re.h"

#if defined(CONFIG_STM32_SPI1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:
 *   stm32_spi1register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

int stm32_spi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  /* TODO: media change callback */

  return OK;
}

/****************************************************************************
 * Name: stm32_spisd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 ****************************************************************************/

int stm32_spisd_initialize(int port, int minor)
{
  struct spi_dev_s *spi;
  int rv;
  stm32_configgpio(GPIO_SPI1_SCK);
  stm32_configgpio(GPIO_SPI1_CS);   /* Assign CS */
  stm32_gpiowrite(GPIO_SPI1_CS, 1); /* Ensure the CS is inactive */

  mcinfo("INFO: Initializing mmcsd port %d minor %d\n",
         port, minor);

  spi = stm32_spibus_initialize(port);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI port %d\n", port);
      return -ENODEV;
    }

  rv = mmcsd_spislotinitialize(minor, minor, spi);
  if (rv < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
            port, minor);
      return rv;
    }

  spiinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}

#endif /* CONFIG_STM32_SPI1 */
