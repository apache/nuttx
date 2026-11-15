/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_spi.c
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
#include <stdbool.h>
#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32.h"
#include "stm32n6570-dk.h"

#ifdef CONFIG_STM32_SPI

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI5
static struct spi_dev_s *g_spi5;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Initialize the SPI buses and their chip select GPIOs.  The SPI pins
 *   themselves are configured by the SPI driver.
 *
 ****************************************************************************/

void stm32_spiinitialize(void)
{
#ifdef CONFIG_STM32_SPI5
  g_spi5 = stm32_spibus_initialize(5);
  if (g_spi5 == NULL)
    {
      spierr("ERROR: Failed to initialize SPI port 5\n");
    }
  else
    {
      stm32_configgpio(GPIO_SPI5_CS);
    }
#endif
}

/****************************************************************************
 * Name: stm32_spiregister
 *
 * Description:
 *   Register the SPI character drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DRIVER
void stm32_spiregister(void)
{
#ifdef CONFIG_STM32_SPI5
  int ret;

  if (g_spi5 != NULL)
    {
      ret = spi_register(g_spi5, 5);
      if (ret < 0)
        {
          spierr("ERROR: Failed to register SPI port 5: %d\n", ret);
        }
    }
#endif
}
#endif

/****************************************************************************
 * Name: stm32_spi5select, stm32_spi5status and stm32_spi5cmddata
 *
 * Description:
 *   Board-specific chip select, status and command/data callbacks for the
 *   SPI5 bus.  The only chip select is the Arduino D10 line.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI5
void stm32_spi5select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08" PRIx32 " CS: %s\n", devid,
          selected ? "assert" : "de-assert");

  stm32_gpiowrite(GPIO_SPI5_CS, !selected);
}

uint8_t stm32_spi5status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int stm32_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif /* CONFIG_STM32_SPI5 */

#endif /* CONFIG_STM32_SPI */
