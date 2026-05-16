/****************************************************************************
 * boards/arm/stm32/alientek-m144z-m4/src/stm32_spi.c
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

/* Board-specific SPI hooks for ALIENTEK M144Z-M4.
 *
 * The board has a single SPI bus exposed on the application side:
 *
 *   SPI1
 *     SCK  = PB3   (GPIO_SPI1_SCK_2,  set by stm32_spibus_initialize)
 *     MISO = PB4   (GPIO_SPI1_MISO_2, set by stm32_spibus_initialize)
 *     MOSI = PB5   (GPIO_SPI1_MOSI_2, set by stm32_spibus_initialize)
 *     NSS  = software, driven by stm32_spi1select() below
 *
 * On-board SPI devices:
 *
 *   W25Q128 16 MiB NOR flash
 *     CS   = PB14  (GPIO_W25_CS, active LOW)
 *
 * When new SPI devices are added (e.g. on-board display, off-board SD-card
 * adapter, off-board sensors) extend the SPIDEV_* branches below.  Keep
 * the list aligned with what is physically on the board -- this BSP only
 * registers chip selects for parts actually wired up.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "alientek-m144z-m4.h"

#ifdef CONFIG_STM32_SPI1

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Configure board-side chip-select GPIOs.  The SCK/MISO/MOSI alternate-
 *   function pins are configured by the common stm32_spibus_initialize()
 *   path; only the GPIO-driven CS lines have to be initialised here.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
#ifdef CONFIG_MTD_W25
  stm32_configgpio(GPIO_W25_CS);
#endif
}

/****************************************************************************
 * Name: stm32_spi1select / stm32_spi1status
 *
 * Description:
 *   Board-specific chip-select and status callbacks for SPI1.  The common
 *   STM32 SPI driver calls these once per transfer; we drive the chip
 *   select line low/high based on the requested SPIDEV_* devid.
 *
 ****************************************************************************/

void stm32_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  UNUSED(dev);

#ifdef CONFIG_MTD_W25
  if (devid == SPIDEV_FLASH(0))
    {
      stm32_gpiowrite(GPIO_W25_CS, !selected);
      return;
    }
#endif
}

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  UNUSED(dev);
  UNUSED(devid);

  /* No board-level status bits to expose; W25Q128 needs none. */

  return 0;
}

#endif /* CONFIG_STM32_SPI1 */
