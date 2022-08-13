/****************************************************************************
 * boards/arm/stm32/fire-stm32v2/src/stm32_spi.c
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
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "fire-stm32v2.h"

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the M3 Wildfire board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
  /* NOTE: Clocking for SPI1 and/or SPI2 was already provided in stm32_rcc.c.
   *       Configurations of SPI pins is performed in stm32_spi.c.
   *       Here, we only initialize chip select pins unique to the board
   *       architecture.
   */

#ifdef CONFIG_STM32_SPI1
  /* Configure the TFT/Touchscreen CS GPIO */

#if 0 /* Need to study this */
  stm32_configgpio(GPIO_LCD_CS);
#endif

  /* Configure the TFT/Touchscreen and ENC28J60 or SPI-based FLASH PIOs */

  /* Configure ENC28J60 SPI1 CS (also RESET and interrupt pins) */

#ifdef CONFIG_ENC28J60
  stm32_configgpio(GPIO_ENC28J60_CS);
  stm32_configgpio(GPIO_ENC28J60_RESET);
  stm32_configgpio(GPIO_ENC28J60_INTR);
#else

  /* Configure FLASH SPI1 CS */

  stm32_configgpio(GPIO_FLASH_CS);
#endif

#endif /* CONFIG_STM32_SPI1 */

#ifdef CONFIG_STM32_SPI2
  /* Configure the MP3 SPI2 CS GPIO */

  stm32_configgpio(GPIO_MP3_CS);

#endif /* CONFIG_STM32_SPI2 */
}

/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   (including stm32_spibus_initialize()) are provided by common STM32
 *   logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spi1select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

#if 0 /* Need to study this */
  if (devid == SPIDEV_LCD)
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_LCD_CS, !selected);
    }
  else
#endif
#ifdef CONFIG_ENC28J60
  if (devid == SPIDEV_ETHERNET(0))
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_ENC28J60_CS, !selected);
    }
#else
  if (devid == SPIDEV_FLASH(0))
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_FLASH_CS, !selected);
    }
#endif
}

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

  if (devid == SPIDEV_AUDIO)
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_MP3_CS, !selected);
    }
}

uint8_t stm32_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}
#endif

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 */
