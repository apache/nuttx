/****************************************************************************
 * boards/arm/stm32wb/flipperzero/src/stm32_spi.c
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

#include "stm32wb_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void weak_function stm32wb_spidev_initialize(void)
{
  /* NOTE: Clocking was already provided in stm32wb_rcc.c.
   *       Here, we only initialize chip select pins unique to the board
   *       architecture.
   */

#ifdef CONFIG_LCD_ST7565
  stm32wb_configgpio(STM32WB_LCD_CS);       /* ST7565 chip select */
#endif
}

/****************************************************************************
 * Name:  stm32wb_spi1/2select and stm32wb_spi1/2status
 *
 * Description:
 *   The external functions, stm32wb_spi1/2select and stm32wb_spi1/2status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   stm32wb_spibus_initialize()) are provided by common STM32 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32wb_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide stm32wb_spi1/2select() and stm32wb_spi1/2status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32wb_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32wb_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32WB_SPI2
void stm32wb_spi2select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
#ifdef CONFIG_LCD_ST7565
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32wb_gpiowrite(STM32WB_LCD_CS, !selected);
    }
#endif
}

uint8_t stm32wb_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif
