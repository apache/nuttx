/****************************************************************************
 * boards/arm/rp23xx/common/src/rp23xx_spi.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "rp23xx_gpio.h"
#include "hardware/rp23xx_spi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  rp23xx_spi0/1select and rp23xx_spi0/1status
 *
 * Description:
 *   The external functions, rp23xx_spi0/1select and rp23xx_spi0/1status
 *   must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including rp23xx_spibus_initialize()) are provided by
 *   common RP23XX logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in rp23xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide rp23xx_spi0/1select() and rp23xx_spi0/1status()
 *      functions in your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   3. Add a calls to rp23xx_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by rp23xx_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_RP23XX_SPI0
void rp23xx_spi0select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  rp23xx_gpio_put(CONFIG_RP23XX_SPI0_CS_GPIO, !selected);
}

uint8_t rp23xx_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

#  if defined(CONFIG_RP23XX_SPISD) && (CONFIG_RP23XX_SPISD_SPI_CH == 0)
  ret = board_spisd_status(dev, devid);
#  endif
  return ret;
}

#ifdef CONFIG_SPI_CMDDATA
int rp23xx_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#ifdef CONFIG_LCD_ST7789
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      rp23xx_gpio_put(CONFIG_RP23XX_SPI0_RX_GPIO, !cmd);

      return OK;
    }
#endif

  return -ENODEV;
}
#endif
#endif

#ifdef CONFIG_RP23XX_SPI1
void rp23xx_spi1select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  rp23xx_gpio_put(CONFIG_RP23XX_SPI1_CS_GPIO, !selected);
}

uint8_t rp23xx_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

#  if defined(CONFIG_RP23XX_SPISD) && (CONFIG_RP23XX_SPISD_SPI_CH == 1)
  ret = board_spisd_status(dev, devid);
#  endif
  return ret;
}

#ifdef CONFIG_SPI_CMDDATA
int rp23xx_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#if defined (CONFIG_LCD_ST7789) || defined (CONFIG_LCD_ST7735) || defined (CONFIG_LCD_GC9A01)
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      rp23xx_gpio_put(CONFIG_RP23XX_SPI1_RX_GPIO, !cmd);

      return OK;
    }
#endif

  return -ENODEV;
}
#endif
#endif
