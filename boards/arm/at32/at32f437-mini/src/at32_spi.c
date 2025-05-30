/****************************************************************************
 * boards/arm/at32/at32f437-mini/src/at32_spi.c
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
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "at32.h"
#include "at32f437-mini.h"

#if defined(CONFIG_AT32_SPI1) || defined(CONFIG_AT32_SPI2) || \
defined(CONFIG_AT32_SPI3)|| defined(CONFIG_AT32_SPI4)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the HY-MiniAT32
 *   board.
 *
 ****************************************************************************/

void at32_spidev_initialize(void)
{
  /* NOTE: Clocking for SPI1 and/or SPI2 was already provided in at32_rcc.c.
   *       Configurations of SPI pins is performed in at32_spi.c.
   *       Here, we only initialize chip select pins unique to the board
   *       architecture.
   */

#ifdef CONFIG_MTD_W25
  at32_configgpio(FLASH_SPI1_CS);      /* FLASH chip select */
#endif
}

/****************************************************************************
 * Name:  at32_spi1/2select and at32_spi1/2status
 *
 * Description:
 *   The external functions, at32_spi1/2/3select and at32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   at32_spibus_initialize()) are provided by common AT32 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in at32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide at32_spi1/2/3select() and at32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to at32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by at32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_SPI1
void at32_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  switch (devid)
    {
  #ifdef CONFIG_MTD_W25
    case SPIDEV_FLASH(0):
      at32_gpiowrite(FLASH_SPI1_CS, !selected);
      break;
  #endif 

    default:
      break;
    }
}

uint8_t at32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  return status;
}
#endif

#ifdef CONFIG_AT32_SPI2
void at32_spi2select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
}

uint8_t at32_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_AT32_SPI3
void at32_spi3select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
}

uint8_t at32_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_AT32_SPI4
void at32_spi4select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
}

uint8_t at32_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: at32_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_AT32_SPI1
int at32_spi1cmddata(struct spi_dev_s *dev, uint32_t devid,
                      bool cmd)
{
  return -ENODEV;
}
#endif
#endif

#endif /* CONFIG_AT32_SPI1 || CONFIG_AT32_SPI2 */
