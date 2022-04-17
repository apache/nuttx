/****************************************************************************
 * boards/arm/tms570/tms570ls31x-usb-kit/src/tms570_spi.c
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
#include "tms570_spi.h"
#include "tms570ls31x_usb_kit.h"

#if defined(CONFIG_TMS570_SPI1) || defined(CONFIG_TMS570_SPI4)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the HY-MiniTMS570
 *   board.
 *
 ****************************************************************************/

void tms570_spidev_initialize(void)
{
#ifdef CONFIG_MMCSD_SPI
#if defined (CONFIG_TMS570_SPI1)
    tms570_spi_gio_config(SPI1_SDCARD_CS);       /* SD/MMC Card chip select */
#endif
#if defined (CONFIG_TMS570_SPI4)
    tms570_spi_gio_config(SPI4_SDCARD_CS);
#endif
#endif
}

/****************************************************************************
 * Name:  tms570_spi1/2select and tms570_spi1/2status
 *
 * Description:
 *   The external functions, tms570_spi1/2/3select and tms570_spi1/2/3status
 *   must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including tms570_spibus_initialize())
 *   are provided by common TMS570 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in tms570_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide tms570_spi1/2/3select() and tms570_spi1/2/3status() functions
 *      in your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   3. Add a calls to tms570_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by tms570_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_TMS570_SPI1
void tms570_spi1select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
#ifdef CONFIG_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
      tms570_spi_giowrite(SPI1_SDCARD_CS, !selected);
    }
#endif
}

uint8_t tms570_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

#ifdef CONFIG_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
       status |= SPI_STATUS_PRESENT;
    }
#endif

  return status;
}
#endif

#ifdef CONFIG_TMS570_SPI4
void tms570_spi4select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
#ifdef CONFIG_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
      tms570_spi_giowrite(SPI4_SDCARD_CS, !selected);
    }
#endif
}

uint8_t tms570_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

#ifdef CONFIG_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
       status |= SPI_STATUS_PRESENT;
    }
#endif

  return status;
}
#endif
#endif /* CONFIG_TMS570_SPI1 || CONFIG_TMS570_SPI2 */
