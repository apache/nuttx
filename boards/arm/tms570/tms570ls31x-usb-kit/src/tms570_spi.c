/****************************************************************************
 * boards/arm/tms570/tms570ls31x-usb-kit/src/tms570_spi.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Laurent Latil <laurent@latil.nom.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "arm_arch.h"
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
void tms570_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
#ifdef CONFIG_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
      tms570_spi_giowrite(SPI1_SDCARD_CS, !selected);
    }
#endif
}

uint8_t tms570_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
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
void tms570_spi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
#ifdef CONFIG_MMCSD_SPI
  if (devid == SPIDEV_MMCSD(0))
    {
      tms570_spi_giowrite(SPI4_SDCARD_CS, !selected);
    }
#endif
}

uint8_t tms570_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
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
