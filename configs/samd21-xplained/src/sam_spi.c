/****************************************************************************
 * configs/samd21-xplained/src/sam_spi.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>

#include "sam_config.h"
#include "sam_port.h"
#include "sam_spi.h"

#include "samd21-xplained.h"

#ifdef SAMD2L2_HAVE_SPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select PORT pins for the SAMD21 Xplained
 *   Pro board.
 *
 ****************************************************************************/

void weak_function sam_spidev_initialize(void)
{
  /* The I/O module containing the SD connector may or may not be installed.  And, if
   * it is installed, it may be in connector EXT1 or EXT2.
   */

#ifdef CONFIG_SAMD21_XPLAINED_IOMODULE
  /* TODO: enable interrupt on card detect */

   sam_configport(PORT_SD_CD);     /* Card detect input */
   sam_configport(PORT_SD_CS);     /* Chip select output */
#endif

#ifdef CONFIG_SAMD21_XPLAINED_OLED1MODULE
   sam_configport(PORT_OLED_DATA); /* Command/data */
   sam_configport(PORT_OLED_CS);   /* Card detect input */
#endif
}

/****************************************************************************
 * Name:  sam_spi[n]select, sam_spi[n]status, and sam_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   include:
 *
 *   o sam_spi[n]select is a functions to manage the board-specific chip
 *     selects
 *   o sam_spi[n]status and sam_spi[n]cmddata:  Implementations of the status
 *     and cmddata methods of the SPI interface defined by struct spi_ops_
 *     (see include/nuttx/spi/spi.h). All other methods including
 *     sam_spibus_initialize()) are provided by common SAMD/L logic.
 *
 *   Where [n] is the SERCOM number for the SPI module.
 *
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide sam_spi[n]select() and sam_spi[n]status() functions in your
 *      board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam_spi[n]cmddata() functions in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to sam_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by sam_spibus_initialize() may then be used to bind
 *      the  SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spi[n]select
 *
 * Description:
 *   PIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the PIO chip select pins is as a normal
 *   GPIO output.  In that case, the automatic control of the CS pins is
 *   bypassed and this function must provide control of the chip select.
 *   NOTE:  In this case, the GPIO output pin does *not* have to be the
 *   same as the NPCS pin normal associated with the chip select number.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *   selected - TRUE:Select the device, FALSE:De-select the device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SAMD2L2_HAVE_SPI0
void sam_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
#ifdef CONFIG_SAMD21_XPLAINED_IOMODULE_EXT1
  /* Select/de-select the SD card */

  if (devid == SPIDEV_MMCSD(0))
    {
      /* Active low */

      sam_portwrite(PORT_SD_CS, !selected);
    }
#endif

#ifdef CONFIG_SAMD21_XPLAINED_OLED1MODULE_EXT1
  /* Select/de-select the OLED */

  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Active low */

      sam_portwrite(PORT_OLED_CS, !selected);
    }
#endif
}
#endif

#ifdef SAMD2L2_HAVE_SPI1
void sam_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
#ifdef CONFIG_SAMD21_XPLAINED_IOMODULE_EXT2
  /* Select/de-select the SD card */

  if (devid == SPIDEV_MMCSD(0))
    {
      /* Active low */

      sam_portwrite(PORT_SD_CS, !selected);
    }
#endif

#ifdef CONFIG_SAMD21_XPLAINED_OLED1MODULE_EXT2
  /* Select/de-select the OLED */

  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Active low */

      sam_portwrite(PORT_OLED_CS, !selected);
    }
#endif
}
#endif

#ifdef SAMD2L2_HAVE_SPI2
void sam_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

#ifdef SAMD2L2_HAVE_SPI3
void sam_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

#ifdef SAMD2L2_HAVE_SPI4
void sam_spi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

#ifdef SAMD2L2_HAVE_SPI5
void sam_spi5select(FAR struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

/****************************************************************************
 * Name: sam_spi[n]status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h.
 *
 ****************************************************************************/

#ifdef SAMD2L2_HAVE_SPI0
uint8_t sam_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

#ifdef CONFIG_SAMD21_XPLAINED_IOMODULE_EXT1
  /* Check if an SD card is present in the microSD slot */

  if (devid == SPIDEV_MMCSD(0))
    {
      /* Active low */

      if (!sam_portread(PORT_SD_CD))
        {
          ret |= SPI_STATUS_PRESENT;
        }
    }
#endif

  return ret;
}
#endif

#ifdef SAMD2L2_HAVE_SPI1
uint8_t sam_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

#ifdef CONFIG_SAMD21_XPLAINED_IOMODULE_EXT2
  /* Check if an SD card is present in the microSD slot */

  if (devid == SPIDEV_MMCSD(0))
    {
      /* Active low */

      if (!sam_portread(PORT_SD_CD))
        {
          ret |= SPI_STATUS_PRESENT;
        }
    }
#endif

  return ret;
}
#endif

#ifdef SAMD2L2_HAVE_SPI2
uint8_t sam_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef SAMD2L2_HAVE_SPI3
uint8_t sam_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef SAMD2L2_HAVE_SPI4
uint8_t sam_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef SAMD2L2_HAVE_SPI5
uint8_t sam_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;
  return ret;
}
#endif

/****************************************************************************
 * Name: sam_spi[n]cmddata
 *
 * Description:
 *   Some SPI devices require an additional control to determine if the SPI
 *   data being sent is a command or is data.  If CONFIG_SPI_CMDDATA then
 *   this function will be called to different be command and data transfers.
 *
 *   This is often needed, for example, by LCD drivers.  Some LCD hardware
 *   may be configured to use 9-bit data transfers with the 9th bit
 *   indicating command or data.  That same hardware may be configurable,
 *   instead, to use 8-bit data but to require an additional, board-
 *   specific GPIO control to distinguish command and data.  This function
 *   would be needed in that latter case.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef SAMD2L2_HAVE_SPI0
int sam_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#ifdef CONFIG_SAMD21_XPLAINED_OLED1MODULE_EXT1
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* This is the Data/Command control pad which determines whether the
       * data bits are data or a command.
       *
       * High: the inputs are treated as display data.
       * Low:  the inputs are transferred to the command registers.
       */

      (void)sam_portwrite(PORT_OLED_DATA, !cmd);
    }
#endif

  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI1
int sam_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#ifdef CONFIG_SAMD21_XPLAINED_OLED1MODULE_EXT2
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* This is the Data/Command control pad which determines whether the
       * data bits are data or a command.
       *
       * High: the inputs are treated as display data.
       * Low:  the inputs are transferred to the command registers.
       */

      (void)sam_portwrite(PORT_OLED_DATA, !cmd);
    }
#endif

  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI2
int sam_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI3
int sam_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI4
int sam_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI5
int sam_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#endif /* CONFIG_SPI_CMDDATA */
#endif /* SAMD2L2_HAVE_SPI */
