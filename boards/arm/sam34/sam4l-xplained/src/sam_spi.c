/****************************************************************************
 * boards/arm/sam34/sam4l-xplained/src/sam_spi.c
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

#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>

#include "sam_gpio.h"
#include "sam_spi.h"
#include "sam4l-xplained.h"

#ifdef CONFIG_SAM34_SPI0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM3U10E-EVAL
 *   board.
 *
 ****************************************************************************/

void weak_function sam_spidev_initialize(void)
{
  /* The I/O module containing the SD connector may or may not be installed.
   * And, if it is installed, it may be in connector EXT1 or EXT2.
   */

#ifdef CONFIG_SAM4L_XPLAINED_IOMODULE
  /* TODO: enable interrupt on card detect */

  sam_configgpio(GPIO_SD_CD);     /* Card detect input */
  sam_configgpio(GPIO_SD_CS);     /* Chip select output */
#endif

#ifdef CONFIG_SAM4L_XPLAINED_OLED1MODULE
  sam_configgpio(GPIO_OLED_DATA); /* Command/data */
  sam_configgpio(GPIO_OLED_CS);   /* Card detect input */
#endif
}

/****************************************************************************
 * Name:  sam_spi0select, sam_spi0status, and sam_spic0mddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   include:
 *
 *   o sam_spi0select is a functions tomanage the board-specific chip selects
 *   o sam_spi0status and sam_spic0mddata:  Implementations of the status
 *     and cmddata methods of the SPI interface defined by struct spi_ops_
 *     (see include/nuttx/spi/spi.h). All other methods including
 *     sam_spibus_initialize()) are provided by common SAM3/4 logic.
 *
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide sam_spi0select() and sam_spi0status() functions in your
 *      board specific logic. These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam_spic0mddata() functions in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to sam_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by sam_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spi0select
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
 *   devid - Identifies the (logical) device
 *   selected - TRUE:Select the device, FALSE:De-select the device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_spi0select(uint32_t devid, bool selected)
{
#ifdef CONFIG_SAM4L_XPLAINED_IOMODULE
  /* Select/de-select the SD card */

  if (devid == SPIDEV_MMCSD(0))
    {
      /* Active low */

      sam_gpiowrite(GPIO_SD_CS, !selected);
    }

#ifdef CONFIG_SAM4L_XPLAINED_OLED1MODULE
  else
#endif
#endif

#ifdef CONFIG_SAM4L_XPLAINED_OLED1MODULE
  /* Select/de-select the OLED */

  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Active low */

      sam_gpiowrite(GPIO_OLED_CS, !selected);
    }
#endif
}

/****************************************************************************
 * Name: sam_spi0status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h.
 *
 ****************************************************************************/

uint8_t sam_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

#ifdef CONFIG_SAM4L_XPLAINED_IOMODULE
  /* Check if an SD card is present in the microSD slot */

  if (devid == SPIDEV_MMCSD(0))
    {
      /* Active low */

      if (!sam_gpioread(GPIO_SD_CD))
        {
          ret |= SPI_STATUS_PRESENT;
        }
    }
#endif

  return ret;
}

#endif /* CONFIG_SAM34_SPI0 */

/****************************************************************************
 * Name: sam_spic0mddata
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
int sam_spic0mddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#ifdef CONFIG_SAM4L_XPLAINED_OLED1MODULE
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* This is the Data/Command control pad which determines whether the
       * data bits are data or a command.
       *
       * High: the inputs are treated as display data.
       * Low:  the inputs are transferred to the command registers.
       */

      sam_gpiowrite(GPIO_OLED_DATA, !cmd);
    }
#endif

      return OK;
}
#endif
