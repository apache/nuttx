/************************************************************************************
 * configs/flipnclick-sam3x/src/sam_spi0.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "sam_gpio.h"
#include "sam_spi.h"

#include "flipnclick-sam3x.h"

#ifdef CONFIG_SAM34_SPI0

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM4E-EK board.
 *
 ************************************************************************************/

void weak_function sam_spidev_initialize(void)
{
#ifdef HAVE_SSD1306
  /* Configure the SSD1306 chip select and command/data GPIOs */

  sam_configgpio(GPIO_SSD1306_CS);
  sam_configgpio(GPIO_SSD1306_DC);
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
 *   2. Provide sam_spi0select() and sam_spi0status() functions in your board-
 *      specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam_spic0mddata() functions in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to sam_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by sam_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
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
  spiinfo("devid: %08x CS: %s\n",
          (unsigned int)devid, selected ? "assert" : "de-assert");

#ifdef HAVE_SSD1306
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Low:  the display is selected
       * High: the display is deselected
       */

      sam_gpiowrite(GPIO_SSD1306_CS, !selected);
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

uint8_t sam_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}
/****************************************************************************
 * Name: sam_spi0cmddata
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
 *   specific PIO control to distinguish command and data.  This function
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
int sam_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  spiinfo("devid: %08x %s\n", (unsigned int)devid, cmd ? "cmd" : "data");

#ifdef HAVE_SSD1306
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* High: the inputs are treated as display data.
       * Low:  the inputs are transferred to the command registers.
       */

      sam_gpiowrite(GPIO_SSD1306_DC, !cmd);
    }
#endif

  return 0;

}
#endif

#endif /* CONFIG_SAM34_SPI0 */
