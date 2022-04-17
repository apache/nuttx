/****************************************************************************
 * boards/arm/samv7/samv71-xult/src/sam_spi.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "sam_gpio.h"
#include "sam_spi.h"
#include "samv71-xult.h"

#ifdef CONFIG_SAMV7_SPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAMV71-XULT board.
 *
 ****************************************************************************/

void sam_spidev_initialize(void)
{
#ifdef CONFIG_SAMV7_SPI0_MASTER
  /* Make sure that the EDBG DIGI_SPI CS is high so that it does not
   * interfere
   */

  sam_configgpio(CLICK_EDBG_CS);

#ifdef CONFIG_SAMV71XULT_MB1_SPI
  /* Enable chip select for mikroBUS1 */

  sam_configgpio(CLICK_MB1_CS);
#endif

#ifdef CONFIG_SAMV71XULT_MB2_SPI
  /* Enable chip select for mikroBUS2 */

  sam_configgpio(CLICK_MB2_CS);

#endif

#ifdef CONFIG_LCD_ST7789
  /* Enable CS and CMD/DATA for LCD */

  sam_configgpio(SPI0_NPCS1);
  sam_configgpio(GPIO_LCD_CD);
#endif

#endif /* CONFIG_SAMV7_SPI0_MASTER */

#ifdef CONFIG_SAMV7_SPI0_SLAVE
#endif

#ifdef CONFIG_SAMV7_SPI1_MASTER
#endif

#ifdef CONFIG_SAMV7_SPI1_SLAVE
#endif
}

/****************************************************************************
 * Name:  sam_spi[0|1]select, sam_spi[0|1]status, and sam_spi[0|1]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They include:
 *
 *   o sam_spi[0|1]select is a functions tomanage the board-specific chip
 *                 selects
 *   o sam_spi[0|1]status and sam_spi[0|1]cmddata:  Implementations of the
 *     status and cmddata methods of the SPI interface defined by struct
 *     spi_ops_ (see include/nuttx/spi/spi.h).
 *     All other methods including sam_spibus_initialize()) are
 *     provided by common SAM3/4 logic.
 *
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide sam_spi[0|1]select() and sam_spi[0|1]status() functions in
 *      your board- specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board
 *      is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam_spi[0|1]cmddata() functions in your board-specific logic.  This
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
 * Name: sam_spi[0|1]select
 *
 * Description:
 *   GPIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the GPIO chip select pins is as a normal
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

#ifdef CONFIG_SAMV7_SPI0_MASTER
void sam_spi0select(uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  switch (devid)
    {
#ifdef CONFIG_IEEE802154_MRF24J40
      case SPIDEV_IEEE802154(0):

        /* Set the GPIO low to select and high to de-select */

#if defined(CONFIG_SAMV71XULT_MB1_BEE)
        sam_gpiowrite(CLICK_MB1_CS, !selected);
#elif defined(CONFIG_SAMV71XULT_MB2_BEE)
        sam_gpiowrite(CLICK_MB2_CS, !selected);
#endif
        break;
#endif

#ifdef CONFIG_LCD_ST7789
      case SPIDEV_DISPLAY(0):
        sam_gpiowrite(SPI0_NPCS1, !selected);
        break;
#endif

      default:
        break;
    }
}
#endif

#ifdef CONFIG_SAMV7_SPI1_MASTER
void sam_spi1select(uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}
#endif

/****************************************************************************
 * Name: sam_spi[0|1]status
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

#ifdef CONFIG_SAMV7_SPI0_MASTER
uint8_t sam_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_SAMV7_SPI1_MASTER
uint8_t sam_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
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
#ifdef CONFIG_SAMV7_SPI0_MASTER
int sam_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      sam_gpiowrite(GPIO_LCD_CD, !cmd);
      return OK;
    }
  else
    {
      return -ENODEV;
    }
}
#endif /* CONFIG_SAMV7_SPI0_MASTER */
#endif /* CONFIG_SPI_CMDDATA */

#endif /* CONFIG_SAMV7_SPI */
