/****************************************************************************
 * boards/arm/samv7/same70-xplained/src/sam_spi.c
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
#include "same70-xplained.h"

#ifdef CONFIG_SAMV7_SPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select PIO pins for the SAME70-XPLD board.
 *
 ****************************************************************************/

void sam_spidev_initialize(void)
{
#ifdef CONFIG_SAMV7_SPI0_MASTER
#ifdef CONFIG_SAME70XPLAINED_MB1_SPI
  /* Enable chip select for mikroBUS1 */

  sam_configgpio(CLICK_MB1_CS);
#endif

#ifdef CONFIG_SAME70XPLAINED_MB2_SPI
  /* Enable chip select for mikroBUS2 */

  sam_configgpio(CLICK_MB2_CS);
#endif
#endif

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
 *                selects
 *   o sam_spi[0|1]status and sam_spi[0|1]cmddata:  Implementations of the
 *     status and cmddata methods of the SPI interface defined by struct
 *     spi_ops_ (see include/nuttx/spi/spi.h).
 *     All other methods including sam_spibus_initialize()) are provided by
 *     common SAM3/4 logic.
 *
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide sam_spi[0|1]select() and sam_spi[0|1]status() functions in
 *      your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using PIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam_spi[0|1]cmddata() functions in your board-specific logic.  This
 *      function will perform cmd/data selection operations using PIOs in
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
 *   PIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the PIO chip select pins is as a normal
 *   PIO output.  In that case, the automatic control of the CS pins is
 *   bypassed and this function must provide control of the chip select.
 *   NOTE:  In this case, the PIO output pin does *not* have to be the
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

#if defined(CONFIG_SAME70XPLAINED_MB1_BEE)
        sam_gpiowrite(CLICK_MB1_CS, !selected);
#elif defined(CONFIG_SAME70XPLAINED_MB2_BEE)
        sam_gpiowrite(CLICK_MB2_CS, !selected);
#endif
        break;
#endif

#ifdef CONFIG_IEEE802154_XBEE
      case SPIDEV_IEEE802154(0):

        /* Set the GPIO low to select and high to de-select */

#if defined(CONFIG_SAME70XPLAINED_MB1_XBEE)
        sam_gpiowrite(CLICK_MB1_CS, !selected);
#elif defined(CONFIG_SAME70XPLAINED_MB2_XBEE)
        sam_gpiowrite(CLICK_MB2_CS, !selected);
#endif
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
uint8_t sam_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_SAMV7_SPI1_MASTER
uint8_t sam_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#endif /* CONFIG_SAMV7_SPI */
