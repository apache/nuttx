/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_spi.c
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
#include "sam4e-ek.h"

#if defined(CONFIG_SAM34_SPI0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM4E-EK board.
 *
 ****************************************************************************/

void weak_function sam_spidev_initialize(void)
{
#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_ADS7843E)
  /* The touchscreen connects using NPCS0 (PA11). */

  sam_configgpio(GPIO_TSC_CS);
#endif

#if defined(CONFIG_MTD_AT25)
  /* The AT25 Serial FLASH connects using NPCS3 (PA5). */

  sam_configgpio(GPIO_FLASH_CS);
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
  switch (devid)
    {
#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_ADS7843E)
      /* The touchscreen chip select is implemented as a GPIO OUTPUT that
       * must be controlled by this function.
       * This is because the ADS7843E driver must be able to sample the
       * device BUSY GPIO input between SPI transfers.
       * However, the AD7843E will tri-state the BUSY input whenever the
       * chip select is de-asserted.
       * So the only option is to control the chip select manually and hold
       * it low throughout the SPI transfer.
       */

      case SPIDEV_TOUCHSCREEN(0):
        sam_gpiowrite(GPIO_TSC_CS, !selected);
        break;
#endif

#if defined(CONFIG_MTD_AT25)
       /* The AT25 Serial FLASH connects using NPCS3 (PA5). */

      case SPIDEV_FLASH(0):
        sam_gpiowrite(GPIO_FLASH_CS, !selected);
        break;
#endif

      default:
        break;
    }
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
  return 0;
}

#endif /* CONFIG_SAM34_SPI0 || CONFIG_SAM34_SPI1 */
