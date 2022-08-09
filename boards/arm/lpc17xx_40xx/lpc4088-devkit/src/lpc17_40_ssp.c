/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-devkit/src/lpc17_40_ssp.c
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

#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_ssp.h"
#include "lpc4088-devkit.h"

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1) || \
    defined(CONFIG_LPC17_40_SSP2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define ssp_dumpgpio(p,m) lpc17_40_dumpgpio(p,m)
#else
#  define ssp_dumpgpio(p,m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc4088_devkit_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPC1766-STK.
 *
 ****************************************************************************/

void weak_function lpc4088_devkit_sspdev_initialize(void)
{
  /* Configure the SSP0 chip select GPIOs. */

#ifdef CONFIG_LPC17_40_SSP0
#endif

  /* Configure SSP1 chip select GPIOs.  This includes the touchscreen on the
   * the LCD module.
   */

#ifdef CONFIG_LPC17_40_SSP1
  ssp_dumpgpio(GPIO_TC_CS, "BEFORE SSP1 Initialization");
  lpc17_40_configgpio(GPIO_TC_CS);
  ssp_dumpgpio(GPIO_TC_CS, "AFTER SSP1 Initialization");
#endif

  /* Configure the SSP2 chip select GPIOs. */

#ifdef CONFIG_LPC17_40_SSP2
#endif
}

/****************************************************************************
 * Name:  lpc17_40_ssp0/1/2select and lpc17_40_ssp0/1/2status
 *
 * Description:
 *   The external functions, lpc17_40_ssp0/1/2select and
 *   lpc17_40_ssp0/1/2status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including lpc17_40_sspbus_initialize())
 *   are provided by common LPC17xx/LPC40xx logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in lpc17_40_boardinitialize() to configure SPI/SSP chip
 *      select pins.
 *   2. Provide lpc17_40_ssp0/1/2select() and lpc17_40_ssp0/1/2status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to lpc17_40_sspbus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by lpc17_40_sspbus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SSP0
void  lpc17_40_ssp0select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t lpc17_40_ssp0status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}
#endif

#ifdef CONFIG_LPC17_40_SSP1
void  lpc17_40_ssp1select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  if (devid == SPIDEV_TOUCHSCREEN(0))
    {
      /* Assert/de-assert the CS pin to the touchscreen */

      ssp_dumpgpio(GPIO_TC_CS, "lpc17_40_ssp1select() Entry");
      lpc17_40_gpiowrite(GPIO_TC_CS, !selected);
      ssp_dumpgpio(GPIO_TC_CS, "lpc17_40_ssp1select() Exit");
    }
}

uint8_t lpc17_40_ssp1status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}
#endif

#ifdef CONFIG_LPC17_40_SSP2
void  lpc17_40_ssp2select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
}

uint8_t lpc17_40_ssp2status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning nothing\n");
  return 0;
}
#endif

#endif /* CONFIG_LPC17_40_SSP0 || CONFIG_LPC17_40_SSP1  || CONFIG_LPC17_40_SSP2 */
