/****************************************************************************
 * boards/arm/lpc17xx_40xx/zkit-arm-1769/src/lpc17_40_ssp.c
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

#include "arm_arch.h"
#include "chip.h"
#include "lpc17_40_ssp.h"
#include "lpc17_40_gpio.h"
#include "zkit-arm-1769.h"

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_SPI_INFO
#  define ssp_dumpgpio(m) lpc17_40_dumpgpio(SDCCS_GPIO, m)
#else
#  define ssp_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: zkit_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the ZKIT-ARM-1769 Kit.
 *
 ****************************************************************************/

void weak_function zkit_sspdev_initialize(void)
{
  /* Configure the SPI-based LCD CS GPIO */

  ssp_dumpgpio("zkit_sspdev_initialize() Entry)");

  /* Configure chip select for the LCD. */

#ifdef CONFIG_LPC17_40_SSP0
#ifdef CONFIG_NX_LCDDRIVER
  lpc17_40_configgpio(ZKITARM_OLED_CS);
#endif
#endif /* CONFIG_LPC17_40_SSP0 */

  ssp_dumpgpio("zkit_sspdev_initialize() Exit");
}

/****************************************************************************
 * Name:  lpc17_40_ssp0/ssp1select and lpc17_40_ssp0/ssp1status
 *
 * Description:
 *   The external functions, lpc17_40_ssp0/ssp1select and
 *   lpc17_40_ssp0/ssp1status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including lpc17_40_sspbus_initialize())
 *   are provided by common LPC17xx/LPC40xx logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in lpc17_40_boardinitialize() to configure SPI/SSP chip
 *      select pins.
 *   2. Provide lpc17_40_ssp0/ssp1select() and lpc17_40_ssp0/ssp1status()
 *      functions in your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   3. Add a calls to lpc17_40_sspbus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by lpc17_40_sspbus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_SSP1
void  lpc17_40_ssp1select(FAR struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  ssp_dumpgpio("lpc17_40_ssp1select() Entry");

#warning "Assert CS here (false)"

  ssp_dumpgpio("lpc17_40_ssp1select() Exit");
}

uint8_t lpc17_40_ssp1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name:  lpc17_40_ssp1cmddata
 *
 * Description: Dummy Function
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
int weak_function lpc17_40_ssp1cmddata(FAR struct spi_dev_s *dev,
                                    uint32_t devid, bool cmd)
{
  return OK;
}

#endif /* CONFIG_SPI_CMDDATA */
#endif /* CONFIG_LPC17_40_SSP1 */

#ifdef CONFIG_LPC17_40_SSP0
void  lpc17_40_ssp0select(FAR struct spi_dev_s *dev, uint32_t devid,
                          bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  ssp_dumpgpio("lpc17_40_ssp0select() Entry");

#ifdef CONFIG_NX_LCDDRIVER
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Assert the CS pin to the OLED display */

      lpc17_40_gpiowrite(ZKITARM_OLED_CS, !selected);
    }
#endif

  ssp_dumpgpio("lpc17_40_ssp0select() Exit");
}

uint8_t lpc17_40_ssp0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      spiinfo("Returning SPI_STATUS_PRESENT\n");
      return SPI_STATUS_PRESENT;
    }

  spiinfo("Returning zero\n");
  return 0;
}

#endif /* CONFIG_LPC17_40_SSP0 */
#endif /* CONFIG_LPC17_40_SSP0 || CONFIG_LPC17_40_SSP1 */
