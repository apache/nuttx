/****************************************************************************
 * boards/arm/lpc17xx_40xx/zkit-arm-1769/src/lpc17_40_spi.c
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Kannan <code@zilogic.com>
 *
 * Based on boards/lpcxpresso-lpc1768/src/up_ssp.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "lpc17_40_spi.h"
#include "lpc17_40_gpio.h"
#include "zkit-arm-1769.h"

#if defined(CONFIG_LPC17_40_SPI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_SPI_INFO
#  define spi_dumpgpio(m) lpc17_40_dumpgpio(SDCCS_GPIO, m)
#else
#  define spi_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: zkit_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select and card detect GPIO pins for the
 *   ZKIT-ARM-1769 Kit.
 *
 ****************************************************************************/

void weak_function zkit_spidev_initialize(void)
{
  /* Configure the SPI-based microSD CS  and Card Detect (CD) GPIO */

  spi_dumpgpio("zkit_spidev_initialize() Entry)");

  /* Configure card detect and chip select for the SD slot. */

  lpc17_40_configgpio(ZKITARM_SD_CS);
  lpc17_40_configgpio(ZKITARM_SD_CD);

  spi_dumpgpio("zkit_spidev_initialize() Exit");
}

/****************************************************************************
 * Name:  lpc17_40_spiselect and lpc17_40_spistatus
 *
 * Description:
 *   The external functions, lpc17_40_spiselect and  lpc17_40_spistatus
 *   must be provided by board-specific logic. They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including lpc17_40_spibus_initialize())
 *   are provided by common LPC17xx/LPC40xx logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in lpc17_40_boardinitialize() to configure SPI/SSP
 *      chip select pins.
 *   2. Provide lpc17_40_spiselect and  lpc17_40_spistatus functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to lpc17_40_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by lpc17_40_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void  lpc17_40_spiselect(FAR struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  spi_dumpgpio("lpc17_40_spiselect() Entry");

  if (devid == SPIDEV_MMCSD(0))
    {
      /* Assert/de-assert the CS pin to the card */

      lpc17_40_gpiowrite(ZKITARM_SD_CS, !selected);
    }

  spi_dumpgpio("lpc17_40_spiselect() Exit");
}

uint8_t lpc17_40_spistatus(FAR struct spi_dev_s *dev, uint32_t devid)
{
  if (devid == SPIDEV_MMCSD(0))
    {
      /* Read the state of the card-detect bit */

      if (lpc17_40_gpioread(ZKITARM_SD_CD) == 0)
        {
          spiinfo("Returning SPI_STATUS_PRESENT\n");
          return SPI_STATUS_PRESENT;
        }
    }

  spiinfo("Returning zero\n");
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
/****************************************************************************
 * Name:  lpc17_40_spicmddata
 *
 * Description: Dummy Function
 *
 ****************************************************************************/

int lpc17_40_spicmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}

#endif /* CONFIG_SPI_CMDDATA */
#endif /* CONFIG_LPC17_40_SPI */
