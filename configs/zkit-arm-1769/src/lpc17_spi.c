/************************************************************************************
 * configs/zkit-arm-1769/src/lpc17_spi.c
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Kannan <code@zilogic.com>
 *
 * Based on configs/lpcxpresso-lpc1768/src/up_ssp.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "lpc17_spi.h"
#include "lpc17_gpio.h"
#include "zkitarm_internal.h"

#if defined(CONFIG_LPC17_SPI)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* CONFIG_DEBUG_SPI enables debug output from this file (needs CONFIG_DEBUG too) */

#ifdef CONFIG_DEBUG_SPI
#  define spidbg  lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/* Dump GPIO registers */

#if defined(CONFIG_DEBUG_SPI) && defined(CONFIG_DEBUG_VERBOSE)
#  define spi_dumpgpio(m) lpc17_dumpgpio(SDCCS_GPIO, m)
#else
#  define spi_dumpgpio(m)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: zkit_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select and card detect GPIO pins for the
 *   ZKIT-ARM-1769 Kit.
 *
 ************************************************************************************/

void weak_function zkit_spiinitialize(void)
{
  /* Configure the SPI-based microSD CS  and Card Detect (CD) GPIO */

  spi_dumpgpio("zkit_spiinitialize() Entry)");

  /* Configure card detect and chip select for the SD slot. */

  (void)lpc17_configgpio(ZKITARM_SD_CS);
  (void)lpc17_configgpio(ZKITARM_SD_CD);

  spi_dumpgpio("zkit_spiinitialize() Exit");
}

/************************************************************************************
 * Name:  lpc17_spiselect and lpc17_spistatus
 *
 * Description:
 *   The external functions, lpc17_spiselect and  lpc17_spistatus
 *   must be provided by board-specific logic. They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common LPC17xx logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in lpc17_boardinitialize() to configure SPI/SSP chip select
 *      pins.
 *   2. Provide lpc17_spiselect and  lpc17_spistatus functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

void  lpc17_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  spi_dumpgpio("lpc17_spiselect() Entry");

  if (devid == SPIDEV_MMCSD)
    {
      /* Assert/de-assert the CS pin to the card */

      (void)lpc17_gpiowrite(ZKITARM_SD_CS, !selected);
    }

  spi_dumpgpio("lpc17_spiselect() Exit");
}

uint8_t lpc17_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  if (devid == SPIDEV_MMCSD)
    {
      /* Read the state of the card-detect bit */

      if (lpc17_gpioread(ZKITARM_SD_CD) == 0)
        {
          spidbg("Returning SPI_STATUS_PRESENT\n");
          return SPI_STATUS_PRESENT;
        }
    }

  spidbg("Returning zero\n");
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
/******************************************************************************
 * Name:  lpc17_spicmddata
 *
 * Description: Dummy Function
 *
 ******************************************************************************/

int lpc17_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
  return OK;
}

#endif /* CONFIG_SPI_CMDDATA */
#endif /* CONFIG_LPC17_SPI */
