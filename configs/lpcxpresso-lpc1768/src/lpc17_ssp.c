/************************************************************************************
 * configs/lpcxpresso-lpc1768/src/lpc17_ssp.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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
#include "lpc17_gpio.h"
#include "lpc17_ssp.h"
#include "lpcxpresso-lpc1768.h"

#if defined(CONFIG_LPC17_SSP0) || defined(CONFIG_LPC17_SSP1)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_GPIO_INFO
#  define ssp_dumpgpio(m) lpc17_dumpgpio(SDCCS_GPIO, m)
#else
#  define ssp_dumpgpio(m)
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpcxpresso_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPCXpresso.
 *
 ************************************************************************************/

void weak_function lpcxpresso_sspdev_initialize(void)
{
  /* Configure the SPI-based microSD CS GPIO */

  ssp_dumpgpio("lpcxpresso_sspdev_initialize() Entry)");

  /* Configure card detect and chip select for the SD slot.  NOTE:  Jumper J55 must
   * be set correctly for the SD slot chip select.
   */

#ifdef CONFIG_LPC17_SSP1
  (void)lpc17_configgpio(LPCXPRESSO_SD_CS);
  (void)lpc17_configgpio(LPCXPRESSO_SD_CD);

  /* Configure chip select for the OLED. For the SPI interface, insert jumpers in
   * J42, J43, J45 pin1-2 and J46 pin 1-2.
   */

#ifdef CONFIG_NX_LCDDRIVER
  (void)lpc17_configgpio(LPCXPRESSO_OLED_CS);
#endif
#endif

  ssp_dumpgpio("lpcxpresso_sspdev_initialize() Exit");
}

/************************************************************************************
 * Name:  lpc17_ssp0/ssp1select and lpc17_ssp0/ssp1status
 *
 * Description:
 *   The external functions, lpc17_ssp0/ssp1select and lpc17_ssp0/ssp1status
 *   must be provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including lpc17_sspbus_initialize())
 *   are provided by common LPC17xx logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in lpc17_boardinitialize() to configure SPI/SSP chip select
 *      pins.
 *   2. Provide lpc17_ssp0/ssp1select() and lpc17_ssp0/ssp1status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to lpc17_sspbus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by lpc17_sspbus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_SSP0
void  lpc17_ssp0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  ssp_dumpgpio("lpc17_ssp0select() Entry");

#warning "Assert CS here (false)"

  ssp_dumpgpio("lpc17_ssp0select() Exit");
}

uint8_t lpc17_ssp0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_LPC17_SSP1
void  lpc17_ssp1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  ssp_dumpgpio("lpc17_ssp1select() Entry");

  if (devid == SPIDEV_MMCSD(0))
    {
      /* Assert/de-assert the CS pin to the card */

      (void)lpc17_gpiowrite(LPCXPRESSO_SD_CS, !selected);
    }
#ifdef CONFIG_NX_LCDDRIVER
  else if (devid == SPIDEV_DISPLAY(0))
    {
      /* Assert the CS pin to the OLED display */

      (void)lpc17_gpiowrite(LPCXPRESSO_OLED_CS, !selected);
    }
#endif
  ssp_dumpgpio("lpc17_ssp1select() Exit");
}

uint8_t lpc17_ssp1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  if (devid == SPIDEV_MMCSD(0))
    {
      /* Read the state of the card-detect bit */

      if (lpc17_gpioread(LPCXPRESSO_SD_CD) == 0)
        {
          spiinfo("Returning SPI_STATUS_PRESENT\n");
          return SPI_STATUS_PRESENT;
        }
    }

  spiinfo("Returning zero\n");
  return 0;
}
#endif

#endif /* CONFIG_LPC17_SSP0 || CONFIG_LPC17_SSP1 */
