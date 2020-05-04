/****************************************************************************
 * boards/arm/tiva/ekk-lm3s9b96/src/lm_ssi.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jose Pablo Rojas V. <jrojas@nx-engineering.com>
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
#include "tiva_gpio.h"
#include "ekk-lm3s9b96.h"

#if defined(CONFIG_TIVA_SSI0)  || defined(CONFIG_TIVA_SSI1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_SPI enables debug output from this file */

#ifdef CONFIG_DEBUG_SPI
#  define ssi_dumpgpio(m) tiva_dumpgpio(SDCCS_GPIO, m)
#else
#  define ssi_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm_ssidev_initialize
 *
 * Description:
 *   Called to configure chip select GPIO pins for the LM3S9B96 Eval board.
 *
 ****************************************************************************/

void weak_function lm_ssidev_initialize(void)
{
  /* Configure the CS GPIO */

#if 0
  ssi_dumpgpio("lm_ssidev_initialize() Entry");
  ssi_dumpgpio("lm_ssidev_initialize() Exit");
#endif
}

#if 0
/****************************************************************************
 * The external functions, tiva_ssiselect and tiva_ssistatus must be provided
 * by board-specific logic.  The are implementations of the select and status
 * methods SPI interface defined by struct spi_ops_s
 * (see include/nuttx/spi/spi.h).
 * All othermethods (including tiva_ssibus_initialize()) are provided by
 * common logic.  To use this common SPI logic on your board:
 *
 *   1. Provide tiva_ssiselect() and tiva_ssistatus() functions in your
 *      board-specific logic.  This function will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. Add a call to tiva_ssibus_initialize() in your low level
 *      initialization logic
 *   3. The handle returned by tiva_ssibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void tiva_ssiselect(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  ssi_dumpgpio("tiva_ssiselect() Entry");
  if (devid == SPIDEV_MMCSD(0))
    {
      /* Assert the CS pin to the card */

      tiva_gpiowrite(SDCCS_GPIO, !selected);
    }
#ifdef CONFIG_NX_LCDDRIVER
  else if (devid == SPIDEV_DISPLAY(0))
    {
      /* Assert the CS pin to the display */

      tiva_gpiowrite(OLEDCS_GPIO, !selected);
    }
#endif
  ssi_dumpgpio("tiva_ssiselect() Exit");
}

uint8_t tiva_ssistatus(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("Returning SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}
#endif
#endif /* CONFIG_TIVA_SSI0 || CONFIG_TIVA_SSI1 */
