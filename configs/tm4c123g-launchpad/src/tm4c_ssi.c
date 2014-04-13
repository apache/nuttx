/************************************************************************************
 * configs/tm4c123g-launchpad/src/tm4c_ssi.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#include "tiva_gpio.h"
#include "tm4c123g-launchpad.h"

/* The TM4C123G LaunchPad microSD CS is on SSI0 */

#if !defined(CONFIG_SSI0_DISABLE) || !defined(CONFIG_SSI1_DISABLE)

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* CONFIG_DEBUG_SPI enables debug output from this file (needs CONFIG_DEBUG too) */

#ifdef CONFIG_DEBUG_SPI
#  define ssidbg lldbg
#else
#  define ssidbg(x...)
#endif

/* Dump GPIO registers */

#if defined(CONFIG_DEBUG_SPI) && defined(CONFIG_DEBUG_VERBOSE)
#  define ssivdbg lldbg
#  define ssi_dumpgpio(m) tiva_dumpgpio(SDCCS_GPIO, m)
#else
#  define ssivdbg(x...)
#  define ssi_dumpgpio(m)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: tm4c_ssiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the TM4C123G LaunchPad.
 *
 ************************************************************************************/

void weak_function tm4c_ssiinitialize(void)
{
}

/****************************************************************************
 * The external functions, tiva_spiselect and tiva_spistatus must be provided
 * by board-specific logic.  The are implementations of the select and status
 * methods SPI interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 * All othermethods (including tiva_spiinitialize()) are provided by common
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide tiva_spiselect() and tiva_spistatus() functions in your
 *      board-specific logic.  This function will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. Add a call to tiva_spiinitialize() in your low level initialization
 *      logic
 *   3. The handle returned by tiva_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void tiva_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  ssidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  ssi_dumpgpio("tiva_spiselect() Entry");
  ssi_dumpgpio("tiva_spiselect() Exit");
}

uint8_t tiva_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  ssidbg("Returning SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}

#endif /* !CONFIG_SSI0_DISABLE || !CONFIG_SSI1_DISABLE */
