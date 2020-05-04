/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_spi.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Michael Jung <mijung@gmx.net>
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
#include "pnev5180b.h"

#if defined(CONFIG_LPC17_40_SPI)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pnev5180b_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select and card detect GPIO pins for the
 *   NXP PNEV5180B NFC Frontend Development Kit.
 *
 ****************************************************************************/

void weak_function pnev5180b_spidev_initialize(void)
{
}

/****************************************************************************
 * Name:  lpc17_40_spiselect and lpc17_40_spistatus
 *
 * Description:
 *   The external functions, lpc17_40_spiselect and  lpc17_40_spistatus
 *   must be provided by board-specific logic. They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   lpc17_40_spibus_initialize()) are provided by common LPC17xx/LPC40xx
 *   logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in lpc17_40_boardinitialize() to configure SPI/SSP chip
 *      select pins.
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

void lpc17_40_spiselect(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");
}

uint8_t lpc17_40_spistatus(FAR struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("devid: %d\n", (int)devid);
  return 0;
}

#endif /* CONFIG_LPC17_40_SPI */
