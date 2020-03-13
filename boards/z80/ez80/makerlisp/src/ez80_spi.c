/****************************************************************************
 * boards/z80/ez80/makerlisp/src/ez80_spi.c
 *
 *   Copyright (C) 2019 Greg Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <arch/chip/io.h>

#include "chip.h"
#include "z80_internal.h"
#include "ez80_spi.h"
#include "makerlisp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NOTE:  We are using a SDCard adapter/module without Card Detect pin!
 * Then we don't need to Card Detect callback here.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the MakerLisp board.
 *
 ****************************************************************************/

void ez80_spidev_initialize(void)
{
#ifdef HAVE_MMCSD
  uint8_t regval;

  /* MMC/SD CS: Port B pin 4 as output */

  regval  = inp(EZ80_PB_DR);
  regval |= EZ80_GPIOD4;
  outp(EZ80_PB_DR, regval);

  regval  = inp(EZ80_PB_ALT1);
  regval &= ~EZ80_GPIOD4;
  outp(EZ80_PB_ALT1, regval);

  regval  = inp(EZ80_PB_ALT2);
  regval &= ~EZ80_GPIOD4;
  outp(EZ80_PB_ALT2, regval);

  regval  = inp(EZ80_PB_DDR);
  regval &= ~EZ80_GPIOD4;
  outp(EZ80_PB_DDR, regval);
#endif
}

/****************************************************************************
 * The external functions, ez80_spiselect, ez80_spistatus, and
 * ez80_spicmddata must be provided by board-specific logic.  These are
 * implementations of the select, status, and cmddata methods of the SPI
 * interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).  All
 * other methods (including ez80_spibus_initialize()) are provided by common
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide ez80_spiselect() and ez80_spistatus() functions in your
 *      board-specific logic.  This function will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration, provide
 *      the ez80_spiscmddata() function in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to ez80_spibus_initialize() in your low level
 *      initialization logic
 *   4. The handle returned by ez80_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void ez80_spiselect(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
#ifdef HAVE_MMCSD
  if (devid == SPIDEV_MMCSD(0))
    {
      uint8_t regval;

      /* Set port B pin 4 output */

      regval  = inp(EZ80_PB_DR);

      if (selected)
        {
          regval &= ~EZ80_GPIOD4;
        }
      else
        {
          regval |= EZ80_GPIOD4;
        }

      outp(EZ80_PB_DR, regval);
    }
#endif
}

uint8_t ez80_spistatus(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

#ifdef HAVE_MMCSD
  if (devid == SPIDEV_MMCSD(0))
    {
      /* No card detect pin.. Always claim that the card is present in
       * slot 0
       */

       status |= SPI_STATUS_PRESENT;
    }
#endif

  return status;
}

int ez80_spicmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
