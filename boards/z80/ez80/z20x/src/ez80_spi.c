/****************************************************************************
 * boards/z80/ez80/z20x/src/ez80_spi.c
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

#include <errno.h>

#include <arch/chip/io.h>

#include "chip.h"
#include "z80_internal.h"
#include "ez80_spi.h"
#include "z20x.h"

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
 *   Called to configure SPI chip select GPIO pins for the z20x board.
 *
 ****************************************************************************/

void ez80_spidev_initialize(void)
{
#if defined(HAVE_SPIFLASH) || defined(HAVE_MMCSD) || defined(HAVE_XPT2046)
  uint8_t regval;
  uint8_t pins;

  /* SPI Devices on the z20x main board:
   *
   *   W25Q32JV CS: Port PB5 as output
   *
   * SPI Devices on LCD card:
   *
   *   MMC/SD CS: Port PB2/nSS as output
   *   XPT2046 CS: PB1/T1_IN as output
   *   SST25VF016 CS: Pulled high
   */

  pins    = 0;
#ifdef HAVE_SPIFLASH
  pins   |= EZ80_GPIOD5;
#endif
#ifdef HAVE_XPT2046
  pins   |= EZ80_GPIOD1;
#endif
#ifdef HAVE_MMCSD
  pins   |= EZ80_GPIOD2;
#endif

  regval  = inp(EZ80_PB_DR);
  regval |= pins;
  outp(EZ80_PB_DR, regval);

  regval  = inp(EZ80_PB_ALT1);
  regval &= ~pins;
  outp(EZ80_PB_ALT1, regval);

  regval  = inp(EZ80_PB_ALT2);
  regval &= ~pins;
  outp(EZ80_PB_ALT2, regval);

  regval  = inp(EZ80_PB_DDR);
  regval &= ~pins;
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
#ifdef HAVE_SPIFLASH
  if (devid == SPIDEV_FLASH(0))
    {
      uint8_t regval;

      /* Set PB5 output */

      regval  = inp(EZ80_PB_DR);

      if (selected)
        {
          regval &= ~EZ80_GPIOD5;
        }
      else
        {
          regval |= EZ80_GPIOD5;
        }

      outp(EZ80_PB_DR, regval);
      return;
    }
#endif

#ifdef HAVE_XPT2046
  if (devid == SPIDEV_TOUCHSCREEN(0))
    {
      uint8_t regval;

      /* Set PB1/T1_IN output */

      regval  = inp(EZ80_PB_DR);

      if (selected)
        {
          regval &= ~EZ80_GPIOD1;
        }
      else
        {
          regval |= EZ80_GPIOD1;
        }

      outp(EZ80_PB_DR, regval);
      return;
    }
#endif

#ifdef HAVE_MMCSD
  if (devid == SPIDEV_MMCSD(0))
    {
      uint8_t regval;

      /* Set PB2/nSS output */

      regval  = inp(EZ80_PB_DR);

      if (selected)
        {
          regval &= ~EZ80_GPIOD2;
        }
      else
        {
          regval |= EZ80_GPIOD2;
        }

      outp(EZ80_PB_DR, regval);
      return;
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
