/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_regops.c
 *
 *   Copyright (C) 2015-2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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

#include <assert.h>
#include <debug.h>
#include <stdio.h>

#include "mrf24j40.h"
#include "mrf24j40_regops.h"

/****************************************************************************
 * Internal Driver Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mrf24j40_setreg
 *
 * Description:
 *   Define the value of an MRF24J40 device register
 *
 ****************************************************************************/

void mrf24j40_setreg(FAR struct spi_dev_s *spi, uint32_t addr, uint8_t val)
{
  uint8_t buf[3];
  int     len;

  if (!(addr & 0x80000000))
    {
      addr  &= 0x3F; /* 6-bit address */
      addr <<= 1;
      addr  |= 0x01; /* writing */
      buf[0] = addr;
      len    = 1;
    }
  else
    {
      addr  &= 0x3FF; /* 10-bit address */
      addr <<= 5;
      addr  |= 0x8010; /* writing long */
      buf[0] = (addr >>   8);
      buf[1] = (addr & 0xFF);
      len    = 2;
    }

  buf[len++] = val;

  mrf24j40_spi_lock(spi);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), true);
  SPI_SNDBLOCK(spi, buf, len);
  SPI_SELECT(spi, SPIDEV_IEEE802154(0), false);
  mrf24j40_spi_unlock(spi);
}

/****************************************************************************
 * Name: mrf24j40_getreg
 *
 * Description:
 *   Return the value of an MRF24J40 device register*
 *
 ****************************************************************************/

uint8_t mrf24j40_getreg(FAR struct spi_dev_s *spi, uint32_t addr)
{
  uint8_t buf[3];
  uint8_t rx[3];
  int     len;

  if (!(addr & 0x80000000))
    {
      /* 6-bit address */

      addr  &= 0x3F;
      addr <<= 1;
      buf[0] = addr;
      len    = 1;
    }
  else
    {
      /* 10-bit address */

      addr  &= 0x3FF;
      addr <<= 5;
      addr  |= 0x8000;
      buf[0] = (addr >>   8);
      buf[1] = (addr & 0xFF);
      len    = 2;
    }

  buf[len++] = 0xFF; /* dummy */

  mrf24j40_spi_lock(spi);
  SPI_SELECT     (spi, SPIDEV_IEEE802154(0), true);
  SPI_EXCHANGE   (spi, buf, rx, len);
  SPI_SELECT     (spi, SPIDEV_IEEE802154(0), false);
  mrf24j40_spi_unlock(spi);

  /* wlinfo("r[%04X]=%02X\n", addr, rx[len - 1]); */
  return rx[len - 1];
}

/****************************************************************************
 * Name: mrf24j40_regdump
 *
 * Description:
 *   Display the value of all registers.
 *
 ****************************************************************************/

int mrf24j40_regdump(FAR struct mrf24j40_radio_s *dev)
{
  uint32_t i;
  char buf[4+16*3+2+1];
  int len = 0;

  wlinfo("Short regs:\n");

  for (i = 0; i < 0x40; i++)
    {
      if ((i & 15) == 0)
        {
          len=sprintf(buf, "%02x: ", i & 0xFF);
        }

      len += sprintf(buf+len, "%02x ", mrf24j40_getreg(dev->spi, i));
      if ((i & 15) == 15)
        {
          sprintf(buf+len, "\n");
          wlinfo("%s", buf);
        }
    }

  wlinfo("Long regs:\n");

  for (i = 0x80000200; i < 0x80000250; i++)
    {
      if ((i & 15) == 0)
        {
          len=sprintf(buf, "%02x: ", i & 0xFF);
        }

      len += sprintf(buf+len, "%02x ", mrf24j40_getreg(dev->spi, i));
      if ((i & 15) == 15)
        {
          sprintf(buf+len, "\n");
          wlinfo("%s", buf);
        }
    }

  return 0;
}
