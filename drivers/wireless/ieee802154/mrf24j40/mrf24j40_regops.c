/****************************************************************************
 * drivers/wireless/ieee802154/mrf24j40/mrf24j40_regops.c
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

#include <assert.h>
#include <debug.h>
#include <inttypes.h>
#include <stdio.h>

#include "mrf24j40.h"
#include "mrf24j40_regops.h"

/****************************************************************************
 * Public Functions
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
      addr  &= 0x3f; /* 6-bit address */
      addr <<= 1;
      addr  |= 0x01; /* writing */
      buf[0] = addr;
      len    = 1;
    }
  else
    {
      addr  &= 0x3ff; /* 10-bit address */
      addr <<= 5;
      addr  |= 0x8010; /* writing long */
      buf[0] = (addr >>   8);
      buf[1] = (addr & 0xff);
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

      addr  &= 0x3f;
      addr <<= 1;
      buf[0] = addr;
      len    = 1;
    }
  else
    {
      /* 10-bit address */

      addr  &= 0x3ff;
      addr <<= 5;
      addr  |= 0x8000;
      buf[0] = (addr >>   8);
      buf[1] = (addr & 0xff);
      len    = 2;
    }

  buf[len++] = 0xff; /* dummy */

  mrf24j40_spi_lock(spi);
  SPI_SELECT     (spi, SPIDEV_IEEE802154(0), true);
  SPI_EXCHANGE   (spi, buf, rx, len);
  SPI_SELECT     (spi, SPIDEV_IEEE802154(0), false);
  mrf24j40_spi_unlock(spi);

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
  char buf[4 + 16 * 3 + 2 + 1];
  int len = 0;

  wlinfo("Short regs:\n");

  for (i = 0; i < 0x40; i++)
    {
      if ((i & 15) == 0)
        {
          snprintf(buf, sizeof(buf), "%02" PRIx32 ": ", i & 0xff);
          len = strlen(buf);
        }

      snprintf(buf + len, sizeof(buf) - len,
               "%02x ", mrf24j40_getreg(dev->spi, i));
      len += strlen(buf + len);
      if ((i & 15) == 15)
        {
          snprintf(buf + len, sizeof(buf) - len, "\n");
          wlinfo("%s", buf);
        }
    }

  wlinfo("Long regs:\n");

  for (i = 0x80000200; i < 0x80000250; i++)
    {
      if ((i & 15) == 0)
        {
          snprintf(buf, sizeof(buf), "%02" PRIx32 ": ", i & 0xff);
          len = strlen(buf);
        }

      snprintf(buf + len, sizeof(buf) - len,
               "%02x ", mrf24j40_getreg(dev->spi, i));
      len += strlen(buf + len);
      if ((i & 15) == 15)
        {
          snprintf(buf + len, sizeof(buf) - len, "\n");
          wlinfo("%s", buf);
        }
    }

  return 0;
}
