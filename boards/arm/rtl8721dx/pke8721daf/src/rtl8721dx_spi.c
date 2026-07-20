/****************************************************************************
 * boards/arm/rtl8721dx/pke8721daf/src/rtl8721dx_spi.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/param.h>
#include <syslog.h>
#include <errno.h>

#include "ameba_gpio.h"
#include "ameba_spi.h"
#include "rtl8721dx_pke8721daf.h"

#ifdef CONFIG_AMEBA_SPI

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One entry per SPI bus exposed to NuttX at /dev/spiN.  The CLK/MOSI/MISO/CS
 * pads below are examples used by the `spi` config (system/spi spitool);
 * adjust them to match your board's wiring.
 */

struct rtl8721dx_spi_s
{
  int     bus;                  /* Controller index (AMEBA_SPI0/AMEBA_SPI1) */
  uint8_t clkpin;               /* SCLK pad (AMEBA_PA()/AMEBA_PB() encoding) */
  uint8_t mosipin;              /* MOSI pad */
  uint8_t misopin;              /* MISO pad */
  uint8_t cspin;                /* Chip-select pad (software CS GPIO) */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtl8721dx_spi_s g_spi_buses[] =
{
  {
    AMEBA_SPI0, AMEBA_PA(14), AMEBA_PA(15), AMEBA_PA(16), AMEBA_PA(17)
  },
  {
    AMEBA_SPI1, AMEBA_PB(18), AMEBA_PB(19), AMEBA_PB(20), AMEBA_PB(21)
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8721dx_spi_initialize
 *
 * Description:
 *   Register the board's SPI master buses at /dev/spiN.
 *
 ****************************************************************************/

int rtl8721dx_spi_initialize(void)
{
  int i;

  for (i = 0; i < (int)nitems(g_spi_buses); i++)
    {
      if (ameba_spi_register(g_spi_buses[i].bus, g_spi_buses[i].clkpin,
                             g_spi_buses[i].mosipin, g_spi_buses[i].misopin,
                             g_spi_buses[i].cspin) == NULL)
        {
          syslog(LOG_ERR,
                 "ERROR: ameba_spi_register(/dev/spi%d) failed\n",
                 g_spi_buses[i].bus);
          return -ENODEV;
        }
    }

  return OK;
}

#endif /* CONFIG_AMEBA_SPI */
