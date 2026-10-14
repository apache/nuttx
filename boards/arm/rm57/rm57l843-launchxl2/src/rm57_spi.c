/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_spi.c
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

/* SPI on the LAUNCHXL2-RM57L.
 *
 * Two MibSPI modules reach the BoosterPack headers complete
 * (docs/sprr397.pdf sheets 7 and 8):
 *
 *   MibSPI1, BoosterPack site 2      MibSPI3, BoosterPack site 1
 *     SPICLK    J5.7                   SPICLK    J1.7
 *     SIMO[0]   J6.6                   SIMO      J2.6
 *     SOMI[0]   J6.7                   SOMI      J2.7
 *     nCS[0]    J6.3                   nCS[0]    J5.5 (on site 2)
 *     nCS[1]    J6.8                   nCS[1]    J2.3
 *     nCS[2]    J6.9                   nCS[2]    J2.8
 *                                      nCS[3]    J2.9
 *
 * Nothing on the board itself is an SPI device.  A device is selected by
 * the index part of its devid: index n drives nCS[n] of that bus, as a
 * plain GIO output - see the note on chip selects in rm57_mibspi.h.  With
 * I2C1 enabled, MibSPI3 nCS[2] and nCS[3] become I2C1 SDA and SCL and are
 * not claimed here.
 *
 * MibSPI2, MibSPI4 and MibSPI5 can be enabled and are registered below, but
 * this file claims no chip select for them and board.h routes none of the
 * MibSPI2/MibSPI4 balls, so they are only of use once both are added.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "rm57_mibspi.h"
#include "rm57l843-launchxl2.h"

#ifdef CONFIG_RM57_MIBSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPICS pins claimed as chip selects, one bit per pin */

#define MIBSPI1_CSMASK      0x07    /* nCS[0..2] */
#define MIBSPI2_CSMASK      0x00
/* nCS[2] and nCS[3] share balls B2 and C3 with I2C1 */

#ifdef CONFIG_RM57_I2C1
#  define MIBSPI3_CSMASK    0x03    /* nCS[0..1] */
#else
#  define MIBSPI3_CSMASK    0x0f    /* nCS[0..3] */
#endif
#define MIBSPI4_CSMASK      0x00
#define MIBSPI5_CSMASK      0x00

/* Loopback self-test */

#define SPITEST_MAXWORDS    1000

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RM57_MIBSPI_LOOPBACK
static uint16_t g_spitest_tx[SPITEST_MAXWORDS];
static uint16_t g_spitest_rx[SPITEST_MAXWORDS];

/* Lengths chosen to sit on both sides of the 128- and 256-buffer limits of
 * the multi-buffer RAM, where a chunked transfer would have its seams.
 */

static const uint16_t g_spitest_lengths[] =
{
  1, 2, 3, 127, 128, 129, 255, 256, 257, SPITEST_MAXWORDS
};

static const uint32_t g_spitest_frequencies[] =
{
  20000, 100000, 400000, 1000000, 10000000, 37500000, 50000000
};

/* The patterns are run at both: in multi-buffer mode a pass at 1 MHz is
 * long enough to be waited for in the interrupt, one at 37.5 MHz is short
 * enough to be polled.
 */

static const uint32_t g_spitest_runfreqs[] =
{
  1000000, 37500000
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_spi_select
 *
 * Description:
 *   Common body of the per-bus select callbacks.
 *
 ****************************************************************************/

static void rm57_spi_select(int bus, uint8_t csmask, uint32_t devid,
                            bool selected)
{
  uint32_t cs = SPIDEVID_INDEX(devid);

  spiinfo("MibSPI%d devid %08" PRIx32 " %s\n", bus, devid,
          selected ? "assert" : "de-assert");

  if (cs < 8 && (csmask & (1 << cs)) != 0)
    {
      rm57_mibspi_cswrite(bus, cs, !selected);
    }
}

/****************************************************************************
 * Name: rm57_spi_looptest
 *
 * Description:
 *   With the modules in internal loopback, push known patterns through one
 *   bus and check that they come back, and report the frequency the driver
 *   settles on for a spread of requests.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_MIBSPI_LOOPBACK
static int rm57_spi_looptest(int bus, struct spi_dev_s *spi)
{
  uint8_t *tx8 = (uint8_t *)g_spitest_tx;
  uint8_t *rx8 = (uint8_t *)g_spitest_rx;
  uint32_t frequency;
  int failures = 0;
  int nbits;
  int run;
  int i;
  int j;

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);

  for (i = 0; i < nitems(g_spitest_frequencies); i++)
    {
      syslog(LOG_INFO, "MibSPI%d: %" PRIu32 " Hz requested, %" PRIu32
             " Hz set\n", bus, g_spitest_frequencies[i],
             SPI_SETFREQUENCY(spi, g_spitest_frequencies[i]));
    }

  /* Each frequency at 8 and then at 16 bits */

  for (run = 0; run < 2 * nitems(g_spitest_runfreqs); run++)
    {
      nbits     = (run & 1) != 0 ? 16 : 8;
      frequency = SPI_SETFREQUENCY(spi, g_spitest_runfreqs[run >> 1]);
      SPI_SETBITS(spi, nbits);

      for (i = 0; i < nitems(g_spitest_lengths); i++)
        {
          int nwords = g_spitest_lengths[i];
          int bad = 0;

          /* A pattern that changes with position and with the run, so a
           * word that is dropped, repeated or left over shows up.
           */

          for (j = 0; j < nwords; j++)
            {
              if (nbits == 8)
                {
                  tx8[j] = (uint8_t)(j * 7 + nwords);
                  rx8[j] = (uint8_t)~tx8[j];
                }
              else
                {
                  g_spitest_tx[j] = (uint16_t)(j * 257 + nwords);
                  g_spitest_rx[j] = (uint16_t)~g_spitest_tx[j];
                }
            }

          SPI_EXCHANGE(spi, g_spitest_tx, g_spitest_rx, nwords);

          for (j = 0; j < nwords; j++)
            {
              if (nbits == 8 ? tx8[j] != rx8[j] :
                               g_spitest_tx[j] != g_spitest_rx[j])
                {
                  bad++;
                }
            }

          syslog(bad != 0 ? LOG_ERR : LOG_INFO,
                 "MibSPI%d: loopback %8" PRIu32 " Hz %2d bit %4d words: "
                 "%s (%d bad)\n",
                 bus, frequency, nbits, nwords,
                 bad != 0 ? "FAIL" : "ok", bad);

          if (bad != 0)
            {
              failures++;
            }
        }
    }

  SPI_SETBITS(spi, 8);
  SPI_LOCK(spi, false);

  return failures == 0 ? OK : -EIO;
}
#endif

/****************************************************************************
 * Name: rm57_spi_bus
 *
 * Description:
 *   Initialize one bus, claim its chip selects and register it.
 *
 ****************************************************************************/

static int rm57_spi_bus(int bus, uint8_t csmask)
{
  struct spi_dev_s *spi;
  int ret;

  spi = rm57_mibspibus_initialize(bus);
  if (spi == NULL)
    {
      spierr("ERROR: MibSPI%d: initialization failed\n", bus);
      return -ENODEV;
    }

  ret = rm57_mibspi_csconfig(bus, csmask);
  if (ret < 0)
    {
      spierr("ERROR: MibSPI%d: chip select setup failed: %d\n", bus, ret);
      return ret;
    }

#ifdef CONFIG_RM57_MIBSPI_LOOPBACK
  /* Reported, not returned: a failed self-test should not take the rest of
   * the board bring-up down with it.
   */

  rm57_spi_looptest(bus, spi);
#endif

#ifdef CONFIG_SPI_DRIVER
  ret = spi_register(spi, bus);
  if (ret < 0)
    {
      spierr("ERROR: MibSPI%d: spi_register failed: %d\n", bus, ret);
      return ret;
    }
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_mibspiNselect, rm57_mibspiNstatus and rm57_mibspiNcmddata
 *
 * Description:
 *   The board half of each bus's spi_ops_s.  No device on this board
 *   reports status or needs a command/data line.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_MIBSPI1
void rm57_mibspi1select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  rm57_spi_select(1, MIBSPI1_CSMASK, devid, selected);
}

uint8_t rm57_mibspi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int rm57_mibspi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif

#ifdef CONFIG_RM57_MIBSPI2
void rm57_mibspi2select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  rm57_spi_select(2, MIBSPI2_CSMASK, devid, selected);
}

uint8_t rm57_mibspi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int rm57_mibspi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif

#ifdef CONFIG_RM57_MIBSPI3
void rm57_mibspi3select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  rm57_spi_select(3, MIBSPI3_CSMASK, devid, selected);
}

uint8_t rm57_mibspi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int rm57_mibspi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif

#ifdef CONFIG_RM57_MIBSPI4
void rm57_mibspi4select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  rm57_spi_select(4, MIBSPI4_CSMASK, devid, selected);
}

uint8_t rm57_mibspi4status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int rm57_mibspi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif

#ifdef CONFIG_RM57_MIBSPI5
void rm57_mibspi5select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  rm57_spi_select(5, MIBSPI5_CSMASK, devid, selected);
}

uint8_t rm57_mibspi5status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int rm57_mibspi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
#endif

/****************************************************************************
 * Name: rm57_spi_setup
 *
 * Description:
 *   Bring up every enabled MibSPI bus.  Called from rm57_bringup().
 *
 ****************************************************************************/

int rm57_spi_setup(void)
{
  int ret = OK;

#ifdef CONFIG_RM57_MIBSPI1
  ret = rm57_spi_bus(1, MIBSPI1_CSMASK);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_MIBSPI2
  ret = rm57_spi_bus(2, MIBSPI2_CSMASK);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_MIBSPI3
  ret = rm57_spi_bus(3, MIBSPI3_CSMASK);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_MIBSPI4
  ret = rm57_spi_bus(4, MIBSPI4_CSMASK);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_MIBSPI5
  ret = rm57_spi_bus(5, MIBSPI5_CSMASK);
  if (ret < 0)
    {
      return ret;
    }
#endif

  return ret;
}

#endif /* CONFIG_RM57_MIBSPI */
