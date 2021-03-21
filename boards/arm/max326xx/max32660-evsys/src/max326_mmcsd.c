/****************************************************************************
 * boards/arm/max326xx/max32660-evsys/src/max326_mmcsd.c
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

#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>

#include "max326_spim.h"

#ifdef CONFIG_MMCSD_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_MAX326XX_HAVE_SPIM
#  error "SD driver requires SPI master mode support"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

#if defined(CONFIG_MAX326XX_SPIM0)
#  define MMCSD_PORT 0
#elif defined(CONFIG_MAX326XX_SPIM1)
#  define MMCSD_PORT 1
#endif

#define MMCSD_SLOT 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef MICROSD_CD
/* NOTE:  We are using a SDCard adapter/module without Card Detect pin!
 * Then we don't need to Card Detect callback here.
 */

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
#  error "No card detect pin"
#endif
#else
#  error "Missing card detect logic"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_spiregister
 *
 * Description:
 *   Media change callback
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#if defined(CONFIG_MAX326XX_SPIM0)
int max326_spi0register(struct spi_dev_s *dev, spi_mediachange_t callback,
                        void *arg)
#elif defined(CONFIG_MAX326XX_SPIM1)
int max326_spi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                        void *arg)
#endif
{
  spiinfo("INFO:  SPI callback\n");
  return OK;
}
#endif

/****************************************************************************
 * Name: max326_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

int max326_mmcsd_initialize(int minor)
{
  struct spi_dev_s *spi;
  int rv;

  mcinfo("INFO: Initializing mmcsd card\n");

  spi = max326_spibus_initialize(MMCSD_PORT);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI port %d\n", MMCSD_PORT);
      return -ENODEV;
    }

  rv = mmcsd_spislotinitialize(minor, MMCSD_SLOT, spi);
  if (rv < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
            MMCSD_PORT, MMCSD_SLOT);
      return rv;
    }

  spiinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}

#endif /* CONFIG_MMCSD_SPI */
