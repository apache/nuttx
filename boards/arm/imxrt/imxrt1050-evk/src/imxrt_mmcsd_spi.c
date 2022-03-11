/****************************************************************************
 * boards/arm/imxrt/imxrt1050-evk/src/imxrt_mmcsd_spi.c
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

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/mmcsd.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "imxrt_lpspi.h"

#ifdef CONFIG_MMCSD_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_IMXRT_LPSPI1
#  error "SD driver requires CONFIG_IMXRT_LPSPI1 to be enabled"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

static const int SD_SPI_PORT = CONFIG_NSH_MMCSDSPIPORTNO; /* SD is connected to SPI1 port */

static const int SD_SLOT_NO  = 0; /* There is only one SD slot */

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
 * Name: imxrt_spi1register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPSPI1
int imxrt_lpspi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  spiinfo("INFO: Registering spi1 device\n");
  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_mmcsd_spi_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

int imxrt_mmcsd_spi_initialize(int minor)
{
  struct spi_dev_s *spi;
  int rv;

  mcinfo("INFO: Initializing mmcsd card\n");

  spi = imxrt_lpspibus_initialize(SD_SPI_PORT);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI port %d\n", SD_SPI_PORT);
      return -ENODEV;
    }

  rv = mmcsd_spislotinitialize(minor, SD_SLOT_NO, spi);
  if (rv < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
            SD_SPI_PORT, SD_SLOT_NO);
      return rv;
    }

  spiinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}

#endif /* CONFIG_MMCSD_SPI */
