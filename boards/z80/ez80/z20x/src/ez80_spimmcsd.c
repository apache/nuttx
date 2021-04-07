/****************************************************************************
 * boards/z80/ez80/z20x/src/ez80_spimmcsd.c
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

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>

#include "chip.h"
#include "ez80_spi.h"
#include "z20x.h"

#ifdef HAVE_MMCSD

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NOTES:
 *
 * 1. We are using a SDCard adapter/module without Card Detect pin!
 *    Then we don't need to Card Detect callback here.
 * 2. Media Change callbacks are not yet implemented in the SPI driver.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ez80_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card.
 *
 ****************************************************************************/

int ez80_mmcsd_initialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  mcinfo("INFO: Initializing mmcsd card\n");

  /* Get/initialize the SPI interface */

  spi = ez80_spibus_initialize(1);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI\n");
      return -ENODEV;
    }

  /* Register the MMC/SD block driver for
   * slot 0 with device minor number 0.
   */

  ret = mmcsd_spislotinitialize(0, 0, spi);
  if (ret < 0)
    {
      mcerr("ERROR: Failed to bind SPI to SD slot 0\n");
      return ret;
    }

  mcinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}

#endif /* HAVE_MMCSD */
