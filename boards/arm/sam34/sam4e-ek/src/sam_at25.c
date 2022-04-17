/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_at25.c
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

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include "sam_spi.h"
#include "sam4e-ek.h"

#ifdef HAVE_AT25

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_at25_automount
 *
 * Description:
 *   Initialize, configure, and mount the AT25 serial FLASH.  The FLASH will
 *   be mounted at /dev/at25.
 *
 ****************************************************************************/

int sam_at25_automount(int minor)
{
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the SPI port driver */

      spi = sam_spibus_initialize(FLASH_CSNUM);
      if (!spi)
        {
          ferr("ERROR: Failed to initialize SPI port %d\n", FLASH_CSNUM);
          return -ENODEV;
        }

      /* Now bind the SPI interface to the AT25 SPI FLASH driver */

      mtd = at25_initialize(spi);
      if (!mtd)
        {
          ferr("ERROR: Failed to bind SPI port to the AT25 FLASH driver\n");
          return -ENODEV;
        }

#if defined(CONFIG_SAM4EEK_AT25_FTL)
      /* And finally, use the FTL layer to wrap the MTD driver as a block
       * driver at /dev/mtdblockN, where N=minor device number.
       */

      ret = ftl_initialize(minor, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }

#elif defined(CONFIG_SAM4EEK_AT25_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          ferr("ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Mount the file system at /mnt/at25 */

      ret = nx_mount(NULL, "/mnt/at25", "nxffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* HAVE_AT25 */
