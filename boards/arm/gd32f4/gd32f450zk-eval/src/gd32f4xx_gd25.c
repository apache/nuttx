/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/src/gd32f4xx_gd25.c
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

#include "gd32f4xx.h"
#include "gd32f450z_eval.h"

#ifdef HAVE_GD25

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gd25_automount
 *
 * Description:
 *   Initialize, configure, and mount the GD25 SPI FLASH.  The FLASH will
 *   be mounted at /dev/gd25.
 *
 ****************************************************************************/

int gd32_gd25_automount(int minor)
{
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the SPI port driver */

      spi = gd32_spibus_initialize(SPI_FLASH_CSNUM);
      if (!spi)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
                 SPI_FLASH_CSNUM);
          return -ENODEV;
        }

      /* Now bind the SPI interface to the GD25 SPI FLASH driver */

      mtd = gd25_initialize(spi, 0);
      if (!mtd)
        {
          syslog(LOG_ERR, "ERROR: Failed to bind SPI port to the \
                 GD25 FLASH driver\n");
          return -ENODEV;
        }

#if defined(CONFIG_GD32F450ZK_EVAL_GD25_FTL)
      /* And finally, use the FTL layer to wrap the MTD driver as a block
       * driver at /dev/mtdblockN, where N=minor device number.
       */

      ret = ftl_initialize(minor, mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n",
                 ret);
          return ret;
        }

#elif defined(CONFIG_GD32F450ZK_EVAL_GD25_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Mount the file system at /mnt/gd25 */

      ret = nx_mount(NULL, "/mnt/gd25", "nxffs", 0, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to mount the NXFFS volume: %d\n",
                 ret);
          return ret;
        }

      syslog(LOG_INFO, "INFO: NXFFS volume /mnt/gd25 mount \
            spi flash success: %d\n", ret);
#endif

      /* Now we are initialized */

      initialized = true;
    }

  UNUSED(ret);

  return OK;
}

#endif /* HAVE_GD25 */
