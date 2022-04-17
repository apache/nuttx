/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_at45db.c
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

#include "stm32l4_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32L4_SPI1
#  error "AT45DB driver requires CONFIG_STM32_SPI1 to be enabled"
#endif

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

static const int AT45DB_SPI_PORT = 1; /* AT45DB is connected to SPI1 port */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_at45dbinitialize
 *
 * Description:
 *   Initialize and configure the AT45DB SPI Serial Flash Memory
 *
 ****************************************************************************/

int stm32_at45dbinitialize(int minor)
{
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the SPI bus driver */

      finfo("INFO: Initializing AT45DB\n");

      spi = stm32l4_spibus_initialize(AT45DB_SPI_PORT);
      if (spi == NULL)
        {
          ferr("ERROR: Failed to initialize SPI port %d\n", AT45DB_SPI_PORT);
          return -ENODEV;
        }

      mtd = at45db_initialize(spi);
      if (mtd == NULL)
        {
          ferr("ERROR: Failed to initialize AT45DB\n");
          return -ENODEV;
        }

#if defined(CONFIG_FS_NXFFS)
      /* Initialize to provide NXFFS on the MTD interface */

      finfo("Initialize the NXFFS file system\n");
      ret = nxffs_initialize(mtd);
      if (ret < 0)
        {
          ferr("ERROR: NXFFS initialization failed: %d\n", ret);
          return ret;
        }

      /* Mount the file system at /mnt/at45db */

      finfo("Mount the NXFFS file system at /dev/at45db\n");
      ret = nx_mount(NULL, "/mnt/at45db", "nxffs", 0, NULL);
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
