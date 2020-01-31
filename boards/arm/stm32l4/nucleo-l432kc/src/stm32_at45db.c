/*******************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_at45.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ******************************************************************************/

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>

#include "stm32l4_spi.h"

/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#ifndef CONFIG_STM32L4_SPI1
#  error "AT45DB driver requires CONFIG_STM32_SPI1 to be enabled"
#endif

/*******************************************************************************
* Private Definitions
******************************************************************************/

static const int AT45DB_SPI_PORT = 1; /* AT45DB is connected to SPI1 port */

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*******************************************************************************
 * Name: stm32_at45dbinitialize
 *
 * Description:
 *   Initialize and configure the AT45DB SPI Serial Flash Memory
 *
 ******************************************************************************/

int stm32_at45dbinitialize(int minor)
{
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
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
      ret = mount(NULL, "/mnt/at45db", "nxffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}
