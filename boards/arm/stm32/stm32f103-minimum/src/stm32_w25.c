/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_w25.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_STM32_SPI1
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/fs/smart.h>
#  include <nuttx/mtd/configdata.h>
#endif

#include "stm32_spi.h"

#include "stm32f103_minimum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog
 * timer
 */

#define W25_SPI_PORT 1

/* Configuration ************************************************************/
/* Can't support the W25 device if it SPI1 or W25 support is not enabled */

#define HAVE_W25  1
#if !defined(CONFIG_STM32_SPI1) || !defined(CONFIG_MTD_W25)
#  undef HAVE_W25
#endif

/* Can't support W25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_W25
#endif

/* Can't support both FAT and SMARTFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_SMARTFS)
#  warning "Can't support both FAT and SMARTFS -- using FAT"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Initialize and register the W25 FLASH file system.
 *
 ****************************************************************************/

int stm32_w25initialize(int minor)
{
  int ret;
#ifdef HAVE_W25
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  FAR struct mtd_geometry_s geo;
#if defined(CONFIG_MTD_PARTITION_NAMES)
  FAR const char *partname = CONFIG_STM32F103MINIMUM_FLASH_PART_NAMES;
#endif

  /* Get the SPI port */

  spi = stm32_spibus_initialize(W25_SPI_PORT);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
             W25_SPI_PORT);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the W25 SPI FLASH driver */

  mtd = w25_initialize(spi);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port %d to the Winbond"
                      "W25 FLASH driver\n", W25_SPI_PORT);
      return -ENODEV;
    }

#ifndef CONFIG_FS_SMARTFS
  /* And finally, use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(minor, mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Initialize the FTL layer\n");
      return ret;
    }
#else
  /* Initialize to provide SMARTFS on the MTD interface */

  /* Get the geometry of the FLASH device */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_STM32F103MINIMUM_FLASH_PART
  {
    int partno;
    int partsize;
    int partoffset;
    int partszbytes;
    int erasesize;
    const char *partstring = CONFIG_STM32F103MINIMUM_FLASH_PART_LIST;
    const char *ptr;
    FAR struct mtd_dev_s *mtd_part;
    char  partref[4];

    /* Now create a partition on the FLASH device */

    partno = 0;
    ptr = partstring;
    partoffset = 0;

    /* Get the Flash erase size */

    erasesize = geo.erasesize;

    while (*ptr != '\0')
      {
        /* Get the partition size */

        partsize = atoi(ptr);
        partszbytes = (partsize << 10); /* partsize is defined in KB */

        /* Check if partition size is bigger then erase block */

        if (partszbytes < erasesize)
          {
            syslog(LOG_ERR, "ERROR: Partition size is lesser than erasesize!\n");
            return -1;
          }

        /* Check if partition size is multiple of erase block */

        if ( (partszbytes % erasesize) != 0 )
          {
            syslog(LOG_ERR, "ERROR: Partition size is not multiple of erasesize!\n");
            return -1;
          }

        mtd_part = mtd_partition(mtd, partoffset,  partszbytes/ erasesize);
        partoffset += partszbytes / erasesize;

#ifdef CONFIG_STM32F103MINIMUM_FLASH_CONFIG_PART
        /* Test if this is the config partition */

        if (CONFIG_STM32F103MINIMUM_FLASH_CONFIG_PART_NUMBER == partno)
          {
            /* Register the partition as the config device */

            mtdconfig_register(mtd_part);
          }
        else
#endif
          {
            /* Now initialize a SMART Flash block device and bind it
             * to the MTD device.
             */

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
            sprintf(partref, "p%d", partno);
            smart_initialize(CONFIG_STM32F103MINIMUM_FLASH_MINOR, mtd_part, partref);
#endif
          }

        /* Set the partition name */

#if defined(CONFIG_MTD_PARTITION_NAMES)
        if (!mtd_part)
          {
            syslog(LOG_ERR, "Error: failed to create partition %s\n", partname);
            return -1;
          }

        mtd_setpartitionname(mtd_part, partname);

        /* Now skip to next name.  We don't need to split the string here
         * because the MTD partition logic will only display names up to
         * the comma, thus allowing us to use a single static name
         * in the code.
         */

        while (*partname != ',' && *partname != '\0')
          {
            /* Skip to next ',' */

            partname++;
          }

        if (*partname == ',')
          {
            partname++;
          }
#endif

        /* Update the pointer to point to the next size in the list */

        while ((*ptr >= '0') && (*ptr <= '9'))
          {
            ptr++;
          }

        if (*ptr == ',')
          {
            ptr++;
          }

        /* Increment the part number */

        partno++;
      }
  }
#else /* CONFIG_STM32F103MINIMUM_FLASH_PART */

  /* Configure the device with no partition support */

  smart_initialize(CONFIG_STM32F103MINIMUM_FLASH_MINOR, mtd, NULL);

#endif /* CONFIG_STM32F103MINIMUM_FLASH_PART */
#endif /* CONFIG_FS_SMARTFS */
#endif /* HAVE_W25 */

  return OK;
}
