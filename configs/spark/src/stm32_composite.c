/****************************************************************************
 * config/spark/src/stm32_composite.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David_s5 <david_s5@nscdg.com>
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#ifdef CONFIG_MTD_SST25
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <sys/mount.h>
#endif

#ifdef CONFIG_SYSTEM_USBMONITOR
#  include <apps/usbmonitor.h>
#endif

#ifdef CONFIG_USBDEV
#  include "stm32_usbdev.h"
#endif

#include "stm32.h"
#include "spark.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Can't support the SST25 device if it SPI2 or SST25 support is not enabled */

#define HAVE_SST25  1
#if !defined(CONFIG_STM32_SPI2) || !defined(CONFIG_MTD_SST25)
#  undef HAVE_SST25
#endif

/* Can't support SST25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_SST25
#endif

#ifndef CONFIG_SPARK_FLASH_MOUNT_POINT
#  define CONFIG_SPARK_FLASH_MOUNT_POINT "/mnt/p%d"
#endif

/* Use minor device number 0 is not is provided */

#ifndef CONFIG_SPARK_FLASH_MINOR
#  define CONFIG_SPARK_FLASH_MINOR 0
#endif

/* Can't support both FAT and NXFFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_NXFFS)
#  warning "Can't support both FAT and NXFFS -- using FAT"
#endif

#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1

/* Can't support USB device is USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_SYSTEM_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_composite_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

static int do_composite_archinitialize(void)
{
#ifdef HAVE_SST25
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  int ret;

  /* Configure SPI-based devices */

  /* Get the SPI port */

  fvdbg("Initializing SPI port %d\n", CONFIG_SPARK_FLASH_SPI);

  spi = up_spiinitialize(CONFIG_SPARK_FLASH_SPI);
  if (!spi)
    {
      fdbg("ERROR: Failed to initialize SPI port %d\n",
           CONFIG_SPARK_FLASH_SPI);
      return -ENODEV;
    }

  fvdbg("Successfully initialized SPI port %d\n", CONFIG_SPARK_FLASH_SPI);

  /* Now bind the SPI interface to the SST25 SPI FLASH driver */

  fvdbg("Bind SPI to the SPI flash driver\n");
  mtd = sst25_initialize(spi);
  if (!mtd)
    {
      fdbg("ERROR: Failed to bind SPI port %d to the SPI FLASH driver\n",
           CONFIG_SPARK_FLASH_SPI);
    }
  else
    {
      fvdbg("Successfully bound SPI port %d to the SPI FLASH driver\n",
            CONFIG_SPARK_FLASH_SPI);
    }

#ifndef CONFIG_SPARK_FLASH_PART

  /* Use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(CONFIG_SPARK_FLASH_MINOR, mtd);
  if (ret < 0)
    {
      fdbg("ERROR: Initialize the FTL layer\n");
      return ret;
    }

#if CONFIG_SPARK_MOUNT_FLASH
  char  partname[16];
  char  mntpoint[16];

  /* mount -t vfat /dev/mtdblock0 /mnt/p0 */

  snprintf(partname, sizeof(partname), "/dev/mtdblock%d",
           CONFIG_SPARK_FLASH_MINOR);
  snprintf(mntpoint, sizeof(mntpoint)-1, CONFIG_SPARK_FLASH_MOUNT_POINT,
           CONFIG_SPARK_FLASH_MINOR);

  /* Mount the file system at /mnt/pn  */

  ret = mount(partname, mntpoint, "vfat", 0, NULL);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to mount the FAT volume: %d\n", errno);
      return ret;
    }

#endif
#else
    {
      int partno;
      int partsize;
      int partoffset;
      const char *partstring = CONFIG_SPARK_FLASH_PART_LIST;
      const char *ptr;
      FAR struct mtd_dev_s *mtd_part;
      char  partname[16];
      char  mntpoint[16];

      /* Now create a partition on the FLASH device */

      partno = CONFIG_SPARK_FLASH_MINOR;
      ptr = partstring;
      partoffset = 0;
      while (*ptr != '\0')
        {
          /* Get the partition size */

          partsize = atoi(ptr);
          mtd_part = mtd_partition(mtd, partoffset, (partsize >> 2) * 16);
          partoffset += (partsize >> 2) * 16;

          /* Use the FTL layer to wrap the MTD driver as a block driver */

          ret = ftl_initialize(partno, mtd_part);
          if (ret < 0)
            {
              fdbg("ERROR: Initialize the FTL layer\n");
              return ret;
            }

          snprintf(partname,sizeof(partname), "/dev/mtdblock%d", partno);
          snprintf(mntpoint,sizeof(mntpoint)-1, CONFIG_SPARK_FLASH_MOUNT_POINT,
                   partno);

          /* Mount the file system at /mnt/pn  */

          ret = mount(partname, mntpoint, "vfat", 0, NULL);
          if (ret < 0)
            {
              fdbg("ERROR: Failed to mount the FAT volume: %d\n", errno);
              return ret;
            }

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
#endif /* CONFIG_SPARK_FLASH_PART */

#endif /* HAVE_SST25 */

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start(0, NULL);
  if (ret != OK)
    {
      fdbg("ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int composite_archinitialize(void)
{
#if defined(CONFIG_NSH_ARCHINIT) && defined(CONFIG_NSH_BUILTIN_APPS)
  return OK;
#else
  return do_composite_archinitialize();
#endif
}
