/****************************************************************************
 * boards/sparc/bm3823/xx3823/src/bm3823_am29lv.c
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

#include <sys/mount.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_MTD_AM29LV
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/fs/smart.h>
#  include <nuttx/mtd/configdata.h>
#endif


#include "xx3823.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* Can't support the W25 device if it SPI1 or W25 support is not enabled */

#define HAVE_AM29LV  1
#if !defined(CONFIG_MTD_AM29LV)
#  undef HAVE_AM29LV
#endif

/* Can't support W25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_AM29LV
#endif

/* Can't support both FAT and SMARTFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_SMARTFS)
#  warning "Can't support both FAT and SMARTFS -- using FAT"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3823_w25initialize
 *
 * Description:
 *   Initialize and register the W25 FLASH file system.
 *
 ****************************************************************************/

int bm3823_am29lv_initialize(int minor)
{
  int ret;
#ifdef HAVE_AM29LV
  FAR struct mtd_dev_s *mtd;
  FAR struct mtd_geometry_s geo;
#if defined(CONFIG_MTD_PARTITION_NAMES)
  FAR const char *partname = CONFIG_XX3823_FLASH_PART_NAMES;
#endif

  /* Now bind the SPI interface to the W25 SPI FLASH driver */

  mtd = am29lv_initialize();
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize AM29LV FLASH driver\n");
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

#ifdef CONFIG_XX3823_FLASH_PART
  {
    int partno;
    int partsize;
    int partoffset;
    int partszbytes;
    int erasesize;
    const char *partstring = CONFIG_XX3823_FLASH_PART_LIST;
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

#ifdef CONFIG_XX3823_FLASH_CONFIG_PART
        /* Test if this is the config partition */

        if (CONFIG_XX3823_FLASH_CONFIG_PART_NUMBER == partno)
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
            smart_initialize(CONFIG_XX3823_FLASH_MINOR, mtd_part, partref);
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
#else /* CONFIG_XX3823_FLASH_PART */

  /* Configure the device with no partition support */

  smart_initialize(CONFIG_XX3823_FLASH_MINOR, mtd, NULL);

#endif /* CONFIG_XX3823_FLASH_PART */
#endif /* CONFIG_FS_SMARTFS */
#endif /* HAVE_AM29LV */

  /* Create a RAM MTD device if configured */

#if defined(CONFIG_RAMMTD) && defined(CONFIG_XX3823_RAMMTD)
  {
    uint8_t *start = (uint8_t *)kmm_malloc(CONFIG_XX3823_RAMMTD_SIZE * 1024);
    mtd = rammtd_initialize(start, CONFIG_XX3823_RAMMTD_SIZE * 1024);
    mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);

#if defined(CONFIG_MTD_SMART) && defined(CONFIG_FS_SMARTFS)
    /* Now initialize a SMART Flash block device and bind it to the MTD device */

    smart_initialize(CONFIG_XX3823_RAMMTD_MINOR, mtd, NULL);
#endif
  }

#endif /* CONFIG_RAMMTD && CONFIG_XX3823_RAMMTD */

  return OK;
}
