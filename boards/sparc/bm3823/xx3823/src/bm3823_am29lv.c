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
  struct mtd_dev_s *mtd;
  struct mtd_geometry_s geo;
#if defined(CONFIG_MTD_PARTITION_NAMES)
  const char *partname = CONFIG_XX3823_FLASH_PART_NAMES;
#endif

  /* Now bind the SPI interface to the W25 SPI FLASH driver */

  mtd = am29lv_initialize();
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize AM29LV FLASH driver\n");
      return -ENODEV;
    }

#endif /* HAVE_AM29LV */

  /* Create a RAM MTD device if configured */

#if defined(CONFIG_RAMMTD) && defined(CONFIG_XX3823_RAMMTD)

  uint8_t *start = (uint8_t *)kmm_malloc(CONFIG_XX3823_RAMMTD_SIZE * 1024);
  mtd = rammtd_initialize(start, CONFIG_XX3823_RAMMTD_SIZE * 1024);
  mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);

#endif /* CONFIG_RAMMTD && CONFIG_XX3823_RAMMTD */

  return OK;
}
